#include "rigidbody/RigidBody.h"
#include "contact/Contact.h"
#include "util/MeshAssets.h"
#include "polyscope/surface_mesh.h"

int RigidBody::counter = 0;

RigidBody::RigidBody(float _mass, Geometry* _geometry, const std::string& _filename) :
    fixed(false),
    mass(_mass),
    x(0,0,0),
    //R(Eigen::Matrix3f::Identity()),
    xdot(0,0,0),
    omega(0,0,0),
    q(1,0,0,0),
    Ibody(Eigen::Matrix3f::Identity()),
    IbodyInv(Eigen::Matrix3f::Identity()),
    Iinv(Eigen::Matrix3f::Zero()),
    f(0,0,0),
    tau(0,0,0),
    fc(0,0,0),
    tauc(0,0,0),
    geometry(_geometry),
    contacts(),
    mesh(nullptr)
{
    Ibody = geometry->computeInertia(mass);
    IbodyInv = Ibody.inverse();

    if( !_filename.empty() )
    {
        // Read the mesh using the asset registry, which caches previously loaded meshes.
        auto* cachedMesh = MeshAssetRegistry::loadObj(_filename);

        if (cachedMesh != nullptr)
        {
            // Register the mesh with Polyscope
            mesh = polyscope::registerSurfaceMesh(std::to_string(RigidBody::counter), cachedMesh->meshV, cachedMesh->meshF);
            mesh->setSmoothShade(true);
        }
    }
    contacts.clear();
    RigidBody::counter++;
}


void RigidBody::updateInertiaMatrix()
{
    if( !fixed )
    {
        I = q * Ibody * q.inverse();
        Iinv = q * IbodyInv * q.inverse();
    }
    else
    {
        Iinv.setZero();
    }
}

void RigidBody::addForceAtPos(const Eigen::Vector3f& pos, const Eigen::Vector3f& force)
{
    const Eigen::Vector3f r = pos - x;
    f += force;
    tau += r.cross(force);
}

void RigidBody::getVelocityAtPos(const Eigen::Vector3f& pos, Eigen::Vector3f& vel)
{
    const Eigen::Vector3f r = pos - x;
    vel = xdot + r.cross(omega);
}
