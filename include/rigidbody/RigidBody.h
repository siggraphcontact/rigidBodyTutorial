#pragma once

#include "collision/Geometry.h"
#include <Eigen/Dense>
#include <vector>

class Contact;
struct Mesh;

namespace polyscope
{
    class SurfaceMesh;
}


// Rigid body class.
// Stores properties for rigid body simulation, including
// the geometry and list of contact constraints.
//
class RigidBody
{
public:
    RigidBody(float _mass, Geometry* _geometry, const std::string& meshFilename = "");

    static int counter;

    void updateInertiaMatrix();
    void addForceAtPos(const Eigen::Vector3f& pos, const Eigen::Vector3f& force);
    void getVelocityAtPos(const Eigen::Vector3f& pos, Eigen::Vector3f& vel);

    bool fixed;                         // Flag for a static rigid body. Default is 'false'.
    float mass;                         // Mass.
    Eigen::Matrix3f I, Iinv;            // Inertia and inverse inertia matrix (global)
    Eigen::Matrix3f Ibody, IbodyInv;    // Inertia and inverse inertia in the local body frame.
    Eigen::Vector3f x;                  // Position.
    Eigen::Quaternionf q;               // Orientation.

    Eigen::Vector3f xdot;               // Linear velocity.
    Eigen::Vector3f omega;              // Angular velocity.
    Eigen::Vector3f f;                  // Linear force.
    Eigen::Vector3f tau;                // Angular force (torque).

    Eigen::Vector3f fc;                 // Linear constraint force.
    Eigen::Vector3f tauc;               // Angular constraint force

    std::unique_ptr<Geometry> geometry; // The geometry of the rigid body.
    std::vector<Contact*> contacts;     // Pointer array of contact constraints involving this body.

    polyscope::SurfaceMesh* mesh;                         // Used for rendering


};
