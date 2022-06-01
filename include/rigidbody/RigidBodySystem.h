#pragma once

#include <Eigen/Dense>

#include <memory>
#include <vector>

class Contact;
class CollisionDetect;
class Solver;
class RigidBody;

typedef std::function<void(std::vector<RigidBody*>&)> PreStepFunc;
typedef std::function<void()> ResetFunc;

class RigidBodySystem
{
public:

    RigidBodySystem();

    virtual ~RigidBodySystem();

    // Advance the simulation.
    void step(float _dt);

    // Remove all rigid bodies and cleanup the memory.
    void clear();

    // Add rigid body @a _b to the system. The rigid body is owned and managed by this RigidBodySystem.
    void addBody(RigidBody* _b);

    // Accessors for the body array.
    const std::vector<RigidBody*>& getBodies() const { return m_bodies; }
    std::vector<RigidBody*>& getBodies() { return m_bodies; }

    // Accessors for the contact array.
    const std::vector<Contact*>& getContacts() const;
    std::vector<Contact*>& getContacts();

    // Callbacks.
    void setPreStepFunc(PreStepFunc _func) { m_preStepFunc = _func; }
    void setResetFunc(ResetFunc _func) { m_resetFunc = _func; }

    float contactStiffness;     // Contact stiffness, used for computing Baumgarte stabilization parameters.
    float contactDamping;       // Contact damping, used for computing Baumgarte stabilization parameters.
    float mu;                   // Coefficient of friction for all bodies in the system
    int solverIter;             // Number of solver iterations

private:

    std::vector<RigidBody*> m_bodies;
    std::unique_ptr<CollisionDetect> m_collisionDetect;
    Solver* m_solver;

    // Compute the world-space inverse inertia matrices for all bodies.
    //  This function also updates the rotation matrices using the quaternions.
    void computeInertias();

    // Compute the constraint forces.
    void calcConstraintForces(float dt);

    PreStepFunc m_preStepFunc;      // Callback function that is called before body positions and orientations are updated.
    ResetFunc m_resetFunc;          // Callback for simulation reset.
};


template<typename ScalarType, int Options>
Eigen::Quaternion<ScalarType, Options> operator*(ScalarType a, const Eigen::Quaternion<ScalarType, Options>& q)
{
    return Eigen::Quaternion<ScalarType, Options>(a*q.w(),a*q.x(),a*q.y(),a*q.z());
}

template<typename ScalarType, int Options>
Eigen::Quaternion<ScalarType, Options> operator+(const Eigen::Quaternion<ScalarType, Options>& q1, const Eigen::Quaternion<ScalarType, Options>& q2)
{
    return Eigen::Quaternion<ScalarType, Options>(q1.w()+q2.w(),q1.x()+q2.x(),q1.y()+q2.y(),q1.z()+q2.z());
}
