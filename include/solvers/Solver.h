#pragma once

class RigidBodySystem;

// Contact solver interface.
//
class Solver
{
public:

    // Constructor.
    // Inputs: 
    //    _rigidBodySystem - The rigid body system.
    //
    Solver(RigidBodySystem* _rigidBodySystem) : m_rigidBodySystem(_rigidBodySystem), m_maxIter(20) { }

    // Sets the maximum number of iterations for the solver method.
    // 
    // The behavior will change based on the specific algorithm.
    // e.g., for PGS, this changes the number of iterations
    // for the BPP algorithm, this changes the number of pivoting step.
    //
    void setMaxIter(int _maxIter) { m_maxIter = _maxIter; }

    // Returns the max iterations.
    //
    int getMaterIter() const { return m_maxIter; }

    // The method that solves for the constraint forces in @a m_rigidBodySystem.
    // 
    // Inputs:
    //    h - the simulation time step.
    // Output:
    //   Constraint forces are stored in Contact::lambda.
    //
    virtual void solve(float h) = 0;

protected:

    RigidBodySystem* m_rigidBodySystem;
    int m_maxIter;
};
