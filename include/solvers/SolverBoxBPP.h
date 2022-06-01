#pragma once

#include "solvers/Solver.h"

// Block Principal Pivoting (BPP) Boxed LCP solver.
//
class SolverBoxBPP : public Solver
{
public:
    SolverBoxBPP(RigidBodySystem* _rigidBodySystem);

    // Implement Block Principal Pivoting method.
    //
    virtual void solve(float h);

};
