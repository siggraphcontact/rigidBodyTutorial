#include "solvers/SolverBoxPGS.h"

#include "contact/Contact.h"
#include "rigidbody/RigidBody.h"
#include "rigidbody/RigidBodySystem.h"

#include <Eigen/Dense>


SolverBoxPGS::SolverBoxPGS(RigidBodySystem* _rigidBodySystem) : Solver(_rigidBodySystem)
{

}

void SolverBoxPGS::solve(float h)
{
    // TODO Implement a PGS method that solves for the
    //      contact constraint forces in @a rigidBodySystem. 
    //      Assume a boxed LCP formulation.
    //
    std::vector<Contact*>& contacts = m_rigidBodySystem->getContacts();
    const int numContacts = contacts.size();

    // Build array of 3x3 diagonal matrices, one for each contact.
    // 
    std::vector<Eigen::Matrix3f> Acontactii;
    if( numContacts > 0 )
    {

        // TODO Compute the right-hand side vector : b = -gamma*phi/h - J*vel - dt*JMinvJT*force
        //

        // PGS main loop.
        // There is no convergence test here. Simply stop after @a maxIter iterations.
        //
        for(int iter = 0; iter < m_maxIter; ++iter)
        {
            // TODO For each contact, compute an updated value of contacts[i]->lambda
            //      using matrix-free pseudo-code provided in the course appendix.
            //
            for(int i = 0; i < numContacts; ++i)
            {
                // TODO initialize current solution as x = b(i)
                // 

                // TODO Loop over all other contacts involving c->body0
                //      and accumulate : x -= (J0*Minv0*Jother^T) * lambda_other
                //


                // TODO Loop over all other contacts involving c->body1
                //      and accumulate : x -= (J0*Minv0*Jother^T) * lambda_other
                //


                // TODO Compute the diagonal term : Aii = J0*Minv0*J0^T + J1*Minv1*J1^T
                //


                // TODO Update lambda by solving the sub-problem : Aii * contacts[i].lambda = x
                //      For non-interpenetration constraints, lambda is non-negative and should be clamped
                //      to the range (0... infinity).
                //      Otherwise friction constraints should be clamped to the range (-mu*lambda_n, mu*lambda_n)
                //      where lambda_n is the impulse of the corresponding non-interpentration constraint.
                //

            }
        }
    }
}

