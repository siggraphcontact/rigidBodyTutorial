#include "solvers/SolverBoxBPP.h"

#include "rigidbody/RigidBodySystem.h"
#include "rigidbody/RigidBody.h"
#include "contact/Contact.h"

#include <Eigen/Dense>

namespace
{
    enum eIndexSet { kFree = 0, kLower, kUpper, kIgnore };

    static inline void multAndSub(const JBlock& G, const Eigen::Vector3f& x, const Eigen::Vector3f& y, const float a, Eigen::Ref<Eigen::VectorXf> b)
    {
            b -= a * G.col(0) * x(0);
            b -= a * G.col(1) * x(1);
            b -= a * G.col(2) * x(2);
            b -= a * G.col(3) * y(0);
            b -= a * G.col(4) * y(1);
            b -= a * G.col(5) * y(2);
    }

    static inline unsigned int initContacts(std::vector<Contact*>& contacts)
    {
        // Count the number of constraint rows
        // and assign each contact an index.
        //
        unsigned int rows = 0;
        for(auto c : contacts)
        {
            c->index = rows;
            rows += c->J0.rows();
        }

        return rows;
    }

    // Update the box bounds, lower and upper, of the constraint impulses.
    // The value in Contact::lambda is used for updating the bounds.
    //
    static inline void updateBounds(std::vector<Contact*>& contacts, Eigen::VectorXf& lower, Eigen::VectorXf& upper)
    {
        for(auto c : contacts)
        {
            const unsigned int dim = c->J0.rows();
            // Non-interpenetration row.
            //
            lower(c->index) = 0.0f;
            upper(c->index) = std::numeric_limits<float>::max();

            // Friction rows.
            // Compute the box bounds as [-mu*lambda_n, mu*lambda_n]
            // which is an approximation of the isotropic Coulomb friction cone.
            //
            for(unsigned int i = 1; i < dim; ++i)
            {
                lower(c->index + i) = -c->mu * c->lambda(0);
                upper(c->index + i) = c->mu * c->lambda(0);
            }
        }
    }

    // Build the rhs vector of the Schur complement linear system.
    //
    static inline void buildRHS(const std::vector<Contact*>& contacts, float h, Eigen::VectorXf& b)
    {
        for(auto c : contacts)
        {
            const float gamma = h * c->k / (h * c->k + c->b); // error reduction parameter
            const unsigned int dim = c->J0.rows();
            b.segment(c->index, dim) = -gamma * c->phi / h;

            multAndSub(c->J0, c->body0->xdot, c->body0->omega, 1.0f, b.segment(c->index, dim));
            multAndSub(c->J1, c->body1->xdot, c->body1->omega, 1.0f, b.segment(c->index, dim));

            if( !c->body0->fixed )
            {
                multAndSub(c->J0Minv, c->body0->f, c->body0->tau, h, b.segment(c->index, dim));
            }
            if( !c->body1->fixed )
            {
                multAndSub(c->J1Minv, c->body1->f, c->body1->tau, h, b.segment(c->index, dim));
            }
        }
    }

    // Build the Schur complement system using the contact constraints.
    //
    static inline void buildMatrix(const std::vector<Contact*>& contacts, float h, Eigen::MatrixXf& A)
    {
        for(auto c : contacts)
        {
            const unsigned int dim = c->J0.rows();
            const float eps = 1.0f / (h * h * c->k + h * c->b);    // constraint force mixing

            A.block(c->index, c->index, dim, dim) = 1e-10f * Eigen::MatrixXf::Identity(dim, dim);
            A(c->index, c->index) += eps;

            if( !c->body0->fixed )
            {
                A.block(c->index, c->index, dim, dim) += c->J0Minv * c->J0.transpose();
                for(auto cc : c->body0->contacts)
                {
                    if( cc != c )
                    {
                        const int ddim = c->J0.rows();
                        if( cc->body0 == c->body0 )
                        {
                            A.block(c->index, cc->index, dim, ddim) += c->J0Minv * cc->J0.transpose();
                        }
                        else
                        {
                            A.block(c->index, cc->index, dim, ddim) += c->J0Minv * cc->J1.transpose();
                        }
                    }
                }
            }


            if( !c->body1->fixed )
            {
                A.block(c->index, c->index, dim, dim) += c->J1Minv * c->J1.transpose();

                for(auto cc : c->body1->contacts)
                {
                    if( cc != c )
                    {
                        const int ddim = c->J1.rows();
                        if( cc->body0 == c->body1 )
                        {
                            A.block(c->index, cc->index, dim, ddim) += c->J1Minv * cc->J0.transpose();
                        }
                        else
                        {
                            A.block(c->index, cc->index, dim, ddim) += c->J1Minv * cc->J1.transpose();
                        }
                    }
                }
            }
        }

    }

    // Pivoting rules.
    // The index set is updates based on the LCP variables x and v.
    // Specifically, 'free' variables are pivoted to the 'tight' set if the lower or upper bounds are violated.
    // Similarly, 'tight' variables are pivoted to the 'free' set if the residual velocity v has the wrong sign.
    // 
    static inline unsigned int pivot(Eigen::VectorXi& idx, Eigen::VectorXf& x, const Eigen::VectorXf& l, const Eigen::VectorXf& u, const Eigen::VectorXf& v)
    {
        static const float tol = 1e-5f;
        unsigned int numPivots = 0;
        const unsigned int n = idx.rows();
        for (unsigned int j = 0; j < n; ++j)
        {
            // By default, assume all variables belong to the 'free' set.
            //
            int new_idx = kFree;  

            if (idx[j] == kIgnore) continue;

            if(idx[j] == kFree && x[j] <= l[j])     // case: free variable, lower bound
            {
                new_idx = kLower;
            }
            else if(idx[j] == kFree && x[j] >= u[j])  // case: free variable, upper bound
            {
                new_idx = kUpper;
            }
            else if(idx[j] == kLower && v[j] > -tol)    // case: lower tight variable, +ve velocity
            {
                new_idx = kLower;
            }
            else if(idx[j] == kUpper && v[j] < tol)     // case: upper tight variable, -ve velocity
            {
                new_idx = kUpper;
            }

            if(new_idx != idx[j])
            {
                ++numPivots;
                idx[j] = new_idx;
            }
        }
        return numPivots;
    }

    // Returns the indices of tight variables in tightIdx.
    //
    static inline unsigned int tightIndices(const Eigen::VectorXi& idx, std::vector<int>& tightIdx)
    {
        const unsigned int n = idx.rows();
        unsigned int numTight = 0;
        tightIdx.clear();
        for(unsigned int i = 0; i < n; ++i)
        {
            if( idx[i] == kLower || idx[i] == kUpper )
            {
                ++numTight;
                tightIdx.push_back(i);
            }
        }
        return numTight;
    }

    // Returns the indices of free variables in freeIdx.
    //
    static inline unsigned int freeIndices(const Eigen::VectorXi& idx, std::vector<int>& freeIdx)
    {
        const unsigned int n = idx.rows();
        unsigned int numFree = 0;
        freeIdx.clear();
        for(unsigned int i = 0; i < n; ++i)
        {
            if( idx[i] == kFree )
            {
                ++numFree;
                freeIdx.push_back(i);
            }
        }
        return numFree;
    }

    // Solve the principal sub-problem comprising the free variables:
    //      Aff * xf = bf - Aft * xt
    // 
    //  where Aff is the sub-matrix of free variables, bf are the corresponding entries in the rhs vector,
    //  Aft is the sub-matrix that couples the tight variables and the free variables, and 
    //  xt are the tight variables whose values is determined by the lower and upper bounds (l and u).
    //
    // Inputs: 
    //    A - the lead matrix
    //    b - the rhs vector
    //    idx - the index set of all variables
    //    l - the lower bounds
    //    u - the upper bounds
    //
    // Outputs:
    //    x - the solution of constraint impulses (free and tight variables)
    //
    static inline void solvePrincipalSubproblem(const Eigen::MatrixXf& A,
                                                const Eigen::VectorXf& b,
                                                const Eigen::VectorXi& idx,
                                                const Eigen::VectorXf& l,
                                                const Eigen::VectorXf& u,
                                                Eigen::VectorXf& x)
    {
        std::vector<int> freeIdx, tightIdx;
        const unsigned int numTight = tightIndices(idx, tightIdx);
        const unsigned int numFree = freeIndices(idx, freeIdx);
        if( numFree > 0 )
        {
            Eigen::MatrixXf Aff(numFree, numFree);
            Eigen::VectorXf bf(numFree);

            // Build sub-matrix using free indices
            //
            for(unsigned int j = 0; j < numFree; ++j)
            {
                for(unsigned int i = 0; i < numFree; ++i)
                {
                    Aff(i,j) = A(freeIdx[i], freeIdx[j]);
                }
            }

            // Build rhs vector using free indices
            //
            for (unsigned int i = 0; i < numFree; ++i)
            {
                bf(i) = b(freeIdx[i]);
            }

            // Update rhs vector with impulses from tight indices
            //  e.g.     bf -= A_ft * x_t
            //
            for(unsigned int j = 0; j < numTight; ++j)
            {
                for(unsigned int i = 0; i < numFree; ++i)
                {
                    if( idx[tightIdx[j]] == kLower )
                    {
                        bf(i) -= A(freeIdx[i], tightIdx[j]) * l(tightIdx[j]);
                    }
                    else if( idx[tightIdx[j]] == kUpper )
                    {
                        bf(i) -= A(freeIdx[i], tightIdx[j]) * u(tightIdx[j]);
                    }
                }
            }

            // Cholesky solve for the principal sub-problem.
            //
            Eigen::LDLT<Eigen::MatrixXf> ldlt(Aff);
            const Eigen::VectorXf xf = ldlt.solve(bf);

            // Update free variables with the solution
            for(unsigned int i = 0; i < numFree; ++i)
            {
                x(freeIdx[i]) = xf(i);
            }

            // Update tight variables with values from lower/upper bounds.
            //
            for(unsigned int i = 0; i < numTight; ++i)
            {
                if( idx[tightIdx[i]] == kLower )
                {
                    x(tightIdx[i]) = l(tightIdx[i]);
                }
                else if( idx[tightIdx[i]] == kUpper )
                {
                    x(tightIdx[i]) = u(tightIdx[i]);
                }
            }
        }

    }

    static inline void updateContacts(const Eigen::VectorXf& x, std::vector<Contact*>& contacts)
    {
        // Distribute impulses to the contacts
        //
        for(auto c : contacts)
        {
            const unsigned int dim = c->J0.rows();
            c->lambda = x.segment(c->index, dim);
        }
    }

}


SolverBoxBPP::SolverBoxBPP(RigidBodySystem* _rigidBodySystem) : Solver(_rigidBodySystem)
{

}

void SolverBoxBPP::solve(float h)
{
    auto contacts = m_rigidBodySystem->getContacts();

    const unsigned int numContacts = contacts.size();
    if( numContacts > 0 )
    {
        const unsigned int dim = initContacts(contacts);

        Eigen::MatrixXf A = Eigen::MatrixXf::Zero(dim,dim);
        Eigen::VectorXf b = Eigen::VectorXf::Zero(dim);
        Eigen::VectorXf x = Eigen::VectorXf::Zero(dim);

        // Update box bounds.
        //
        Eigen::VectorXf lower = Eigen::VectorXf::Zero(dim);
        Eigen::VectorXf upper = Eigen::VectorXf::Zero(dim);

		updateBounds(contacts, lower, upper);

		// Initialize the index set.
        // All variables are initially set to 'free'.
        //
		Eigen::VectorXi idx = Eigen::VectorXi::Constant(dim, kFree);

		// Ignore variables where lower and upper are zero.
		//
		for (int i = 0; i < dim; ++i)
		{
			static const float tol = 1e-5f;
			if (std::abs(upper(i)) < tol && std::abs(lower(i) < tol))
				idx(i) = kIgnore;
		}

        // Construct the lead matrix and rhs vector.
        //
        buildMatrix(contacts, h, A);
        buildRHS(contacts, h, b);

        // Perform an initial solve to update friction box bounds.
        // 
        solvePrincipalSubproblem(A, b, idx, lower, upper, x);
        updateContacts(x, contacts);
        updateBounds(contacts, lower, upper);
        idx.setConstant(kFree);

        // Block pivoting iterations.
        //
        for(unsigned int iter = 0; iter < m_maxIter; ++iter)
        {
            // Solve the principal sub-problem:
            //    Aff * xf = bf - Aft * xt
            //
            solvePrincipalSubproblem(A, b, idx, lower, upper, x);

            // Compute residual velocity v.
            //
            const Eigen::VectorXf v = A*x - b;

            // Pivot.
            // 
            const int numPivots = pivot(idx, x, lower, upper, v);
            if( numPivots == 0 )
                break;      // Done
        }

        updateContacts(x, contacts);
        updateBounds(contacts, lower, upper);
    }
}

