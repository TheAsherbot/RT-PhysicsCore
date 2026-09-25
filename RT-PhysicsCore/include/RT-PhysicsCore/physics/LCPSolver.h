#pragma once

#include <vector>

namespace RT_PhysicsCore
{
    // Solves the Linear Complementarity Problem
    //   w = M*z + q,  w >= 0,  z >= 0,  z . w = 0
    // via Lemke's algorithm: tableau pivoting with a unit covering vector
    // and Bland's-rule tie-breaking in the minimum-ratio test (guards
    // against cycling on degenerate ties without the cost of full
    // lexicographic ordering).
    //
    // Written for the symmetric positive-semidefinite effective-mass
    // matrices rigid body contact problems produce - that's exactly the
    // condition that makes Lemke's algorithm well-behaved and guaranteed
    // to terminate here. It isn't a general-purpose LCP solver for
    // arbitrary M.
    //
    // maxPivots caps the pivot count (0 = a size-based default). Returns
    // false - z left empty - if no solution is found within that cap
    // (a genuine ray termination, or the cap was hit); callers need a
    // fallback for that case, since it can happen on pathological input.
    bool SolveLCPLemke(const std::vector<std::vector<float>>& M, const std::vector<float>& q,
                        std::vector<float>& z, int maxPivots = 0);
}
