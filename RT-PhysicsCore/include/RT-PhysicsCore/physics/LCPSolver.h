/**
 * @file LCPSolver.h
 * @brief Linear Complementarity Problem (LCP) solver using Lemke's pivoting algorithm.
 *
 * Provides exact simultaneous contact impulse solving for positive semi-definite
 * effective mass matrices with Bland's-rule anti-cycling tie-breaking.
 */

#pragma once

#include <vector>

namespace RT_PhysicsCore
{
    /**
     * @brief Solves the Linear Complementarity Problem w = M*z + q via Lemke's algorithm.
     *
     * Finds vectors w and z satisfying:
     *   w = M*z + q,  w >= 0,  z >= 0,  z . w = 0
     *
     * Uses tableau pivoting with a unit covering vector and Bland's-rule tie-breaking
     * in the minimum-ratio test to prevent cycling on degenerate ties without the overhead
     * of full lexicographic ordering.
     *
     * Tailored for symmetric positive-semidefinite (PSD) effective-mass matrices produced
     * by rigid body contact constraints, which guarantees termination.
     *
     * @param M Symmetric positive-semidefinite effective-mass matrix (n x n).
     * @param q Constraint bias / velocity discrepancy vector (length n).
     * @param[out] z Output solution impulse vector (cleared to length n, populated on success).
     * @param maxPivots Maximum allowed pivot steps (0 = default based on problem size: 4n + 50).
     * @return True if a complementary solution was reached; false on ray termination or pivot exhaustion.
     */
    bool SolveLCPLemke(const std::vector<std::vector<float>>& M, const std::vector<float>& q,
        std::vector<float>& z, int maxPivots = 0);
}