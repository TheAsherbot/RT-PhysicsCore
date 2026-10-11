/**
 * @file LCPSolver.h
 * @brief Linear Complementarity Problem (LCP) solver using Lemke's pivoting algorithm.
 *
 * Provides exact simultaneous contact impulse solving for positive semi-definite
 * effective mass matrices with Bland's-rule anti-cycling tie-breaking.
 */

#pragma once

#include "RT-PhysicsCore/core/math/MathTypes.h"
#include <vector>

namespace RT_PhysicsCore
{
    /**
     * @enum LcpStatus
     * @brief Termination status resulting from Lemke LCP pivoting.
     */
    enum class LcpStatus
    {
        Solved,             ///< Complementary feasible solution successfully determined.
        RayTermination,     ///< Ray termination encountered along secondary ray; no solution found.
        PivotLimit,         ///< Maximum pivot count reached without finding a solution.
        NumericalFailure    ///< Degenerate pivot or singular tableau pivot value encountered.
    };

    /**
     * @struct LcpResult
     * @brief Detailed outcome reporting from Lemke algorithm execution.
     */
    struct LcpResult
    {
        LcpStatus status{ LcpStatus::NumericalFailure }; ///< Termination condition.
        int pivotsExecuted{ 0 };                         ///< Number of Gauss-Jordan pivots performed.
    };

    /**
     * @brief Solves the Linear Complementarity Problem w = M*z + q via Lemke's algorithm using a flat tableau.
     *
     * Finds vectors w and z satisfying:
     *   w = M*z + q,  w >= 0,  z >= 0,  z . w = 0
     *
     * Uses flat contiguous tableau pivoting with a unit covering vector and lexicographic ratio testing
     * to prevent cycling on degenerate ties.
     *
     * @param M Symmetric positive-semidefinite effective-mass matrix (flat contiguous array of size n * n).
     * @param q Constraint bias / velocity discrepancy vector (length n).
     * @param[out] z Output solution impulse vector (cleared to length n, populated on success).
     * @param n Dimension of the LCP system.
     * @param maxPivots Maximum allowed pivot steps (0 = default based on problem size: 4n + 50).
     * @param regEpsilon Regularization factor added to matrix diagonals for numerical stability.
     * @return Execution result summary.
     */
    LcpResult SolveLCPLemke(const std::vector<Real>& M, const std::vector<Real>& q,
        std::vector<Real>& z, int n, int maxPivots = 0, Real regEpsilon = 1e-9);

    /**
     * @brief Convenience overload of SolveLCPLemke accepting a 2D matrix (vector of rows).
     * @param M Symmetric positive-semidefinite effective-mass matrix (n x n rows).
     * @param q Constraint bias / velocity discrepancy vector (length n).
     * @param[out] z Output solution impulse vector (cleared to length n, populated on success).
     * @param maxPivots Maximum allowed pivot steps (0 = default based on problem size: 4n + 50).
     * @param regEpsilon Regularization factor added to matrix diagonals for numerical stability.
     * @return Execution result summary.
     */
    LcpResult SolveLCPLemke(const std::vector<std::vector<Real>>& M, const std::vector<Real>& q,
        std::vector<Real>& z, int maxPivots = 0, Real regEpsilon = 1e-9);
}