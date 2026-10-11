/**
 * @file LCPSolver.cpp
 * @brief Implementation of stabilized flat-tableau Lemke algorithm with lexicographic ratio testing.
 */

#include "RT-PhysicsCore/physics/LCPSolver.h"
#include <algorithm>
#include <cmath>

namespace RT_PhysicsCore
{
    namespace
    {
        constexpr Real kPivotEpsilon = 1e-12;
        constexpr Real kRatioEpsilon = 1e-12;

        void PerformPivot(std::vector<Real>& tableau, int rows, int cols, int row, int col)
        {
            Real pivotVal = tableau[row * cols + col];
            Real invPivot = 1.0 / pivotVal;

            for (int j = 0; j < cols; ++j)
            {
                tableau[row * cols + j] *= invPivot;
            }

            for (int i = 0; i < rows; ++i)
            {
                if (i == row)
                {
                    continue;
                }

                Real factor = tableau[i * cols + col];
                if (std::abs(factor) < kPivotEpsilon)
                {
                    continue;
                }

                for (int j = 0; j < cols; ++j)
                {
                    tableau[i * cols + j] -= factor * tableau[row * cols + j];
                }
            }
        }
    }

    LcpResult SolveLCPLemke(const std::vector<Real>& M, const std::vector<Real>& q,
        std::vector<Real>& z, int n, int maxPivots, Real regEpsilon)
    {
        LcpResult result;
        z.assign(n, 0.0);

        if (n == 0)
        {
            result.status = LcpStatus::Solved;
            return result;
        }

        if (maxPivots <= 0)
        {
            maxPivots = 4 * n + 50;
        }

        // 1. Symmetrically scale M and q so that M_ii = 1.0
        std::vector<Real> scale(n, 1.0);
        std::vector<Real> scaledM(n * n, 0.0);
        std::vector<Real> scaledQ(n, 0.0);

        for (int i = 0; i < n; ++i)
        {
            Real diag = M[i * n + i] + regEpsilon;
            scale[i] = (diag > kPivotEpsilon) ? (1.0 / std::sqrt(diag)) : 1.0;
        }

        for (int i = 0; i < n; ++i)
        {
            scaledQ[i] = q[i] * scale[i];
            for (int j = 0; j < n; ++j)
            {
                Real val = M[i * n + j];
                if (i == j)
                {
                    val += regEpsilon;
                }
                scaledM[i * n + j] = val * scale[i] * scale[j];
            }
        }

        // Check if q >= 0 already (trivial zero solution)
        int minQRow = 0;
        Real minQVal = scaledQ[0];
        for (int i = 1; i < n; ++i)
        {
            if (scaledQ[i] < minQVal)
            {
                minQVal = scaledQ[i];
                minQRow = i;
            }
        }

        if (minQVal >= -kPivotEpsilon)
        {
            result.status = LcpStatus::Solved;
            return result;
        }

        // Tableau layout:
        // [0 .. n-1]: w (slack) columns
        // [n .. 2n-1]: z (impulse) columns
        // [2n]: z0 (artificial covering variable)
        // [2n+1]: RHS
        const int cols = 2 * n + 2;
        const int wCol0 = 0;
        const int zCol0 = n;
        const int artCol = 2 * n;
        const int rhsCol = 2 * n + 1;

        std::vector<Real> tableau(n * cols, 0.0);
        std::vector<int> basis(n);

        for (int i = 0; i < n; ++i)
        {
            tableau[i * cols + (wCol0 + i)] = 1.0;
            for (int j = 0; j < n; ++j)
            {
                tableau[i * cols + (zCol0 + j)] = -scaledM[i * n + j];
            }
            tableau[i * cols + artCol] = -1.0;
            tableau[i * cols + rhsCol] = scaledQ[i];
            basis[i] = wCol0 + i;
        }

        // Initial pivot to bring artificial variable z0 into basis
        PerformPivot(tableau, n, cols, minQRow, artCol);
        int leavingVar = basis[minQRow];
        basis[minQRow] = artCol;
        int drivingCol = (leavingVar < zCol0) ? (zCol0 + leavingVar) : (leavingVar - zCol0);

        for (int iter = 0; iter < maxPivots; ++iter)
        {
            result.pivotsExecuted++;

            // Ratio test to find leaving row
            int enterRow = -1;
            Real minRatio = 1e30;

            for (int i = 0; i < n; ++i)
            {
                Real dir = tableau[i * cols + drivingCol];
                if (dir <= kPivotEpsilon)
                {
                    continue;
                }

                Real ratio = tableau[i * cols + rhsCol] / dir;
                if (ratio < 0.0 && ratio > -kRatioEpsilon)
                {
                    ratio = 0.0;
                }

                if (ratio >= 0.0)
                {
                    if (ratio + kRatioEpsilon < minRatio)
                    {
                        minRatio = ratio;
                        enterRow = i;
                    }
                    else if (std::abs(ratio - minRatio) <= kRatioEpsilon)
                    {
                        // Lexicographic ratio test tie-breaking
                        if (enterRow != -1 && basis[i] == artCol)
                        {
                            enterRow = i;
                            minRatio = ratio;
                        }
                        else if (enterRow != -1 && basis[enterRow] != artCol)
                        {
                            for (int c = 0; c < 2 * n; ++c)
                            {
                                Real r1 = tableau[i * cols + c] / dir;
                                Real r2 = tableau[enterRow * cols + c] / tableau[enterRow * cols + drivingCol];
                                if (std::abs(r1 - r2) > kRatioEpsilon)
                                {
                                    if (r1 < r2)
                                    {
                                        enterRow = i;
                                        minRatio = ratio;
                                    }
                                    break;
                                }
                            }
                        }
                    }
                }
            }

            if (enterRow == -1)
            {
                result.status = LcpStatus::RayTermination;
                return result;
            }

            int leavingBasisVar = basis[enterRow];
            PerformPivot(tableau, n, cols, enterRow, drivingCol);
            basis[enterRow] = drivingCol;

            // If artificial variable left the basis, optimal solution has been determined
            if (leavingBasisVar == artCol)
            {
                result.status = LcpStatus::Solved;
                break;
            }

            drivingCol = (leavingBasisVar < zCol0) ? (zCol0 + leavingBasisVar) : (leavingBasisVar - zCol0);

            if (iter == maxPivots - 1)
            {
                result.status = LcpStatus::PivotLimit;
                return result;
            }
        }

        // Extract solutions for z and rescale back to unscaled coordinates
        for (int i = 0; i < n; ++i)
        {
            if (basis[i] >= zCol0 && basis[i] < artCol)
            {
                int varIdx = basis[i] - zCol0;
                Real val = tableau[i * cols + rhsCol];
                z[varIdx] = std::max(0.0, val * scale[varIdx]);
            }
        }

        return result;
    }

    LcpResult SolveLCPLemke(const std::vector<std::vector<Real>>& M, const std::vector<Real>& q,
        std::vector<Real>& z, int maxPivots, Real regEpsilon)
    {
        int n = static_cast<int>(q.size());
        if (n == 0)
        {
            z.clear();
            return { LcpStatus::Solved, 0 };
        }

        std::vector<Real> flatM(n * n, 0.0);
        for (int i = 0; i < n && i < static_cast<int>(M.size()); ++i)
        {
            for (int j = 0; j < n && j < static_cast<int>(M[i].size()); ++j)
            {
                flatM[i * n + j] = M[i][j];
            }
        }

        return SolveLCPLemke(flatM, q, z, n, maxPivots, regEpsilon);
    }
}