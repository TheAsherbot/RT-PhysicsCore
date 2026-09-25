#include "RT-PhysicsCore/physics/LCPSolver.h"

#include <cmath>

namespace RT_PhysicsCore
{
    namespace
    {
        // Lemke's algorithm chains many pivots (Gauss-Jordan elimination
        // steps), and it's documented as numerically sensitive - float's
        // ~7 digits of precision isn't much margin for that much chained
        // rounding. Tableau arithmetic is done in double; the public API
        // stays float to match the rest of the codebase.
        constexpr double kLcpEpsilon = 1e-9;

        void Pivot(std::vector<std::vector<double>>& tableau, int row, int col)
        {
            int rows = static_cast<int>(tableau.size());
            int cols = static_cast<int>(tableau[0].size());

            double pivotVal = tableau[row][col];
            for (int j = 0; j < cols; ++j)
                tableau[row][j] /= pivotVal;

            for (int i = 0; i < rows; ++i)
            {
                if (i == row)
                    continue;
                double factor = tableau[i][col];
                if (std::abs(factor) < kLcpEpsilon)
                    continue;
                for (int j = 0; j < cols; ++j)
                    tableau[i][j] -= factor * tableau[row][j];
            }
        }
    }

    bool SolveLCPLemke(const std::vector<std::vector<float>>& M, const std::vector<float>& q,
                        std::vector<float>& z, int maxPivots)
    {
        int n = static_cast<int>(q.size());
        z.assign(n, 0.0f);
        if (n == 0)
            return true;

        if (maxPivots <= 0)
            maxPivots = 4 * n + 50;

        // Columns: [0..n) = w_i, [n..2n) = z_i, 2n = the artificial Lemke
        // variable (z0), 2n+1 = rhs. Tableau row i is the equation
        // w_i - sum_j M[i][j]*z_j - z0 = q_i (unit covering vector).
        const int wCol0 = 0;
        const int zCol0 = n;
        const int artCol = 2 * n;
        const int rhsCol = 2 * n + 1;

        std::vector<std::vector<double>> tableau(n, std::vector<double>(2 * n + 2, 0.0));
        std::vector<int> basis(n);

        for (int i = 0; i < n; ++i)
        {
            tableau[i][wCol0 + i] = 1.0;
            for (int j = 0; j < n; ++j)
                tableau[i][zCol0 + j] = -static_cast<double>(M[i][j]);
            tableau[i][artCol] = -1.0;
            tableau[i][rhsCol] = static_cast<double>(q[i]);
            basis[i] = wCol0 + i;
        }

        // Trivial case: q already feasible, z = 0 solves it.
        int r = 0;
        for (int i = 1; i < n; ++i)
            if (tableau[i][rhsCol] < tableau[r][rhsCol])
                r = i;
        if (tableau[r][rhsCol] >= -kLcpEpsilon)
            return true;

        // Initial pivot brings the artificial variable into row r.
        Pivot(tableau, r, artCol);
        int leavingVar = basis[r];
        basis[r] = artCol;
        int drivingCol = (leavingVar < zCol0) ? (zCol0 + leavingVar) : (leavingVar - zCol0);

        for (int iter = 0; iter < maxPivots; ++iter)
        {
            int enterRow = -1;
            double bestRatio = 0.0;

            for (int i = 0; i < n; ++i)
            {
                double coeff = tableau[i][drivingCol];
                if (coeff <= kLcpEpsilon)
                    continue;

                double ratio = tableau[i][rhsCol] / coeff;
                bool better = (enterRow == -1) || (ratio < bestRatio - kLcpEpsilon);
                bool tied = (enterRow != -1) && (std::abs(ratio - bestRatio) <= kLcpEpsilon);

                // Bland's rule: among tied rows, the one whose current
                // basic variable has the smaller index leaves - a simple,
                // provably cycle-free tie-break. bestRatio only moves on a
                // genuine improvement, never on a tie-break win, so it
                // can't drift away from the true minimum across repeated
                // ties in the same pass.
                if (better)
                {
                    bestRatio = ratio;
                    enterRow = i;
                }
                else if (tied && basis[i] < basis[enterRow])
                {
                    enterRow = i;
                }
            }

            if (enterRow == -1)
                return false; // ray termination - no solution found via this path

            int leavingBasisVar = basis[enterRow];
            Pivot(tableau, enterRow, drivingCol);
            basis[enterRow] = drivingCol;

            if (leavingBasisVar == artCol)
                break; // z0 left the basis - solution found

            drivingCol = (leavingBasisVar < zCol0) ? (zCol0 + leavingBasisVar) : (leavingBasisVar - zCol0);

            if (iter == maxPivots - 1)
                return false; // pivot cap reached without z0 leaving
        }

        for (int i = 0; i < n; ++i)
            if (basis[i] >= zCol0 && basis[i] < artCol)
                z[basis[i] - zCol0] = static_cast<float>(tableau[i][rhsCol]);

        return true;
    }
}
