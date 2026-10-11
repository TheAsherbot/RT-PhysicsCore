/**
 * @file GJK.h
 * @brief Gilbert-Johnson-Keerthi (GJK) convex distance and closest point algorithm.
 */

#pragma once

#include "RT-PhysicsCore/physics/collision/Support.h"

namespace RT_PhysicsCore
{
    /**
     * @struct GjkResult
     * @brief Detailed spatial proximity and closest-point outcome resulting from a GJK distance query.
     */
    struct GjkResult
    {
        bool areDisjoint{ true };               ///< True if shapes are strictly disjoint; false if core shapes penetrate.
        Real distance{ 0.0 };                   ///< Euclidean surface separation distance (negative if intersecting).
        Vec3 closestPointA{ 0.0, 0.0, 0.0 };    ///< Witness point on the surface of shape A.
        Vec3 closestPointB{ 0.0, 0.0, 0.0 };    ///< Witness point on the surface of shape B.
        Vec3 normal{ 0.0, 1.0, 0.0 };           ///< Unit normal pointing from surface of A toward surface of B.
        int iterationsExecuted{ 0 };            ///< Number of simplex refinement cycles performed before convergence.
    };

    /**
     * @class GJK
     * @brief Computes high-precision Euclidean separation distance and closest surface witness points.
     */
    class GJK
    {
    public:
        /**
         * @brief Executes the GJK algorithm to find minimum Euclidean distance between two convex shapes.
         *
         * Operates on the Minkowski Difference A - B = { a - b | a in A, b in B }.
         * Employs Johnson's sub-simplex distance algorithm across points, segments, triangles,
         * and tetrahedra to determine the point in the Minkowski difference closest to the origin.
         * Spherical expansion radii are accounted for analytically, ensuring high numerical stability.
         *
         * @param coreA Convex core specification for shape A.
         * @param coreB Convex core specification for shape B.
         * @param maxIterations Hard cap on simplex iterations (default: 32).
         * @return GjkResult containing separation distance, surface points, and contact normal.
         */
        static GjkResult ComputeDistance(const ConvexCore& coreA, const ConvexCore& coreB, int maxIterations = 32);

    private:
        struct SimplexVertex
        {
            Vec3 pA{ 0.0, 0.0, 0.0 };   ///< Support point on Shape A.
            Vec3 pB{ 0.0, 0.0, 0.0 };   ///< Support point on Shape B.
            Vec3 v{ 0.0, 0.0, 0.0 };    ///< Minkowski difference point: pA - pB.
        };

        struct Simplex
        {
            SimplexVertex vertices[4]{};
            int count{ 0 };
        };

        static bool SolveSimplex(Simplex& simplex, Vec3& searchDirection, Vec3& outClosestPoint,
            Vec3& outClosestA, Vec3& outClosestB);

        static bool SolvePoint(Simplex& simplex, Vec3& searchDirection, Vec3& outClosestPoint,
            Vec3& outClosestA, Vec3& outClosestB);

        static bool SolveSegment(Simplex& simplex, Vec3& searchDirection, Vec3& outClosestPoint,
            Vec3& outClosestA, Vec3& outClosestB);

        static bool SolveTriangle(Simplex& simplex, Vec3& searchDirection, Vec3& outClosestPoint,
            Vec3& outClosestA, Vec3& outClosestB);

        static bool SolveTetrahedron(Simplex& simplex, Vec3& searchDirection, Vec3& outClosestPoint,
            Vec3& outClosestA, Vec3& outClosestB);
    };
}