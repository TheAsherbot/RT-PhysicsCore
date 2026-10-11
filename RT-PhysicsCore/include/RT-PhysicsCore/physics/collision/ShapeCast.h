/**
 * @file ShapeCast.h
 * @brief Bilateral conservative advancement time-of-impact (TOI) algorithm.
 */

#pragma once

#include "RT-PhysicsCore/physics/collision/GJK.h"

namespace RT_PhysicsCore
{
    /**
     * @struct ShapeCastResult
     * @brief Detailed outcome reporting from a bilateral continuous shape-cast query.
     */
    struct ShapeCastResult
    {
        bool hasHit{ false };               ///< True if a physical impact was determined within the substep interval.
        Real timeOfImpact{ 1.0 };           ///< Normalized time fraction in [0, 1] when collision initiates.
        Vec3 normal{ 0.0, 1.0, 0.0 };       ///< Unit contact normal at impact directed from shape A to shape B.
        Vec3 contactPoint{ 0.0, 0.0, 0.0 }; ///< World-space witness contact position at impact.
        int iterations{ 0 };                ///< Number of conservative advancement steps executed.
    };

    /**
     * @class ShapeCast
     * @brief Evaluates exact continuous collision time-of-impact using conservative advancement.
     */
    class ShapeCast
    {
    public:
        /**
         * @brief Evaluates earliest time-of-impact between two moving, rotating convex bodies.
         *
         * Employs conservative advancement:
         * In each iteration, GJK calculates current Euclidean separation distance d.
         * An upper bound on relative closing speed is computed using relative linear velocity and
         * angular velocities: bound = max(0, v_rel dot -n) + ||omega_A|| * rA + ||omega_B|| * rB.
         * The simulation time is conservatively advanced by delta_t = (d - targetGap) / bound,
         * mathematically guaranteeing that no physical intersection is skipped.
         *
         * @param coreA Shape A geometric core at the start of the substep.
         * @param linDispA Linear displacement vector of body A over the substep (vA * h).
         * @param angVelA Angular velocity of body A (rad/s).
         * @param boundRadiusA Maximum bounding radius of body A from center of mass.
         * @param coreB Shape B geometric core at the start of the substep.
         * @param linDispB Linear displacement vector of body B over the substep (vB * h).
         * @param angVelB Angular velocity of body B (rad/s).
         * @param boundRadiusB Maximum bounding radius of body B from center of mass.
         * @param h Substep duration in seconds.
         * @param targetGap Target security skin distance to halt advancement (meters).
         * @param maxIterations Maximum advancement steps before stopping (default: 24).
         * @return ShapeCastResult containing hit flag, time fraction, normal, and contact point.
         */
        static ShapeCastResult Cast(
            const ConvexCore& coreA, const Vec3& linDispA, const Vec3& angVelA, Real boundRadiusA,
            const ConvexCore& coreB, const Vec3& linDispB, const Vec3& angVelB, Real boundRadiusB,
            double h, Real targetGap = 0.001, int maxIterations = 24);
    };
}