/**
 * @file ManifoldReduction.h
 * @brief Extremal polygon reduction for contact manifolds.
 */

#pragma once

#include "RT-PhysicsCore/core/math/MathTypes.h"
#include "RT-PhysicsCore/physics/collision/Contact.h"

namespace RT_PhysicsCore
{
    /**
     * @class ManifoldReduction
     * @brief Reduces arbitrary collections of contact points to at most four area-maximizing points.
     */
    class ManifoldReduction
    {
    public:
        /**
         * @brief Filters an input array of contact points down to at most four extremal points.
         * @param inPoints Pointer to source contact point array.
         * @param inCount Number of source points.
         * @param normal Manifold contact normal.
         * @param outPoints Destination array (must hold at least 4 elements).
         * @return Number of output points retained in outPoints.
         */
        static int Reduce(const ContactPoint* inPoints, int inCount, const Vec3& normal, ContactPoint* outPoints);
    };
}