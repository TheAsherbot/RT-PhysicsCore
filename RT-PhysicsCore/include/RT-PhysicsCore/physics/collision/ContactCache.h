/**
 * @file ContactCache.h
 * @brief Persistent contact manifold cache supporting cross-substep warm starting.
 */

#pragma once

#include "RT-PhysicsCore/physics/collision/Contact.h"
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace RT_PhysicsCore
{
    /**
     * @class ContactCache
     * @brief Manages temporal tracking of contact manifolds across simulation substeps.
     */
    class ContactCache
    {
    public:
        /**
         * @brief Default constructor.
         */
        ContactCache();

        /**
         * @brief Correlates incoming contacts against cached manifolds to transfer warm-starting impulses.
         * @param contact Active manifold to warm-start and update.
         */
        void MatchAndWarmStart(Contact& contact);

        /**
         * @brief Writes back converged impulses into the persistent cache.
         * @param contact Converged manifold containing updated impulses.
         */
        void Store(const Contact& contact);

        /**
         * @brief Marks the start of a frame or substep to track obsolete cached contacts.
         */
        void BeginSubstep();

        /**
         * @brief Purges manifolds that were not refreshed during the active substep.
         */
        void EndSubstep();

        /**
         * @brief Clears all persistent manifolds from memory.
         */
        void Clear();

    private:
        struct CachedPoint
        {
            Vec3 localAnchorA{ 0.0, 0.0, 0.0 };
            Vec3 localAnchorB{ 0.0, 0.0, 0.0 };
            Real normalImpulse{ 0.0 };
            Vec3 tangentImpulseVector{ 0.0, 0.0, 0.0 };
            bool isStuck{ false };
        };

        struct CachedManifold
        {
            Vec3 normal{ 0.0, 1.0, 0.0 };
            std::vector<CachedPoint> points;
            uint32_t lastSubstepIndex{ 0 };
        };

        static uint64_t MakeKey(Entity a, Entity b);

        std::unordered_map<uint64_t, CachedManifold> cache;
        uint32_t currentSubstepIndex{ 0 };
        Real matchThresholdSq{ 0.0004 };
    };
}