/**
 * @file ContactCache.cpp
 * @brief Implementation of persistent contact manifold correlation and warm-starting.
 */

#include "RT-PhysicsCore/physics/collision/ContactCache.h"
#include <algorithm>

namespace RT_PhysicsCore
{
    ContactCache::ContactCache()
    {}

    uint64_t ContactCache::MakeKey(Entity a, Entity b)
    {
        uint32_t idA = static_cast<uint32_t>(a);
        uint32_t idB = static_cast<uint32_t>(b);
        if (idA > idB)
        {
            std::swap(idA, idB);
        }
        return (static_cast<uint64_t>(idA) << 32) | static_cast<uint64_t>(idB);
    }

    void ContactCache::BeginSubstep()
    {
        ++currentSubstepIndex;
    }

    void ContactCache::EndSubstep()
    {
        for (auto it = cache.begin(); it != cache.end(); )
        {
            if (it->second.lastSubstepIndex != currentSubstepIndex)
            {
                it = cache.erase(it);
            }
            else
            {
                ++it;
            }
        }
    }

    void ContactCache::MatchAndWarmStart(Contact& contact)
    {
        uint64_t key = MakeKey(contact.a, contact.b);
        auto it = cache.find(key);
        if (it == cache.end())
        {
            return;
        }

        const CachedManifold& cached = it->second;

        if (glm::dot(cached.normal, contact.normal) < 0.95)
        {
            return;
        }

        bool inverted = (contact.a > contact.b);

        for (int i = 0; i < contact.pointCount; ++i)
        {
            ContactPoint& pt = contact.points[i];
            const Vec3& anchorA = inverted ? pt.localAnchorB : pt.localAnchorA;
            const Vec3& anchorB = inverted ? pt.localAnchorA : pt.localAnchorB;

            Real bestDistSq = matchThresholdSq;
            int bestMatch = -1;

            for (size_t j = 0; j < cached.points.size(); ++j)
            {
                Vec3 dA = cached.points[j].localAnchorA - anchorA;
                Vec3 dB = cached.points[j].localAnchorB - anchorB;
                Real d = glm::dot(dA, dA) + glm::dot(dB, dB);

                if (d < bestDistSq)
                {
                    bestDistSq = d;
                    bestMatch = static_cast<int>(j);
                }
            }

            if (bestMatch != -1)
            {
                const CachedPoint& cp = cached.points[bestMatch];
                pt.normalImpulse = cp.normalImpulse;
                pt.isStuck = cp.isStuck;

                pt.tangentImpulse1 = glm::dot(cp.tangentImpulseVector, contact.tangent1);
                pt.tangentImpulse2 = glm::dot(cp.tangentImpulseVector, contact.tangent2);
            }
        }
    }

    void ContactCache::Store(const Contact& contact)
    {
        uint64_t key = MakeKey(contact.a, contact.b);
        CachedManifold& cached = cache[key];
        cached.normal = contact.normal;
        cached.lastSubstepIndex = currentSubstepIndex;
        cached.points.clear();

        bool inverted = (contact.a > contact.b);

        for (int i = 0; i < contact.pointCount; ++i)
        {
            const ContactPoint& pt = contact.points[i];
            CachedPoint cp;
            cp.localAnchorA = inverted ? pt.localAnchorB : pt.localAnchorA;
            cp.localAnchorB = inverted ? pt.localAnchorA : pt.localAnchorB;
            cp.normalImpulse = pt.normalImpulse;
            cp.tangentImpulseVector = pt.tangentImpulse1 * contact.tangent1 + pt.tangentImpulse2 * contact.tangent2;
            cp.isStuck = pt.isStuck;
            cached.points.push_back(cp);
        }
    }

    void ContactCache::Clear()
    {
        cache.clear();
        currentSubstepIndex = 0;
    }
}