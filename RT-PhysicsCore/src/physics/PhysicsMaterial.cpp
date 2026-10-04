/**
 * @file PhysicsMaterial.cpp
 * @brief Implementation of material property lookup tables and combination heuristics.
 */

#include "RT-PhysicsCore/physics/PhysicsMaterial.h"

#include <cmath>
#include <map>
#include <utility>

namespace RT_PhysicsCore
{
    namespace
    {
        MaterialProperties SoloProperties(MaterialId id)
        {
            switch (id)
            {
                case MaterialId::Default:
                {
                    return { 0.3f, 0.6f, 0.4f };
                }
                case MaterialId::Clay:
                {
                    return { 0.0f, 0.8f, 0.6f };
                }
                case MaterialId::Wood:
                {
                    return { 0.25f, 0.5f, 0.4f };
                }
                case MaterialId::Rubber:
                {
                    return { 0.5f, 0.8f, 0.6f };
                }
                case MaterialId::HardRubber:
                {
                    return { 0.75f, 0.8f, 0.6f };
                }
                case MaterialId::SuperBall:
                {
                    return { 1.0f, 0.6f, 0.4f };
                }
                case MaterialId::BouncyIce:
                {
                    return { 1.0f, 0.0f, 0.0f };
                }
                case MaterialId::Domino:
                {
                    return { 0.1f, 0.8f, 0.6f };
                }
            }

            return { 0.3f, 0.6f, 0.4f };
        }


        // Explicit overrides for specific pairs - add entries here rather
        // than trusting the geometric/arithmetic-mean fallback wherever
        // real (measured or hand-tuned) values matter.
        const std::map<std::pair<MaterialId, MaterialId>, MaterialPairProperties>& PairTable()
        {
            static const std::map<std::pair<MaterialId, MaterialId>, MaterialPairProperties> table = {
                // { { MaterialId::Rubber, MaterialId::Concrete }, { 0.8f, 0.9f, 0.7f } },
            };
            return table;
        }
    }

    MaterialProperties GetMaterialProperties(MaterialId id)
    {
        return SoloProperties(id);
    }

    MaterialPairProperties GetPairProperties(MaterialId a, MaterialId b)
    {
        std::pair<MaterialId, MaterialId> key = (a <= b) ? std::make_pair(a, b) : std::make_pair(b, a);

        const auto& table = PairTable();
        auto it = table.find(key);
        if (it != table.end())
        {
            return it->second;
        }

        MaterialProperties pa = SoloProperties(a);
        MaterialProperties pb = SoloProperties(b);

        MaterialPairProperties result;
        result.restitution = 0.5f * (pa.restitution + pb.restitution);
        result.staticFriction = std::sqrt(pa.staticFriction * pb.staticFriction);
        result.kineticFriction = std::sqrt(pa.kineticFriction * pb.kineticFriction);
        return result;
    }
}