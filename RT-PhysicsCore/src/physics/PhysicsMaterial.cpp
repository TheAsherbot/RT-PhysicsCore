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