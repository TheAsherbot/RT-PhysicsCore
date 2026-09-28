#pragma once

#include "RT-PhysicsCore/physics/PhysicsMaterial.h"

namespace RT_PhysicsCore
{
    // Optional - CollisionSystem treats an entity with no
    // PhysicsMaterialComponent as MaterialId::Default rather than
    // requiring this on every collidable entity.
    struct PhysicsMaterialComponent
    {
        MaterialId material{MaterialId::Default};
    };
}
