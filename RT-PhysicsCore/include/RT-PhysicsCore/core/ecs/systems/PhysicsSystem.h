#pragma once

#include "RT-PhysicsCore/core/ecs/core/System.h"

namespace RT_PhysicsCore
{
    class PhysicsSystem : public ISystem
    {
    public:
        explicit PhysicsSystem(Scene& scene);

        void FixedUpdate(double dt) override;
    };
}
