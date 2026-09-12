#pragma once

#include "RT-PhysicsCore/core/ecs/core/System.h"

namespace RT_PhysicsCore
{
    class RenderSystem : public ISystem
    {
    public:
        explicit RenderSystem(Scene& scene);

        void RenderUpdate() override;
    };
}
