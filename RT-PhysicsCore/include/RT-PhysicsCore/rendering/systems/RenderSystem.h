#pragma once

#include "RT-PhysicsCore/core/ecs/core/System.h"

namespace RT_PhysicsCore
{
    class Renderer;

    class RenderSystem : public ISystem
    {
    public:
        RenderSystem(Scene& scene, Renderer& renderer);

        void RenderUpdate() override;

    private:
        Renderer& renderer;
    };
}
