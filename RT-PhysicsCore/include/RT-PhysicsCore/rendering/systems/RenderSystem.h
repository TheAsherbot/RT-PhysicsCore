/**
 * @file RenderSystem.h
 * @brief ECS system dispatching draw calls for entities with mesh and transform components.
 */

#pragma once

#include "RT-PhysicsCore/core/ecs/core/System.h"

namespace RT_PhysicsCore
{
    class Renderer;

    /**
     * @class RenderSystem
     * @brief Traverses visible entities and coordinates frame drawing with the Renderer.
     */
    class RenderSystem : public ISystem
    {
    public:
        /**
         * @brief Constructs the RenderSystem bound to an active Scene and Renderer.
         * @param scene Reference to the parent ECS Scene.
         * @param renderer Reference to the hardware Renderer.
         */
        RenderSystem(Scene& scene, Renderer& renderer);

        /**
         * @brief Queries renderable entities, submits meshes, and flushes debug line draws.
         */
        void RenderUpdate() override;

    private:
        Renderer& renderer;
    };
}