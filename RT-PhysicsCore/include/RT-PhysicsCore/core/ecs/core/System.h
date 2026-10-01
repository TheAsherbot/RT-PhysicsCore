/**
 * @file System.h
 * @brief Base interface for systems in the Entity Component System (ECS).
 *
 * Defines the lifecycle hooks (Update, FixedUpdate, RenderUpdate) called by Scene.
 */

#pragma once

namespace RT_PhysicsCore
{
    class Scene;

    /**
     * @class ISystem
     * @brief Abstract base class for all ECS logic and simulation systems.
     */
    class ISystem
    {
    public:
        /**
         * @brief Constructs a system bound to a parent Scene.
         * @param scene Reference to the Scene containing entities processed by this system.
         */
        explicit ISystem(Scene& scene);

        /**
         * @brief Virtual destructor for clean polymorphic teardown.
         */
        virtual ~ISystem();

        /**
         * @brief Per-frame logic update hook. Called once per rendered frame.
         */
        virtual void Update();

        /**
         * @brief Fixed-rate simulation step hook. Called once per fixed physics tick.
         * @param dt Fixed delta time in seconds.
         */
        virtual void FixedUpdate(double dt);

        /**
         * @brief Presentation preparation hook. Called immediately before rendering.
         */
        virtual void RenderUpdate();

    protected:
        Scene& scene; ///< Reference to the governing ECS scene.
    };
}