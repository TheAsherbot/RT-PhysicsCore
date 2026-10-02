/**
 * @file Engine.h
 * @brief Main engine coordinator managing the game loop, timing, and lifecycle callbacks.
 *
 * Coordinates fixed-timestep simulation updates, variable-rate frame logic,
 * and presentation interpolation.
 */

#pragma once

#include <functional>

#include "RT-PhysicsCore/telemetry/TelemetryMacros.h"

namespace RT_PhysicsCore
{
    class FixedTimestep;

    /**
     * @class Engine
     * @brief Orchestrates the simulation lifecycle and main execution loop.
     *
     * Decouples physics execution (fixed rate) from rendering and general updates
     * (variable frame rate) using an accumulator-based fixed timestep timer.
     */
    class Engine
    {
    public:
        /// @brief Callback invoked for fixed-rate physics steps.
        /// @param fixedDeltaTime Duration of one fixed physics step in seconds.
        using FixedUpdateCallback = std::function<void(double fixedDeltaTime)>;

        /// @brief Callback invoked once per frame for general logic and system updates.
        /// @param deltaTime Time elapsed since the previous frame in seconds.
        using UpdateCallback = std::function<void(double deltaTime)>;

        /// @brief Callback invoked once per frame for rendering.
        /// @param alpha Blend factor [0.0, 1.0] representing the sub-step interpolation progress.
        using RenderCallback = std::function<void(double alpha)>;

        /**
         * @brief Constructs an Engine instance configured with a target physics frequency.
         * @param physicsHz Frequency of the fixed physics update in Hertz (default: 60 Hz).
         */
        Engine(double physicsHz = 60.0);

        /**
         * @brief Destructor. Cleans up internal timestep resources.
         */
        ~Engine();

        /**
         * @brief Registers the callback for fixed-rate physics simulation steps.
         * @param cb Callback function receiving the fixed delta time in seconds.
         */
        void SetFixedUpdateCallback(FixedUpdateCallback cb);

        /**
         * @brief Registers the callback for per-frame variable updates.
         * @param cb Callback function receiving the frame delta time in seconds.
         */
        void SetUpdateCallback(UpdateCallback cb);

        /**
         * @brief Registers the callback for per-frame rendering and presentation.
         * @param cb Callback function receiving the interpolation factor alpha.
         */
        void SetRenderCallback(RenderCallback cb);

        /**
         * @brief Starts the blocking engine loop. Runs until RequestExit() is called.
         */
        void Run();

        /**
         * @brief Requests termination of the main engine loop at the end of the current frame.
         */
        void RequestExit();

    private:
        bool isRunning = true;
        FixedTimestep* fixedTimestep;

        FixedUpdateCallback fixedUpdateCallback;
        UpdateCallback updateCallback;
        RenderCallback renderCallback;
    };
}