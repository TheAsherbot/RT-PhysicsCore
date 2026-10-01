/**
 * @file FixedTimestep.h
 * @brief Accumulator-based fixed-timestep simulation timer.
 *
 * Implements deterministic physics stepping decoupled from variable frame rates
 * with spiral-of-death delta clamping and presentation alpha calculation.
 */

#pragma once

#include <cstdint>

namespace RT_PhysicsCore
{
    /**
     * @class FixedTimestep
     * @brief Manages accumulator time tracking for fixed-rate physics loops.
     */
    class FixedTimestep
    {
    public:
        /**
         * @brief Constructs the timer configured for a specific target update frequency.
         * @param hz Target fixed simulation frequency in Hertz (default: 60.0 Hz).
         */
        explicit FixedTimestep(double hz = 60.0);

        /**
         * @brief Updates the target update frequency.
         * @param hz Target frequency in Hertz.
         */
        void SetFrequency(double hz);

        /**
         * @brief Gets fixed timestep duration per step in seconds.
         * @return Duration in seconds (1.0 / frequency).
         */
        double FixedDeltaSeconds() const;

        /**
         * @brief Advances timer using wall-clock time and returns fixed steps to execute.
         * @return Number of fixed physics updates required this frame.
         */
        std::uint32_t Step();

        /**
         * @brief Gets interpolation blend factor between the previous and current fixed state.
         * @return Alpha value in range [0.0, 1.0].
         */
        double Alpha() const;

        /**
         * @brief Gets real elapsed delta time for the current frame in seconds.
         * @return Frame delta time in seconds.
         */
        double DeltaSeconds() const;

    private:
        double fixedDelta;  ///< Seconds per fixed simulation step.
        double accumulator; ///< Unprocessed time carried over to next frame.
        double lastTime;    ///< Wall-clock timestamp of previous step call.
        double frameDelta;  ///< Elapsed time since previous frame.
    };
}