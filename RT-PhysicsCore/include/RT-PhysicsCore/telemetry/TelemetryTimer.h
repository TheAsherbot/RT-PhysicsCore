/**
 * @file TelemetryTimer.h
 * @brief RAII-based scoped duration timer for capturing execution benchmarks.
 */

#pragma once

#include <chrono>

namespace RT_PhysicsCore
{
    /**
     * @class TelemetryTimer
     * @brief Measures the lifetime duration of its scope and commits the result to the TelemetryManager.
     */
    class TelemetryTimer
    {
    public:
        /**
         * @brief Constructs a timer instance and captures the starting timestamp.
         * @param name Name label assigned to the execution scope.
         */
        explicit TelemetryTimer(const char* name);

        /**
         * @brief Destructor. Automatically finishes timing and writes the profile result if active.
         */
        ~TelemetryTimer();

        TelemetryTimer(const TelemetryTimer&) = delete;
        TelemetryTimer& operator=(const TelemetryTimer&) = delete;

        /**
         * @brief Manually concludes duration measurement prior to leaving scope.
         */
        void Stop();

    private:
        const char* name;
        std::chrono::time_point<std::chrono::high_resolution_clock> startTimepoint;
        bool isStopped;
    };
}