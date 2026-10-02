/**
 * @file ProfileResult.h
 * @brief Data structures representing scoped profiling records, frame metrics, and session metadata.
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace RT_PhysicsCore
{
    /**
     * @struct ProfileResult
     * @brief Holds the timing and thread data for a single profiled execution scope.
     */
    struct ProfileResult
    {
        /// @brief Label or function name of the profiled scope.
        const char* name;

        /// @brief Microseconds since epoch when the scope began.
        long long startTimestamp;

        /// @brief Microseconds since epoch when the scope completed.
        long long endTimestamp;

        /// @brief Identifier of the execution thread that processed the scope.
        uint32_t threadId;
    };

    /**
     * @struct FrameSummary
     * @brief Aggregated duration metrics for high-level simulation subsystems across a single frame.
     */
    struct FrameSummary
    {
        /// @brief Total elapsed time for the entire frame in milliseconds.
        double totalFrameMs;

        /// @brief Elapsed time spent in fixed physics updates in milliseconds.
        double physicsMs;

        /// @brief Elapsed time spent in variable update logic in milliseconds.
        double updateMs;

        /// @brief Elapsed time spent rendering and presenting in milliseconds.
        double renderMs;
    };

    /**
     * @struct SessionInfo
     * @brief File metadata descriptors for stored session traces on disk.
     */
    struct SessionInfo
    {
        /// @brief Absolute or relative path to the trace file.
        std::string filepath;

        /// @brief Formatted display label derived from the timestamp.
        std::string displayName;

        /// @brief Total size of the session trace file in bytes.
        std::size_t fileSizeBytes;

        /// @brief Indicates whether the session is pinned to prevent deletion.
        bool isStarred;

        /// @brief Raw timestamp string used for sorting records.
        std::string timestamp;
    };
}