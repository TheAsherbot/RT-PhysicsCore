/**
 * @file TelemetryManager.h
 * @brief Central singleton managing performance logging, trace streaming, and runtime frame statistics.
 * @author Asher Jordan
 * @date 2026-10-01
 */

#pragma once

#include "RT-PhysicsCore/telemetry/ProfileResult.h"

#include <fstream>
#include <mutex>
#include <string>
#include <vector>

namespace RT_PhysicsCore
{
    /**
     * @enum TelemetryMode
     * @brief Visualization display state modes for profiling metrics.
     */
    enum class TelemetryMode
    {
        Off = 0,
        Minimal = 1,
        Full = 2
    };

    /**
     * @class TelemetryManager
     * @brief Singleton orchestrator that serializes session events to disk and buffers live frame performance.
     */
    class TelemetryManager
    {
    public:
        /**
         * @brief Retrieves the global TelemetryManager singleton instance.
         * @return Reference to the singleton instance.
         */
        static TelemetryManager& Get();

        /**
         * @brief Opens the session output trace file and serializes the format header.
         */
        void Initialize();

        /**
         * @brief Serializes the format footer and safely flushes remaining log streams to disk.
         */
        void Shutdown();

        /**
         * @brief Thread-safe ingestion and serialization of an individual scope profiling entry.
         * @param result Collected execution scope metadata.
         */
        void WriteProfile(const ProfileResult& result);

        /**
         * @brief Sets the active visual telemetry display mode.
         * @param mode Target display mode.
         */
        void SetMode(TelemetryMode mode);

        /**
         * @brief Retrieves the current visual telemetry display mode.
         * @return Active TelemetryMode.
         */
        TelemetryMode GetMode() const;

        /**
         * @brief Adds a high-level frame execution breakdown to the rolling history buffer.
         * @param summary High-level subsystem timing measurements for the completed frame.
         */
        void PushFrameSummary(const FrameSummary& summary);

        /**
         * @brief Retrieves historical frame summary data used for averaging.
         * @return Const reference to the frame summary buffer.
         */
        const std::vector<FrameSummary>& GetFrameHistory() const;

        /**
         * @brief Retrieves live profiling events captured during the active session.
         * @return Const reference to active session profile results.
         */
        const std::vector<ProfileResult>& GetLiveResults() const;

        /**
         * @brief Clears accumulated live profiling records.
         */
        void ClearLiveResults();

    private:
        TelemetryManager();
        ~TelemetryManager();

        TelemetryManager(const TelemetryManager&) = delete;
        TelemetryManager& operator=(const TelemetryManager&) = delete;

        void WriteHeader();
        void WriteFooter();

        std::ofstream outputStream;
        int profileCount;
        std::mutex mutex;
        bool isInitialized;
        TelemetryMode currentMode;

        std::vector<FrameSummary> frameHistory;
        std::vector<ProfileResult> liveResults;
    };
}