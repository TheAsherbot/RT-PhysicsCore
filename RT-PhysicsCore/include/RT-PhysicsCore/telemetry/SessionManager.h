/**
 * @file SessionManager.h
 * @brief ImGui interface for browsing, starring, and deleting saved telemetry sessions.
 */

#pragma once

#include "RT-PhysicsCore/telemetry/ProfileResult.h"
#include "RT-PhysicsCore/telemetry/ProfilerUI.h"

#include <string>
#include <vector>

namespace RT_PhysicsCore
{
    /**
     * @class SessionManager
     * @brief Provides a full-screen ImGui application for managing and viewing stored profiler sessions.
     */
    class SessionManager
    {
    public:
        /**
         * @brief Constructs the session manager targeting a log directory.
         * @param logDirectory Path to the directory containing session trace files.
         */
        explicit SessionManager(const std::string& logDirectory);

        /**
         * @brief Destructor.
         */
        ~SessionManager();

        SessionManager(const SessionManager&) = delete;
        SessionManager& operator=(const SessionManager&) = delete;

        /**
         * @brief Renders the full session browser and viewer UI for the current frame.
         */
        void Render();

    private:
        void RefreshSessions();
        void RenderSessionList();
        void RenderSessionViewer();

        std::string logDirectory;
        std::vector<SessionInfo> sessions;
        int selectedIndex;
        bool needsRefresh;

        std::vector<ProfileResult> loadedResults;
        std::string loadedSessionName;
        std::vector<FrameSummary> loadedFrameSummaries;

        ProfilerUI profilerUI;
    };
}