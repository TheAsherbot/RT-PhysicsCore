/**
 * @file SessionLoader.h
 * @brief Parses Chrome Tracing JSON session files into in-memory ProfileResult collections.
 */

#pragma once

#include "RT-PhysicsCore/telemetry/ProfileResult.h"

#include <string>
#include <vector>

namespace RT_PhysicsCore
{
    /**
     * @class SessionLoader
     * @brief Reads and deserializes JSON trace files produced by TelemetryManager.
     */
    class SessionLoader
    {
    public:
        /**
         * @brief Loads all trace events from a Chrome Tracing JSON file on disk.
         * @param filepath Path to the JSON trace file.
         * @param outResults Output vector populated with deserialized profile results.
         * @return True if the file was successfully parsed, false on error.
         */
        static bool Load(const std::string& filepath, std::vector<ProfileResult>& outResults);

        /**
         * @brief Scans the telemetry log directory and returns metadata for all discovered sessions.
         * @param directory Path to the directory containing trace JSON files.
         * @return Vector of SessionInfo descriptors sorted by timestamp descending (newest first).
         */
        static std::vector<SessionInfo> DiscoverSessions(const std::string& directory);

        /**
         * @brief Toggles the starred state of a session file by renaming it on disk.
         * @param session The session to toggle. Its filepath and isStarred fields are updated in place.
         * @return True if the rename succeeded.
         */
        static bool ToggleStar(SessionInfo& session);

        /**
         * @brief Deletes a session file from disk. Refuses to delete starred sessions.
         * @param session The session to delete.
         * @return True if the file was deleted, false if starred or on filesystem error.
         */
        static bool DeleteSession(const SessionInfo& session);

        /**
         * @brief Deletes all unstarred session files in the given list.
         * @param sessions The list of sessions to process. Starred sessions are skipped.
         * @return Number of files successfully deleted.
         */
        static int DeleteAllUnstarred(std::vector<SessionInfo>& sessions);
        /**
         * @brief Reconstructs frame summary data from raw profile results by analyzing scope nesting.
         * @param results The loaded profile results to analyze.
         * @return Vector of FrameSummary entries, one per "Frame" scope found.
         */
        static std::vector<FrameSummary> ReconstructFrameSummaries(const std::vector<ProfileResult>& results);
    };
}