/**
 * @file ProfilerUI.h
 * @brief Reusable ImGui-based flame graph and profiler statistics panel.
 */

#pragma once

#include "RT-PhysicsCore/telemetry/ProfileResult.h"

#include <vector>

namespace RT_PhysicsCore
{
    /**
     * @class ProfilerUI
     * @brief Renders an interactive flame graph and frame statistics using Dear ImGui draw commands.
     */
    class ProfilerUI
    {
    public:
        /**
         * @brief Default constructor.
         */
        ProfilerUI();

        /**
         * @brief Destructor.
         */
        ~ProfilerUI();

        ProfilerUI(const ProfilerUI&) = delete;
        ProfilerUI& operator=(const ProfilerUI&) = delete;

        /**
         * @brief Draws the full profiler interface including flame graph and frame history.
         * @param results The collection of profile scope results to visualize.
         * @param history The rolling buffer of frame summaries for the timeline graph.
         */
        void Render(const std::vector<ProfileResult>& results, const std::vector<FrameSummary>& history);

    private:
        float scrollOffsetX;
        float zoomLevel;
        int selectedFrameIndex;
    };
}