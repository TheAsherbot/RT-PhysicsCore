/**
 * @file ProfilerUI.cpp
 * @brief Implementation of the flame graph and profiler statistics rendering.
 */

#include "RT-PhysicsCore/telemetry/ProfilerUI.h"
#include "RT-PhysicsCore/telemetry/MemoryTracker.h"

#include <imgui/imgui.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <unordered_map>

namespace RT_PhysicsCore
{
    ProfilerUI::ProfilerUI()
        : scrollOffsetX(0.0f)
        , zoomLevel(1.0f)
        , selectedFrameIndex(-1)
    {}

    ProfilerUI::~ProfilerUI()
    {}

    void ProfilerUI::Render(const std::vector<ProfileResult>& results, const std::vector<FrameSummary>& history)
    {
        // --- Frame History Timeline ---
        if (ImGui::CollapsingHeader("Frame Timeline", ImGuiTreeNodeFlags_DefaultOpen))
        {
            if (!history.empty())
            {
                int count = static_cast<int>(history.size());

                // Dynamically size the timeline array instead of hardcapping at 120
                std::vector<float> frameTimes(count);
                for (int i = 0; i < count; ++i)
                {
                    frameTimes[i] = static_cast<float>(history[i].totalFrameMs);
                }

                float maxVal = *std::max_element(frameTimes.begin(), frameTimes.end());
                if (maxVal < 1.0f)
                {
                    maxVal = 1.0f;
                }
                char overlay[64];
                snprintf(overlay, sizeof(overlay), "%.1f ms", frameTimes.back());
                ImGui::PlotLines("##FrameGraph", frameTimes.data(), count, 0, overlay, 0.0f, maxVal * 1.2f, ImVec2(0, 80));
            }
        }
        // Extract all frame scopes early so we can read memory from them
        std::vector<const ProfileResult*> frameResults;
        for (const ProfileResult& r : results)
        {
            if (r.name && std::string(r.name) == "Frame")
            {
                frameResults.push_back(&r);
            }
        }
        if (!frameResults.empty())
        {
            if (selectedFrameIndex < 0 || selectedFrameIndex >= static_cast<int>(frameResults.size()))
            {
                selectedFrameIndex = static_cast<int>(frameResults.size()) - 1; // Default to newest
            }
        }
        // --- Memory Stats ---
        std::size_t currentBytes = MemoryTracker::GetCurrentBytes();
        std::size_t peakBytes = MemoryTracker::GetPeakBytes();
        std::size_t allocCount = MemoryTracker::GetActiveAllocationCount();
        bool isLiveMemory = true;
        if (!frameResults.empty() && selectedFrameIndex >= 0 && selectedFrameIndex < static_cast<int>(frameResults.size()))
        {
            const ProfileResult* r = frameResults[selectedFrameIndex];
            if (r->memoryCurrentBytes > 0 || r->memoryPeakBytes > 0)
            {
                currentBytes = r->memoryCurrentBytes;
                peakBytes = r->memoryPeakBytes;
                allocCount = r->memoryAllocations;
                isLiveMemory = false;
            }
        }
        if (ImGui::CollapsingHeader(isLiveMemory ? "Live App Memory" : "Recorded Frame Memory", ImGuiTreeNodeFlags_DefaultOpen))
        {
            ImGui::Text("Current:     %.2f MB", static_cast<double>(currentBytes) / (1024.0 * 1024.0));
            ImGui::Text("Peak:        %.2f MB", static_cast<double>(peakBytes) / (1024.0 * 1024.0));
            ImGui::Text("Allocations: %zu", allocCount);
        }
        // --- Flame Graph ---
        if (ImGui::CollapsingHeader("Flame Graph", ImGuiTreeNodeFlags_DefaultOpen))
        {
            if (results.empty())
            {
                ImGui::TextDisabled("No profiling data captured yet.");
                return;
            }
            // Extract all frame scopes to allow scrubbing
            std::vector<const ProfileResult*> frameResults;
            for (const ProfileResult& r : results)
            {
                if (r.name && std::string(r.name) == "Frame")
                {
                    frameResults.push_back(&r);
                }
            }
            long long frameStart = results.front().startTimestamp;
            long long frameEnd = results.back().endTimestamp;
            if (!frameResults.empty())
            {
                if (selectedFrameIndex < 0 || selectedFrameIndex >= static_cast<int>(frameResults.size()))
                {
                    selectedFrameIndex = static_cast<int>(frameResults.size()) - 1; // Default to newest
                }
                // Add the slider to select a specific frame
                ImGui::SliderInt("Scrub Frame", &selectedFrameIndex, 0, static_cast<int>(frameResults.size()) - 1);
                ImGui::SameLine();
                if (ImGui::Button("Go to Latest"))
                {
                    selectedFrameIndex = static_cast<int>(frameResults.size()) - 1;
                }
                frameStart = frameResults[selectedFrameIndex]->startTimestamp;
                frameEnd = frameResults[selectedFrameIndex]->endTimestamp;
            }
            // Zoom controls
            ImGui::SliderFloat("Zoom", &zoomLevel, 0.1f, 50.0f, "%.1fx");
            if (ImGui::Button("Reset Zoom"))
            {
                zoomLevel = 1.0f;
                scrollOffsetX = 0.0f;
            }
            long long frameRange = frameEnd - frameStart;
            if (frameRange <= 0)
            {
                frameRange = 1;
            }

            // Assign depth by simple nesting detection within the last frame
            struct ScopeEntry
            {
                const ProfileResult* result;
                int depth;
            };

            std::vector<ScopeEntry> visibleScopes;
            std::vector<long long> depthEndStack;

            for (const ProfileResult& r : results)
            {
                if (r.startTimestamp < frameStart || r.endTimestamp > frameEnd + 1)
                {
                    continue;
                }

                // Pop scopes that have ended
                while (!depthEndStack.empty() && r.startTimestamp >= depthEndStack.back())
                {
                    depthEndStack.pop_back();
                }

                int depth = static_cast<int>(depthEndStack.size());
                depthEndStack.push_back(r.endTimestamp);

                visibleScopes.push_back({ &r, depth });
            }

            // Draw
            const float barHeight = 22.0f;
            const float padding = 2.0f;

            int maxDepth = 0;
            for (const ScopeEntry& entry : visibleScopes)
            {
                if (entry.depth > maxDepth)
                {
                    maxDepth = entry.depth;
                }
            }

            float canvasHeight = (maxDepth + 1) * (barHeight + padding) + padding;
            ImVec2 canvasSize = ImVec2(ImGui::GetContentRegionAvail().x, canvasHeight);

            ImGui::BeginChild("FlameCanvas", canvasSize, true, ImGuiWindowFlags_HorizontalScrollbar);

            ImVec2 canvasPos = ImGui::GetCursorScreenPos();
            float canvasWidth = ImGui::GetContentRegionAvail().x * zoomLevel;

            ImDrawList* drawList = ImGui::GetWindowDrawList();

            // Color palette based on scope name hash
            auto hashColor = [](const char* name) -> ImU32
                {
                    unsigned int hash = 2166136261u;
                    if (name)
                    {
                        for (const char* c = name; *c; ++c)
                        {
                            hash ^= static_cast<unsigned int>(*c);
                            hash *= 16777619u;
                        }
                    }
                    float h = static_cast<float>(hash % 360) / 360.0f;
                    return ImGui::ColorConvertFloat4ToU32(ImVec4(
                        0.5f + 0.5f * std::cos(6.2831853f * (h + 0.0f)),
                        0.5f + 0.5f * std::cos(6.2831853f * (h + 0.333f)),
                        0.5f + 0.5f * std::cos(6.2831853f * (h + 0.666f)),
                        0.85f));
                };

            for (const ScopeEntry& entry : visibleScopes)
            {
                const ProfileResult& r = *entry.result;

                float normalizedStart = static_cast<float>(r.startTimestamp - frameStart) / static_cast<float>(frameRange);
                float normalizedEnd = static_cast<float>(r.endTimestamp - frameStart) / static_cast<float>(frameRange);

                float x0 = canvasPos.x + normalizedStart * canvasWidth;
                float x1 = canvasPos.x + normalizedEnd * canvasWidth;
                float y0 = canvasPos.y + entry.depth * (barHeight + padding);
                float y1 = y0 + barHeight;

                // Skip bars that are too narrow to see
                if ((x1 - x0) < 1.0f)
                {
                    continue;
                }

                ImU32 color = hashColor(r.name);
                drawList->AddRectFilled(ImVec2(x0, y0), ImVec2(x1, y1), color, 2.0f);
                drawList->AddRect(ImVec2(x0, y0), ImVec2(x1, y1), IM_COL32(0, 0, 0, 80), 2.0f);

                // Label if the bar is wide enough
                float barWidth = x1 - x0;
                if (barWidth > 40.0f)
                {
                    float durationUs = static_cast<float>(r.endTimestamp - r.startTimestamp);
                    char label[128];
                    if (durationUs >= 1000.0f)
                    {
                        snprintf(label, sizeof(label), "%s (%.2f ms)", r.name ? r.name : "?", durationUs / 1000.0f);
                    }
                    else
                    {
                        snprintf(label, sizeof(label), "%s (%.0f us)", r.name ? r.name : "?", durationUs);
                    }

                    ImVec2 textSize = ImGui::CalcTextSize(label);
                    if (textSize.x < barWidth - 4.0f)
                    {
                        drawList->AddText(ImVec2(x0 + 2.0f, y0 + 2.0f), IM_COL32(255, 255, 255, 255), label);
                    }
                    else
                    {
                        // Just the name without duration
                        drawList->AddText(ImVec2(x0 + 2.0f, y0 + 2.0f), IM_COL32(255, 255, 255, 255), r.name ? r.name : "?");
                    }
                }

                // Tooltip on hover
                ImGui::SetCursorScreenPos(ImVec2(x0, y0));
                ImGui::InvisibleButton(("##scope" + std::to_string(reinterpret_cast<uintptr_t>(&r))).c_str(),
                    ImVec2(barWidth, barHeight));
                if (ImGui::IsItemHovered())
                {
                    ImGui::BeginTooltip();
                    ImGui::Text("Scope: %s", r.name ? r.name : "Unknown");
                    float durationUs = static_cast<float>(r.endTimestamp - r.startTimestamp);
                    ImGui::Text("Duration: %.3f ms (%.0f us)", durationUs / 1000.0f, durationUs);
                    ImGui::Text("Thread: %u", r.threadId);
                    ImGui::EndTooltip();
                }
            }

            // Reserve space so scrollbar works properly
            ImGui::Dummy(ImVec2(canvasWidth, canvasHeight));

            ImGui::EndChild();
        }
    }
}