/**
 * @file TelemetryOverlay.cpp
 * @brief Implementation of the Mode 2 telemetry heads-up display.
 */

#include "RT-PhysicsCore/telemetry/TelemetryOverlay.h"
#include "RT-PhysicsCore/telemetry/TelemetryManager.h"
#include "RT-PhysicsCore/telemetry/MemoryTracker.h"

#include <cstddef>
#include <imgui/imgui.h>

namespace RT_PhysicsCore
{
    TelemetryOverlay::TelemetryOverlay()
    {}

    TelemetryOverlay::~TelemetryOverlay()
    {}

    void TelemetryOverlay::Render()
    {
        TelemetryManager& manager = TelemetryManager::Get();
        if (manager.GetMode() != TelemetryMode::Minimal)
        {
            return;
        }

        const auto& history = manager.GetFrameHistory();
        if (history.empty())
        {
            return;
        }

        double totalFrameSum = 0.0;
        double physicsSum = 0.0;
        double updateSum = 0.0;
        double renderSum = 0.0;

        for (const FrameSummary& summary : history)
        {
            totalFrameSum += summary.totalFrameMs;
            physicsSum += summary.physicsMs;
            updateSum += summary.updateMs;
            renderSum += summary.renderMs;
        }

        double count = static_cast<double>(history.size());
        double avgFrameMs = totalFrameSum / count;
        double avgPhysicsMs = physicsSum / count;
        double avgUpdateMs = updateSum / count;
        double avgRenderMs = renderSum / count;
        double fps = (avgFrameMs > 0.0) ? (1000.0 / avgFrameMs) : 0.0;

        std::size_t currentBytes = MemoryTracker::GetCurrentBytes();
        std::size_t peakBytes = MemoryTracker::GetPeakBytes();
        double currentMb = static_cast<double>(currentBytes) / (1024.0 * 1024.0);
        double peakMb = static_cast<double>(peakBytes) / (1024.0 * 1024.0);

        ImGuiWindowFlags windowFlags = ImGuiWindowFlags_NoDecoration |
            ImGuiWindowFlags_AlwaysAutoResize |
            ImGuiWindowFlags_NoSavedSettings |
            ImGuiWindowFlags_NoFocusOnAppearing |
            ImGuiWindowFlags_NoNav;

        const float distance = 10.0f;
        ImVec2 windowPos = ImVec2(distance, distance);
        ImGui::SetNextWindowPos(windowPos, ImGuiCond_Always);
        ImGui::SetNextWindowBgAlpha(0.65f);

        if (ImGui::Begin("Telemetry HUD", nullptr, windowFlags))
        {
            ImGui::Text("Performance (1s avg)");
            ImGui::Separator();
            ImGui::Text("FPS:          %.1f", fps);
            ImGui::Text("Frame Time:   %.2f ms", avgFrameMs);
            ImGui::Text("Physics Step: %.2f ms", avgPhysicsMs);
            ImGui::Text("Update:       %.2f ms", avgUpdateMs);
            ImGui::Text("Render:       %.2f ms", avgRenderMs);
            ImGui::Separator();
            ImGui::Text("Heap Memory:  %.2f MB (Peak: %.2f MB)", currentMb, peakMb);
        }
        ImGui::End();
    }
}