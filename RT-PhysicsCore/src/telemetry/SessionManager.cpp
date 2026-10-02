/**
 * @file SessionManager.cpp
 * @brief Implementation of the session browser and viewer UI.
 */

#include "RT-PhysicsCore/telemetry/SessionManager.h"
#include "RT-PhysicsCore/telemetry/SessionLoader.h"

#include <imgui/imgui.h>

#include <cstdio>

namespace RT_PhysicsCore
{
    SessionManager::SessionManager(const std::string& logDirectory)
        : logDirectory(logDirectory)
        , selectedIndex(-1)
        , needsRefresh(true)
    {}

    SessionManager::~SessionManager()
    {}

    void SessionManager::Render()
    {
        if (needsRefresh)
        {
            RefreshSessions();
            needsRefresh = false;
        }

        ImGuiViewport* viewport = ImGui::GetMainViewport();
        ImGui::SetNextWindowPos(viewport->WorkPos);
        ImGui::SetNextWindowSize(viewport->WorkSize);

        ImGuiWindowFlags flags = ImGuiWindowFlags_NoTitleBar |
            ImGuiWindowFlags_NoResize |
            ImGuiWindowFlags_NoMove |
            ImGuiWindowFlags_NoCollapse |
            ImGuiWindowFlags_NoBringToFrontOnFocus;

        if (ImGui::Begin("Session Manager", nullptr, flags))
        {
            ImGui::Text("RT Profiler Viewer");
            ImGui::SameLine(ImGui::GetWindowWidth() - 100.0f);
            if (ImGui::Button("Refresh"))
            {
                needsRefresh = true;
            }
            ImGui::Separator();

            // Two-panel layout: session list on left, viewer on right
            float panelWidth = 320.0f;

            ImGui::BeginChild("SessionListPanel", ImVec2(panelWidth, 0), true);
            RenderSessionList();
            ImGui::EndChild();

            ImGui::SameLine();

            ImGui::BeginChild("SessionViewerPanel", ImVec2(0, 0), true);
            RenderSessionViewer();
            ImGui::EndChild();
        }
        ImGui::End();
    }

    void SessionManager::RefreshSessions()
    {
        sessions = SessionLoader::DiscoverSessions(logDirectory);

        // If the previously selected session was deleted, clear the selection
        if (selectedIndex >= static_cast<int>(sessions.size()))
        {
            selectedIndex = -1;
            loadedResults.clear();
            loadedSessionName.clear();
        }
    }

    void SessionManager::RenderSessionList()
    {
        ImGui::Text("Sessions (%zu)", sessions.size());
        ImGui::Separator();

        // Bulk actions
        if (ImGui::Button("Delete All Unstarred"))
        {
            ImGui::OpenPopup("ConfirmDeleteAll");
        }

        if (ImGui::BeginPopupModal("ConfirmDeleteAll", nullptr, ImGuiWindowFlags_AlwaysAutoResize))
        {
            ImGui::Text("This will permanently delete all unstarred sessions.");
            ImGui::Text("Are you sure?");
            ImGui::Separator();

            if (ImGui::Button("Yes, Delete", ImVec2(120, 0)))
            {
                int deleted = SessionLoader::DeleteAllUnstarred(sessions);
                (void)deleted;
                selectedIndex = -1;
                loadedResults.clear();
                loadedSessionName.clear();
                ImGui::CloseCurrentPopup();
            }
            ImGui::SameLine();
            if (ImGui::Button("Cancel", ImVec2(120, 0)))
            {
                ImGui::CloseCurrentPopup();
            }
            ImGui::EndPopup();
        }

        ImGui::Separator();

        // Session list
        for (int i = 0; i < static_cast<int>(sessions.size()); ++i)
        {
            SessionInfo& session = sessions[i];

            ImGui::PushID(i);

            // Star button
            const char* starLabel = session.isStarred ? "[*]" : "[ ]";
            if (ImGui::SmallButton(starLabel))
            {
                SessionLoader::ToggleStar(session);
            }
            ImGui::SameLine();

            // Session entry - selectable
            bool isSelected = (i == selectedIndex);
            char label[256];
            double sizeMb = static_cast<double>(session.fileSizeBytes) / (1024.0 * 1024.0);
            snprintf(label, sizeof(label), "%s (%.1f MB)", session.displayName.c_str(), sizeMb);

            if (ImGui::Selectable(label, isSelected))
            {
                selectedIndex = i;
                loadedResults.clear();
                loadedFrameSummaries.clear();
                loadedSessionName.clear();

                if (SessionLoader::Load(session.filepath, loadedResults))
                {
                    loadedSessionName = session.displayName;
                    loadedFrameSummaries = SessionLoader::ReconstructFrameSummaries(loadedResults);
                }
            }

            // Right-click context menu
            if (ImGui::BeginPopupContextItem())
            {
                if (ImGui::MenuItem(session.isStarred ? "Unstar" : "Star"))
                {
                    SessionLoader::ToggleStar(session);
                }

                if (session.isStarred)
                {
                    ImGui::BeginDisabled();
                    ImGui::MenuItem("Delete (starred)");
                    ImGui::EndDisabled();
                }
                else
                {
                    if (ImGui::MenuItem("Delete"))
                    {
                        SessionLoader::DeleteSession(session);
                        sessions.erase(sessions.begin() + i);
                        if (selectedIndex == i)
                        {
                            selectedIndex = -1;
                            loadedResults.clear();
                            loadedSessionName.clear();
                        }
                        else if (selectedIndex > i)
                        {
                            --selectedIndex;
                        }
                        ImGui::EndPopup();
                        ImGui::PopID();
                        return;
                    }
                }

                ImGui::EndPopup();
            }

            ImGui::PopID();
        }
    }

    void SessionManager::RenderSessionViewer()
    {
        if (loadedResults.empty())
        {
            ImGui::TextDisabled("Select a session from the list to view its flame graph.");
            return;
        }

        ImGui::Text("Viewing: %s  (%zu events)", loadedSessionName.c_str(), loadedResults.size());
        ImGui::Separator();

        // Reuse the same ProfilerUI that the live profiler uses
        profilerUI.Render(loadedResults, loadedFrameSummaries);
    }
}