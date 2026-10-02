/**
 * @file ProfilerWindow.cpp
 * @brief Implementation of the secondary GLFW profiler window and its ImGui context.
 */

#include "RT-PhysicsCore/telemetry/ProfilerWindow.h"
#include "RT-PhysicsCore/telemetry/ProfilerUI.h"
#include "RT-PhysicsCore/telemetry/TelemetryManager.h"
#include "RT-PhysicsCore/utils/Log.h"

#include <glad/gl.h>
#include <GLFW/glfw3.h>
#include <imgui/imgui.h>
#include <imgui/backend/imgui_impl_glfw.h>
#include <imgui/backend/imgui_impl_opengl3.h>

namespace RT_PhysicsCore
{
    ProfilerWindow::ProfilerWindow()
        : window(nullptr)
        , imguiContext(nullptr)
        , profilerUI(nullptr)
    {}

    ProfilerWindow::~ProfilerWindow()
    {
        Close();
    }

    void ProfilerWindow::Open(GLFWwindow* mainWindow)
    {
        if (window)
        {
            return;
        }

        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
        glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
        glfwWindowHint(GLFW_VISIBLE, GLFW_TRUE);
        glfwWindowHint(GLFW_FOCUSED, GLFW_FALSE);

        window = glfwCreateWindow(960, 640, "RT Profiler", nullptr, mainWindow);
        if (!window)
        {
            RT_LOG_ERROR("Failed to create profiler window");
            return;
        }

        // Switch to the new window's context to initialize its GL + ImGui state
        glfwMakeContextCurrent(window);
        gladLoadGL(glfwGetProcAddress);

        ImGuiContext* previousContext = ImGui::GetCurrentContext();

        imguiContext = ImGui::CreateContext();
        ImGui::SetCurrentContext(static_cast<ImGuiContext*>(imguiContext));

        ImGui::StyleColorsDark();
        ImGui_ImplGlfw_InitForOpenGL(window, true);
        ImGui_ImplOpenGL3_Init("#version 330 core");

        // Restore the main window context
        ImGui::SetCurrentContext(previousContext);
        glfwMakeContextCurrent(mainWindow);

        profilerUI = new ProfilerUI();

        RT_LOG_INFO("Profiler window opened");
    }

    void ProfilerWindow::Close(GLFWwindow* mainWindow)
    {
        if (!window)
        {
            return;
        }

        glfwMakeContextCurrent(window);

        ImGuiContext* previousContext = ImGui::GetCurrentContext();
        ImGui::SetCurrentContext(static_cast<ImGuiContext*>(imguiContext));

        ImGui_ImplOpenGL3_Shutdown();
        ImGui_ImplGlfw_Shutdown();
        ImGui::DestroyContext(static_cast<ImGuiContext*>(imguiContext));

        ImGui::SetCurrentContext(previousContext);

        glfwDestroyWindow(window);
        window = nullptr;
        imguiContext = nullptr;

        delete profilerUI;
        profilerUI = nullptr;

        // Restore the main window's GL context so rendering can continue
        if (mainWindow)
        {
            glfwMakeContextCurrent(mainWindow);
        }

        RT_LOG_INFO("Profiler window closed");
    }

    bool ProfilerWindow::IsOpen() const
    {
        return window != nullptr;
    }

    void ProfilerWindow::Render(GLFWwindow* mainWindow)
    {
        if (!window)
        {
            return;
        }

        // Check if the user closed the profiler window via its X button
        if (glfwWindowShouldClose(window))
        {
            // Must restore main context before Close() tries to switch
            Close();
            glfwMakeContextCurrent(mainWindow);
            TelemetryManager::Get().SetMode(TelemetryMode::Minimal);
            return;
        }

        // Save the main ImGui context and switch to the profiler's context
        ImGuiContext* mainContext = ImGui::GetCurrentContext();

        glfwMakeContextCurrent(window);
        ImGui::SetCurrentContext(static_cast<ImGuiContext*>(imguiContext));

        int width = 0;
        int height = 0;
        glfwGetFramebufferSize(window, &width, &height);
        glViewport(0, 0, width, height);
        glClearColor(0.12f, 0.12f, 0.14f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // Fullscreen profiler window
        ImGui::SetNextWindowPos(ImVec2(0.0f, 0.0f));
        ImGui::SetNextWindowSize(ImVec2(static_cast<float>(width), static_cast<float>(height)));

        ImGuiWindowFlags flags = ImGuiWindowFlags_NoTitleBar |
            ImGuiWindowFlags_NoResize |
            ImGuiWindowFlags_NoMove |
            ImGuiWindowFlags_NoCollapse;

        if (ImGui::Begin("Profiler", nullptr, flags))
        {
            TelemetryManager& manager = TelemetryManager::Get();
            profilerUI->Render(manager.GetLiveResults(), manager.GetFrameHistory());
        }
        ImGui::End();

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);

        // Restore the main window's context and ImGui context
        ImGui::SetCurrentContext(mainContext);
        glfwMakeContextCurrent(mainWindow);
    }
}