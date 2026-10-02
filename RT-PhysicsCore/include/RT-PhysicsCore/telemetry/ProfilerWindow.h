/**
 * @file ProfilerWindow.h
 * @brief Manages a dedicated secondary GLFW window for full profiler visualization.
 */

#pragma once

struct GLFWwindow;

namespace RT_PhysicsCore
{
    class ProfilerUI;

    /**
     * @class ProfilerWindow
     * @brief Owns a second GLFW window and ImGui context for rendering the Mode 3 flame graph.
     */
    class ProfilerWindow
    {
    public:
        /**
         * @brief Constructs the profiler window manager.
         */
        ProfilerWindow();

        /**
         * @brief Destructor. Closes and cleans up the profiler window if open.
         */
        ~ProfilerWindow();

        ProfilerWindow(const ProfilerWindow&) = delete;
        ProfilerWindow& operator=(const ProfilerWindow&) = delete;

        /**
         * @brief Opens the secondary profiler window sharing the main window's GL context.
         * @param mainWindow The primary simulation GLFW window used for context sharing.
         */
        void Open(GLFWwindow* mainWindow);

        /**
         * @brief Closes and destroys the secondary profiler window and its ImGui context.
         * @param mainWindow The primary GLFW window to restore as the active GL context. May be nullptr during final shutdown.
         */
        void Close(GLFWwindow* mainWindow = nullptr);

        /**
         * @brief Checks whether the profiler window is currently open and active.
         * @return True if the window exists and has not been closed.
         */
        bool IsOpen() const;

        /**
         * @brief Renders one frame of the profiler UI into the secondary window.
         * @param mainWindow The primary simulation window, used to restore GL context after rendering.
         */
        void Render(GLFWwindow* mainWindow);

    private:
        GLFWwindow* window;
        void* imguiContext;
        ProfilerUI* profilerUI;
    };
}