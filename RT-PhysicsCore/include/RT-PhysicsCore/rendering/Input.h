/**
 * @file Input.h
 * @brief Keyboard and mouse input polling mechanism and cursor capture manager.
 *
 * Polls GLFW device states once per frame and provides discrete press detection,
 * mouse displacement deltas, and FPS mouse-lock capture.
 */

#pragma once

struct GLFWwindow;

namespace RT_PhysicsCore
{
    /**
     * @class Input
     * @brief Polling-based window input manager for keyboard and mouse events.
     */
    class Input
    {
    public:
        /**
         * @brief Default constructor. Unattached instance safely reports no activity.
         */
        Input() = default;

        /**
         * @brief Attaches this input manager to an active GLFW window.
         * @param window Pointer to the underlying GLFWwindow.
         */
        void AttachWindow(GLFWwindow* window);

        /**
         * @brief Polls device states. Must be called once per frame before querying input.
         */
        void Update();

        /**
         * @brief Tests if a keyboard key is currently held down.
         * @param glfwKeyCode Standard GLFW key token (e.g. GLFW_KEY_W).
         * @return True if the key is depressed.
         */
        bool IsKeyDown(int glfwKeyCode) const;

        /**
         * @brief Tests if a keyboard key was pressed down during this specific frame.
         * @param glfwKeyCode Standard GLFW key token (e.g. GLFW_KEY_TAB).
         * @return True only on the leading-edge transition frame.
         */
        bool WasKeyPressed(int glfwKeyCode) const;

        /**
         * @brief Tests if a mouse button is currently held down.
         * @param glfwMouseButton Standard GLFW mouse button token (e.g. GLFW_MOUSE_BUTTON_RIGHT).
         * @return True if the mouse button is depressed.
         */
        bool IsMouseButtonDown(int glfwMouseButton) const;

        /**
         * @brief Gets current horizontal mouse coordinate in window screen pixels.
         * @return X cursor coordinate.
         */
        double MouseX() const;

        /**
         * @brief Gets current vertical mouse coordinate in window screen pixels.
         * @return Y cursor coordinate.
         */
        double MouseY() const;

        /**
         * @brief Gets horizontal cursor displacement since the previous frame.
         * @return Delta X in pixels.
         */
        double MouseDeltaX() const;

        /**
         * @brief Gets vertical cursor displacement since the previous frame (positive downward).
         * @return Delta Y in pixels.
         */
        double MouseDeltaY() const;

        /**
         * @brief Enables or disables locked cursor mode for mouse-look navigation.
         *
         * Re-baselines cursor tracking immediately on change to avoid artificial displacement jumps.
         *
         * @param captured True to lock and hide cursor; false to restore standard cursor.
         */
        void SetCursorCaptured(bool captured);

        /**
         * @brief Checks if cursor-capture mode is currently active.
         * @return True if the cursor is captured and hidden.
         */
        bool IsCursorCaptured() const;

        /**
         * @brief Gets total vertical scroll offset accumulated since initialization.
         * @return Cumulative vertical scroll offset.
         */
        double MouseScrollDeltaY() const;

    private:
        static constexpr int kMaxKeys = 512;
        static constexpr int kMaxMouseButtons = 8;

        static void ScrollCallback(GLFWwindow* window, double xoffset, double yoffset);

        GLFWwindow* window = nullptr;

        bool currentKeys[kMaxKeys] = {};
        bool previousKeys[kMaxKeys] = {};
        bool currentMouseButtons[kMaxMouseButtons] = {};

        double mouseX = 0.0;
        double mouseY = 0.0;
        double lastMouseX = 0.0;
        double lastMouseY = 0.0;
        double mouseDeltaX = 0.0;
        double mouseDeltaY = 0.0;
        double mouseScrollDeltaY = 0.0;
        bool firstUpdate = true;

        bool cursorCaptured = false;
    };
}