#pragma once

struct GLFWwindow;

namespace RT_PhysicsCore
{
    // Polls keyboard and mouse state for one window, once per frame via
    // Update(). Also owns cursor-capture mode (hiding/locking the cursor
    // for FPS-style mouse look, via glfwSetInputMode) - that's an input
    // *mechanism*, not a camera *policy*. Camera decides *when* to
    // capture (e.g. while the right mouse button is held); Input is what
    // actually flips the GLFW input mode and hands back clean deltas
    // afterward, so any other code that wants keyboard/mouse state has
    // the same single place to get it from.
    //
    // Polling-based rather than GLFW-callback-based, matching the rest of
    // this project - simple, no static-trampoline plumbing to route C
    // callbacks back into a C++ object, and more than accurate enough for
    // a per-frame input use case.
    //
    // Default-constructed with no window attached (every query safely
    // returns "nothing happening"); Renderer calls AttachWindow() once
    // its window exists. This avoids Renderer needing a nullable/owning
    // pointer just to sequence construction.
    class Input
    {
    public:
        Input() = default;

        void AttachWindow(GLFWwindow* window);

        // Call once per frame, before reading any state below.
        void Update();

        bool IsKeyDown(int glfwKeyCode) const;
        bool WasKeyPressed(int glfwKeyCode) const; // true only on the frame it went down

        bool IsMouseButtonDown(int glfwMouseButton) const;

        double MouseX() const;
        double MouseY() const;
        double MouseDeltaX() const;
        double MouseDeltaY() const;

        // Hides and locks the cursor for mouse-look when true; restores
        // normal cursor behavior when false. Re-baselines the mouse
        // position on any change so the delta on the next Update() reflects
        // movement *after* the change, not a jump from wherever the
        // cursor happened to be beforehand.
        void SetCursorCaptured(bool captured);
        bool IsCursorCaptured() const;

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
