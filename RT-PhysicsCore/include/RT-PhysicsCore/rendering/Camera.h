#pragma once

#include <glm/glm.hpp>
#include "RT-PhysicsCore/rendering/Input.h"

struct GLFWwindow;

namespace RT_PhysicsCore
{
    // Free-fly / orbit debug camera. Owned by Renderer, driven by GLFW input
    // each frame via ProcessInput().
    //
    // Two modes, cycled with Tab (logged via Log each time it switches):
    //   Mode::FreeFly - WASD/Space/Ctrl move the camera itself; mouse looks
    //                   around. W always moves "forward" from the camera's
    //                   own point of view, not along a fixed world axis.
    //   Mode::Orbit   - WASD/Space/Ctrl move a pivot point (orbitTarget)
    //                   instead of the camera; mouse swings the camera
    //                   around that pivot; scroll zooms (changes distance
    //                   to the pivot). The camera always faces the pivot.
    // Switching modes never jumps the view: entering Orbit drops the pivot
    // out in front of wherever the camera currently is, and leaving Orbit
    // just continues from the camera's current position.
    //
    // Mouse look/orbit only takes effect while the cursor is disabled/
    // captured (GLFW_CURSOR_DISABLED) - Renderer is expected to enable that
    // mode, typically in response to a click or a dedicated toggle key, so
    // the camera doesn't spin the moment the window gets focus. WASD/Space/
    // Ctrl and scroll are NOT gated behind cursor capture.
    //
    // GLFW has no polling API for scroll, so it has to reach the camera via
    // callback: Renderer's GLFW scroll callback should call
    // camera.ProcessScroll(yoffset) each time it fires.
    class Camera
    {
    public:
        enum class Mode
        {
            FreeFly,
            Orbit
        };

        explicit Camera(glm::vec3 startPosition = glm::vec3(0.0f, 2.0f, 8.0f));

        void ProcessInput(Input& input, float deltaTime);

        void SetMode(Mode newMode);
        Mode GetMode() const { return mode; }

        glm::mat4 GetViewMatrix() const;
        glm::mat4 GetProjectionMatrix(float aspectRatio) const;

        glm::vec3 position;
        float yawDegrees{ -90.0f };   // -90 so the default facing direction is -Z
        float pitchDegrees{ 0.0f };

        float fovDegrees{ 45.0f };
        float nearPlane{ 0.1f };
        float farPlane{ 500.0f };

        float moveSpeed{ 5.0f };          // world units per second
        float mouseSensitivity{ 0.1f };   // degrees per pixel of mouse delta

        // Orbit mode only
        glm::vec3 orbitTarget{ 0.0f, 0.0f, 0.0f }; // pivot the camera swings around / WASD moves
        float orbitDistance{ 10.0f };              // distance from orbitTarget to the camera
        float minOrbitDistance{ 1.0f };
        float maxOrbitDistance{ 100.0f };
        float zoomSpeed{ 1.0f };                   // world units per scroll tick

    private:
        glm::vec3 Front() const;
        glm::vec3 Right() const;
        glm::vec3 Up() const;

        Mode mode{ Mode::FreeFly };

        double lastMouseX{ 0.0 };
        double lastMouseY{ 0.0 };
        bool firstMouseSample{ true };

        bool tabWasPressed{ false };
        double pendingScrollDelta{ 0.0 };
    };
}