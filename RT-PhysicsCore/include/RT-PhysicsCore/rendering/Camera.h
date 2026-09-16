#pragma once

#include <glm/glm.hpp>

struct GLFWwindow;

namespace RT_PhysicsCore
{
    // Free-fly debug camera. Owned by Renderer, driven by GLFW input each
    // frame via ProcessInput(). Movement is relative to where the camera is
    // looking (W always moves "forward" from the camera's own point of
    // view, not along a fixed world axis).
    //
    // Mouse look only takes effect while the cursor is disabled/captured
    // (GLFW_CURSOR_DISABLED) - Renderer is expected to enable that mode,
    // typically in response to a click or a dedicated toggle key, so the
    // camera doesn't spin the moment the window gets focus.
    class Camera
    {
    public:
        explicit Camera(glm::vec3 startPosition = glm::vec3(0.0f, 2.0f, 8.0f));

        void ProcessInput(GLFWwindow* window, float deltaTime);

        glm::mat4 GetViewMatrix() const;
        glm::mat4 GetProjectionMatrix(float aspectRatio) const;

        glm::vec3 position;
        float yawDegrees{-90.0f};   // -90 so the default facing direction is -Z
        float pitchDegrees{0.0f};

        float fovDegrees{45.0f};
        float nearPlane{0.1f};
        float farPlane{500.0f};

        float moveSpeed{5.0f};          // world units per second
        float mouseSensitivity{0.1f};   // degrees per pixel of mouse delta

    private:
        glm::vec3 Front() const;
        glm::vec3 Right() const;
        glm::vec3 Up() const;

        double lastMouseX{0.0};
        double lastMouseY{0.0};
        bool firstMouseSample{true};
    };
}
