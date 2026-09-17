#include "RT-PhysicsCore/rendering/Camera.h"
#include "RT-PhysicsCore/utils/Log.h"
#include "RT-PhysicsCore/utils/DebugDraw.h"

#include <GLFW/glfw3.h>
#include <algorithm>
#include <cmath>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

namespace RT_PhysicsCore
{
    Camera::Camera(glm::vec3 startPosition) : position(startPosition)
    {}

    glm::vec3 Camera::Front() const
    {
        float yawRad = glm::radians(yawDegrees);
        float pitchRad = glm::radians(pitchDegrees);
        glm::vec3 front;
        front.x = std::cos(yawRad) * std::cos(pitchRad);
        front.y = std::sin(pitchRad);
        front.z = std::sin(yawRad) * std::cos(pitchRad);
        return glm::normalize(front);
    }

    glm::vec3 Camera::Right() const
    {
        return glm::normalize(glm::cross(Front(), glm::vec3(0.0f, 1.0f, 0.0f)));
    }

    glm::vec3 Camera::Up() const
    {
        return glm::normalize(glm::cross(Right(), Front()));
    }

    void Camera::SetMode(Mode newMode)
    {
        if (newMode == mode)
        {
            return;
        }

        if (newMode == Mode::Orbit)
        {
            orbitTarget = position + Front() * orbitDistance;
        }

        mode = newMode;
        RT_LOG_INFO("Camera: switched to " << (mode == Mode::Orbit ? "Orbit" : "Free-fly") << " mode");
    }

    void Camera::ProcessInput(Input& input, float deltaTime)
    {
        // Capture cursor while right mouse button is held down; release when not
        bool isLookButtonDown = input.IsMouseButtonDown(GLFW_MOUSE_BUTTON_RIGHT);
		input.SetCursorCaptured(isLookButtonDown);

        // --- Mode toggle (Tab) using edge-triggered check from Input ---
        if (input.WasKeyPressed(GLFW_KEY_TAB))
        {
            SetMode(mode == Mode::FreeFly ? Mode::Orbit : Mode::FreeFly);
        }

        // Consume scroll accumulated via ProcessScroll
        double scrollThisFrame = input.MouseScrollDeltaY() - pendingScrollDelta;
        pendingScrollDelta = input.MouseScrollDeltaY();

        float velocity = moveSpeed * deltaTime;
        glm::vec3 front = Front();
        glm::vec3 right = Right();

        if (mode == Mode::FreeFly)
        {
            if (input.IsKeyDown(GLFW_KEY_W))            position += front * velocity;
            if (input.IsKeyDown(GLFW_KEY_S))            position -= front * velocity;
            if (input.IsKeyDown(GLFW_KEY_A))            position -= right * velocity;
            if (input.IsKeyDown(GLFW_KEY_D))            position += right * velocity;
            if (input.IsKeyDown(GLFW_KEY_SPACE))        position += glm::vec3(0.0f, 1.0f, 0.0f) * velocity;
            if (input.IsKeyDown(GLFW_KEY_LEFT_CONTROL)) position -= glm::vec3(0.0f, 1.0f, 0.0f) * velocity;
        }
        else // Mode::Orbit
        {
            if (input.IsKeyDown(GLFW_KEY_W))            orbitTarget += front * velocity;
            if (input.IsKeyDown(GLFW_KEY_S))            orbitTarget -= front * velocity;
            if (input.IsKeyDown(GLFW_KEY_A))            orbitTarget -= right * velocity;
            if (input.IsKeyDown(GLFW_KEY_D))            orbitTarget += right * velocity;
            if (input.IsKeyDown(GLFW_KEY_SPACE))        orbitTarget += glm::vec3(0.0f, 1.0f, 0.0f) * velocity;
            if (input.IsKeyDown(GLFW_KEY_LEFT_CONTROL)) orbitTarget -= glm::vec3(0.0f, 1.0f, 0.0f) * velocity;

            orbitDistance -= static_cast<float>(scrollThisFrame) * zoomSpeed;

            orbitDistance = std::clamp(orbitDistance, minOrbitDistance, maxOrbitDistance);
        }

        // Apply mouse-look only when the cursor is captured
        if (input.IsCursorCaptured())
        {
            double deltaX = input.MouseDeltaX();
            // Input class delivers raw screen delta where downward is positive; invert for pitch look-up
            double deltaY = -input.MouseDeltaY();

            yawDegrees += static_cast<float>(deltaX) * mouseSensitivity;
            pitchDegrees += static_cast<float>(deltaY) * mouseSensitivity;
            pitchDegrees = std::clamp(pitchDegrees, -89.0f, 89.0f);
        }

        if (mode == Mode::Orbit)
        {
            position = orbitTarget - Front() * orbitDistance;
			DebugDraw::Sphere(orbitTarget, 0.25f, glm::vec3(1.0f, 0.0f, 0.0f), 32, false); // small red sphere at camera position
        }
    }

    glm::mat4 Camera::GetViewMatrix() const
    {
        return glm::lookAt(position, position + Front(), Up());
    }

    glm::mat4 Camera::GetProjectionMatrix(float aspectRatio) const
    {
        return glm::perspective(glm::radians(fovDegrees), aspectRatio, nearPlane, farPlane);
    }
}