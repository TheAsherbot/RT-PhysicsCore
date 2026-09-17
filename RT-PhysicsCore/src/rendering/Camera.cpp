#include "RT-PhysicsCore/rendering/Camera.h"
#include "RT-PhysicsCore/rendering/Input.h"
#include <GLFW/glfw3.h>
#include <algorithm>
#include <cmath>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>   // optional but common


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

    void Camera::ProcessInput(Input& input, float deltaTime)
    {
        // Look around only while the right mouse button is held. Input
        // handles the actual cursor hide/lock and re-baselines the delta
        // the moment capture starts, so releasing and re-pressing never
        // causes the view to jump.
        if (input.IsMouseButtonDown(GLFW_MOUSE_BUTTON_RIGHT))
        {
            input.SetCursorCaptured(true);

            yawDegrees += static_cast<float>(input.MouseDeltaX()) * mouseSensitivity;
            pitchDegrees += static_cast<float>(-input.MouseDeltaY()) * mouseSensitivity; // screen Y grows downward
            pitchDegrees = std::clamp(pitchDegrees, -89.0f, 89.0f); // avoid gimbal flip at the poles
        }
        else
        {
            input.SetCursorCaptured(false);
        }

        float velocity = moveSpeed * deltaTime;
        glm::vec3 front = Front();
        glm::vec3 right = Right();

        if (input.IsKeyDown(GLFW_KEY_W)) position += front * velocity;
        if (input.IsKeyDown(GLFW_KEY_S)) position -= front * velocity;
        if (input.IsKeyDown(GLFW_KEY_A)) position -= right * velocity;
        if (input.IsKeyDown(GLFW_KEY_D)) position += right * velocity;
        if (input.IsKeyDown(GLFW_KEY_SPACE)) position += glm::vec3(0.0f, 1.0f, 0.0f) * velocity;
        if (input.IsKeyDown(GLFW_KEY_LEFT_CONTROL)) position -= glm::vec3(0.0f, 1.0f, 0.0f) * velocity;
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
