#include "RT-PhysicsCore/rendering/Camera.h"
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

    void Camera::ProcessInput(GLFWwindow* window, float deltaTime)
    {
        float velocity = moveSpeed * deltaTime;
        glm::vec3 front = Front();
        glm::vec3 right = Right();

        if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) position += front * velocity;
        if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) position -= front * velocity;
        if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) position -= right * velocity;
        if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) position += right * velocity;
        if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS) position += glm::vec3(0.0f, 1.0f, 0.0f) * velocity;
        if (glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS) position -= glm::vec3(0.0f, 1.0f, 0.0f) * velocity;

        if (glfwGetInputMode(window, GLFW_CURSOR) == GLFW_CURSOR_DISABLED)
        {
            double mouseX, mouseY;
            glfwGetCursorPos(window, &mouseX, &mouseY);

            if (firstMouseSample)
            {
                lastMouseX = mouseX;
                lastMouseY = mouseY;
                firstMouseSample = false;
            }

            double deltaX = mouseX - lastMouseX;
            double deltaY = lastMouseY - mouseY; // screen Y grows downward; pitch should increase looking up
            lastMouseX = mouseX;
            lastMouseY = mouseY;

            yawDegrees += static_cast<float>(deltaX) * mouseSensitivity;
            pitchDegrees += static_cast<float>(deltaY) * mouseSensitivity;
            pitchDegrees = std::clamp(pitchDegrees, -89.0f, 89.0f); // avoid gimbal flip at the poles
        }
        else
        {
            // Cursor isn't captured - next time it is, don't jump using a
            // stale delta from wherever the mouse happened to be.
            firstMouseSample = true;
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
