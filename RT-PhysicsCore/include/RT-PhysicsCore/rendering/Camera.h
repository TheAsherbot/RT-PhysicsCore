#pragma once

#include <glm/glm.hpp>

namespace RT_PhysicsCore
{
    class Input;

    // Free-fly debug camera. Owned by Renderer, driven by Input each frame
    // via ProcessInput(). Movement is relative to where the camera is
    // looking (W always moves "forward" from the camera's own point of
    // view, not along a fixed world axis).
    //
    // Mouse look is active only while the right mouse button is held -
    // Camera is what decides *when* to capture the cursor (that's camera
    // policy); Input is what actually does it (that's an input
    // mechanism). Holding to look, rather than a permanent capture or a
    // toggle, keeps the mouse free the rest of the time - useful for a
    // debug/tool camera where you'll want to click elsewhere (a future
    // debug panel, another window) far more often than in an FPS game.
    class Camera
    {
    public:
        explicit Camera(glm::vec3 startPosition = glm::vec3(0.0f, 2.0f, 8.0f));

        void ProcessInput(Input& input, float deltaTime);

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

    private:
        glm::vec3 Front() const;
        glm::vec3 Right() const;
        glm::vec3 Up() const;
    };
}
