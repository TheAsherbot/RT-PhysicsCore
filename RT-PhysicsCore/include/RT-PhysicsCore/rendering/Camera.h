/**
 * @file Camera.h
 * @brief 3D perspective camera supporting Free-Fly and Orbit debug modes.
 *
 * Computes View and Projection matrices for 3D rendering and updates orientation/position
 * from keyboard, mouse-look, and scroll input.
 */

#pragma once

#include <glm/glm.hpp>
#include "RT-PhysicsCore/rendering/Input.h"

struct GLFWwindow;

namespace RT_PhysicsCore
{
    /**
     * @class Camera
     * @brief Dual-mode 3D camera with Free-Fly and Orbit navigation.
     */
    class Camera
    {
    public:
        /**
         * @enum Mode
         * @brief Navigation control style.
         */
        enum class Mode
        {
            FreeFly, ///< WASD moves camera along its own view axes; mouse looks freely.
            Orbit    ///< WASD moves orbitTarget pivot; mouse swings camera around pivot.
        };

        /**
         * @brief Constructs the camera at an initial world position.
         * @param startPosition World position (default: [0, 5, 20]).
         */
        explicit Camera(glm::vec3 startPosition = glm::vec3(0.0f, 5.0f, 20.0f));

        /**
         * @brief Processes user input and updates camera pose and mode transitions.
         * @param input Reference to the Input system.
         * @param deltaTime Frame delta time in seconds.
         */
        void ProcessInput(Input& input, float deltaTime);

        /**
         * @brief Sets the camera navigation mode.
         * @param newMode Target navigation mode.
         */
        void SetMode(Mode newMode);

        /**
         * @brief Gets current active camera navigation mode.
         * @return Active Mode.
         */
        Mode GetMode() const;

        /**
         * @brief Computes the 4x4 View matrix for coordinate transformation to camera space.
         * @return View matrix.
         */
        glm::mat4 GetViewMatrix() const;

        /**
         * @brief Computes perspective projection matrix for the current viewport aspect ratio.
         * @param aspectRatio Viewport width divided by height.
         * @return Perspective projection matrix.
         */
        glm::mat4 GetProjectionMatrix(float aspectRatio) const;

        glm::vec3 position;          ///< World-space camera location.
        float yawDegrees{ -90.0f };  ///< Horizontal rotation angle in degrees (-90 facing -Z).
        float pitchDegrees{ 0.0f };  ///< Vertical elevation angle in degrees [-89, +89].

        float fovDegrees{ 45.0f };   ///< Vertical field of view in degrees.
        float nearPlane{ 0.1f };     ///< Near clipping plane distance.
        float farPlane{ 500.0f };    ///< Far clipping plane distance.

        float moveSpeed{ 5.0f };          ///< Translation speed in world units per second.
        float mouseSensitivity{ 0.1f };   ///< Mouse look sensitivity in degrees per pixel.

        // Orbit mode properties:
        glm::vec3 orbitTarget{ 0.0f, 0.0f, 0.0f }; ///< Focal pivot point in world space.
        float orbitDistance{ 10.0f };              ///< Distance from orbitTarget to camera eye.
        float minOrbitDistance{ 1.0f };            ///< Minimum allowed orbit zoom distance.
        float maxOrbitDistance{ 100.0f };          ///< Maximum allowed orbit zoom distance.
        float zoomSpeed{ 1.0f };                   ///< Distance zoom delta per scroll notch.

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