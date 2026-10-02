/**
 * @file Renderer.h
 * @brief OpenGL 3.3 Core rendering backend managing windowing, shaders, and draw passes.
 *
 * Encapsulates all direct graphics API calls and third-party OpenGL headers behind a Pimpl
 * boundary, decoupling the rest of the engine from GLAD and GLFW.
 */

#pragma once

#include <memory>
#include "RT-PhysicsCore/rendering/Camera.h"
#include "RT-PhysicsCore/rendering/Input.h"

struct GLFWwindow;

namespace RT_PhysicsCore
{

    struct MeshComponent;
    struct WorldTransformComponent;

    /**
     * @class Renderer
     * @brief Manages the GLFW window context, primitive meshes, shaders, and frame presentation.
     */
    class Renderer
    {
    public:
        /**
         * @brief Initializes the GLFW window, loads OpenGL 3.3 function pointers, and compiles shaders.
         * @param width Viewport width in pixels.
         * @param height Viewport height in pixels.
         * @param title Window caption string.
         */
        Renderer(int width, int height, const char* title);

        /**
         * @brief Destructor. Destroys GL resources, programs, and terminates GLFW.
         */
        ~Renderer();

        Renderer(const Renderer&) = delete;
        Renderer& operator=(const Renderer&) = delete;

        /**
         * @brief Checks if the window and OpenGL context initialized successfully.
         * @return True if operational; false if window creation or shader compile failed.
         */
        bool IsValid() const;

        /**
         * @brief Checks if the GLFW window has received an exit request.
         * @return True if the window should close.
         */
        bool ShouldClose() const;

        /**
         * @brief Polls events, updates camera/input, clears color and depth buffers.
         */
        void BeginFrame();

        /**
         * @brief Renders a single entity mesh transformed into world space.
         * @param mesh The visual shape and color descriptor.
         * @param worldTransform Global world position, rotation, and scale.
         */
        void DrawMesh(const MeshComponent& mesh, const WorldTransformComponent& worldTransform);

        /**
         * @brief Drains the DebugDraw line queue and submits line segments to the GPU.
         */
        void FlushDebugDraw();

        /**
         * @brief Swaps the OpenGL front and back buffers to present the frame.
         */
        void EndFrame();

        /**
         * @brief Provides mutable access to the active scene Camera.
         * @return Reference to Camera.
         */
        Camera& GetCamera();

        /**
         * @brief Provides read-only access to the active scene Camera.
         * @return Const reference to Camera.
         */
        const Camera& GetCamera() const;

        /**
         * @brief Provides access to the window Input subsystem.
         * @return Reference to Input.
         */
        Input& GetInput();

        /**
         * @brief Retrieves the active underlying GLFW window handle.
         * @return Pointer to the GLFWwindow instance.
         */
        GLFWwindow* GetWindow();

        /**
         * @brief Retrieves the active underlying GLFW window handle.
         * @return Const pointer to the GLFWwindow instance.
         */
        const GLFWwindow* GetWindow() const;

        /**
         * @brief Provides read-only access to the window Input subsystem.
         * @return Const reference to Input.
         */
        const Input& GetInput() const;

    private:
        struct Impl;
        std::unique_ptr<Impl> impl;
    };
}