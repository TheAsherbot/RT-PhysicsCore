#pragma once

#include <memory>
#include "RT-PhysicsCore/rendering/Camera.h"
#include "RT-PhysicsCore/rendering/Input.h"

namespace RT_PhysicsCore
{
    struct MeshComponent;
    struct WorldTransformComponent;

    // Owns the window, GL context, compiled shaders, and primitive
    // geometry. The only class in the project that touches raw OpenGL -
    // everything else (RenderSystem, MeshComponent) stays graphics-API-
    // agnostic. Deliberately not an interface: there's exactly one backend
    // and no near-term plan for a second, so a virtual IRenderer would just
    // be indirection over a hypothetical.
    //
    // Pimpl'd on purpose, not just out of habit: it keeps <glad/gl.h> and
    // <GLFW/glfw3.h> confined to Renderer.cpp. Anything that only needs to
    // hold a Renderer& (like RenderSystem) doesn't drag platform/graphics
    // headers into its own compile.
    class Renderer
    {
    public:
        Renderer(int width, int height, const char* title);
        ~Renderer();

        Renderer(const Renderer&) = delete;
        Renderer& operator=(const Renderer&) = delete;

        // False if window/context/shader setup failed (already logged via
        // RT_LOG_FATAL when it happens) - check this before calling Run().
        bool IsValid() const;

        bool ShouldClose() const;

        // Polls window/input events, advances Input and the camera, clears
        // the screen.
        void BeginFrame();

        // Draws one entity's mesh using the current camera. No-op if
        // !IsValid().
        void DrawMesh(const MeshComponent& mesh, const WorldTransformComponent& worldTransform);

        // Drains DebugDraw's buffer for this frame and draws it as lines.
        void FlushDebugDraw();

        // Presents the frame (swaps buffers).
        void EndFrame();

        Camera& GetCamera();
        const Camera& GetCamera() const;

        // Keyboard/mouse state for this window - safe to query from
        // anywhere (main.cpp, other systems), not just Camera.
        Input& GetInput();
        const Input& GetInput() const;

    private:
        struct Impl;
        std::unique_ptr<Impl> impl;
    };
}
