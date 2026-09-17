#pragma once

#include <glm/glm.hpp>
#include <vector>

namespace RT_PhysicsCore
{
    // One endpoint of a debug line, with its own color so a single draw
    // call can render many differently-colored segments at once.
    struct DebugLineVertex
    {
        glm::vec3 position;
        glm::vec3 color;
    };

    // Separates lines by depth-test behavior so Renderer can draw them in
    // appropriate passes without needing per-vertex state flags.
    struct DebugDrawData
    {
        std::vector<DebugLineVertex> depthTestedLines;
        std::vector<DebugLineVertex> alwaysOnTopLines;
    };

    // Immediate-mode debug drawing: call Line()/Box()/Sphere() from
    // anywhere - PhysicsSystem, a future CollisionSystem, application code
    // - to queue shapes for this frame. Deliberately has no OpenGL
    // dependency: it just accumulates vertex data. Renderer is the only
    // thing that ever reads that data (via TakeLines()) and turns it into
    // actual draw calls, once per frame.
    //
    // Not thread-safe - matches the rest of the project, which is
    // single-threaded today. If systems ever run on worker threads, this
    // would need a mutex around the buffer the way Log's sinks do.
    class DebugDraw
    {
    public:
        static void Line(const glm::vec3& a, const glm::vec3& b, const glm::vec3& color, bool depthTest = true);

        // Axis-aligned box, drawn as 12 edges.
        static void Box(const glm::vec3& center, const glm::vec3& halfExtents, const glm::vec3& color, bool depthTest = true);

        // Wireframe sphere, drawn as three orthogonal great-circle rings.
        static void Sphere(const glm::vec3& center, float radius, const glm::vec3& color, int segments = 16, bool depthTest = true);

        // Renderer-only: returns and clears the accumulated buffer for
        // this frame. Calling this from anywhere else would steal the
        // lines out from under Renderer before it draws them.
        static DebugDrawData TakeLines();
    };
}
