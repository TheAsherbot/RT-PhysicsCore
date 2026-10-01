/**
 * @file DebugDraw.h
 * @brief Immediate-mode 3D wireframe debug visualization queue.
 *
 * Provides graphics-agnostic geometry collection for lines, wireframe boxes,
 * and spheres, drained each frame by the active Renderer.
 */

#pragma once

#include <glm/glm.hpp>
#include <vector>

namespace RT_PhysicsCore
{
    /**
     * @struct DebugLineVertex
     * @brief A single colored endpoint of a debug line segment.
     */
    struct DebugLineVertex
    {
        glm::vec3 position; ///< World-space vertex position.
        glm::vec3 color;    ///< Linear RGB vertex color.
    };

    /**
     * @struct DebugDrawData
     * @brief Drained debug line buffers separated by depth-testing behavior.
     */
    struct DebugDrawData
    {
        std::vector<DebugLineVertex> depthTestedLines; ///< Lines occluded by 3D scene geometry.
        std::vector<DebugLineVertex> alwaysOnTopLines;  ///< Overlay lines rendered on top of everything.
    };

    /**
     * @class DebugDraw
     * @brief Static queue for submitting immediate-mode diagnostic geometry.
     */
    class DebugDraw
    {
    public:
        /**
         * @brief Queues a 3D line segment between two points.
         * @param a World-space starting coordinate.
         * @param b World-space ending coordinate.
         * @param color Line RGB color.
         * @param depthTest True if occluded by depth buffer; false for overlay rendering.
         */
        static void Line(const glm::vec3& a, const glm::vec3& b, const glm::vec3& color, bool depthTest = true);

        /**
         * @brief Queues a wireframe axis-aligned bounding box (12 edges).
         * @param center World-space center point.
         * @param halfExtents Half-dimensions along X, Y, Z.
         * @param color Wireframe RGB color.
         * @param depthTest True if occluded by depth buffer; false for overlay rendering.
         */
        static void Box(const glm::vec3& center, const glm::vec3& halfExtents, const glm::vec3& color, bool depthTest = true);

        /**
         * @brief Queues a wireframe sphere drawn as three orthogonal great-circle rings.
         * @param center World-space center point.
         * @param radius Sphere radius.
         * @param color Wireframe RGB color.
         * @param segments Vertex resolution per circle ring (default: 16).
         * @param depthTest True if occluded by depth buffer; false for overlay rendering.
         */
        static void Sphere(const glm::vec3& center, float radius, const glm::vec3& color, int segments = 16, bool depthTest = true);

        /**
         * @brief Drains and clears all queued debug lines for GPU submission.
         * @note Dedicated to Renderer only; calling elsewhere steals line data.
         * @return Populated DebugDrawData structure containing accumulated segments.
         */
        static DebugDrawData TakeLines();
    };
}