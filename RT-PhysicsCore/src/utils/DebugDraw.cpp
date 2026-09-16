#include "RT-PhysicsCore/utils/DebugDraw.h"
#include <cmath>

namespace RT_PhysicsCore
{
    namespace
    {
        std::vector<DebugLineVertex>& Buffer()
        {
            static std::vector<DebugLineVertex> buffer;
            return buffer;
        }

        constexpr float kPi = 3.14159265358979323846f;
    }

    void DebugDraw::Line(const glm::vec3& a, const glm::vec3& b, const glm::vec3& color)
    {
        Buffer().push_back({a, color});
        Buffer().push_back({b, color});
    }

    void DebugDraw::Box(const glm::vec3& center, const glm::vec3& halfExtents, const glm::vec3& color)
    {
        glm::vec3 h = halfExtents;

        glm::vec3 c[8] = {
            center + glm::vec3(-h.x, -h.y, -h.z), // 0
            center + glm::vec3(-h.x, -h.y,  h.z), // 1
            center + glm::vec3(-h.x,  h.y, -h.z), // 2
            center + glm::vec3(-h.x,  h.y,  h.z), // 3
            center + glm::vec3( h.x, -h.y, -h.z), // 4
            center + glm::vec3( h.x, -h.y,  h.z), // 5
            center + glm::vec3( h.x,  h.y, -h.z), // 6
            center + glm::vec3( h.x,  h.y,  h.z), // 7
        };

        // bottom face (y = -h.y)
        Line(c[0], c[4], color);
        Line(c[4], c[5], color);
        Line(c[5], c[1], color);
        Line(c[1], c[0], color);
        // top face (y = +h.y)
        Line(c[2], c[6], color);
        Line(c[6], c[7], color);
        Line(c[7], c[3], color);
        Line(c[3], c[2], color);
        // vertical edges connecting the two faces
        Line(c[0], c[2], color);
        Line(c[4], c[6], color);
        Line(c[5], c[7], color);
        Line(c[1], c[3], color);
    }

    void DebugDraw::Sphere(const glm::vec3& center, float radius, const glm::vec3& color, int segments)
    {
        if (segments < 3)
            segments = 3;

        auto ring = [&](int axis) // 0 = XY plane, 1 = XZ plane, 2 = YZ plane
        {
            for (int i = 0; i < segments; ++i)
            {
                float a0 = (2.0f * kPi * i) / segments;
                float a1 = (2.0f * kPi * (i + 1)) / segments;

                glm::vec3 p0, p1;
                switch (axis)
                {
                    case 0:
                        p0 = { std::cos(a0) * radius, std::sin(a0) * radius, 0.0f };
                        p1 = { std::cos(a1) * radius, std::sin(a1) * radius, 0.0f };
                        break;
                    case 1:
                        p0 = { std::cos(a0) * radius, 0.0f, std::sin(a0) * radius };
                        p1 = { std::cos(a1) * radius, 0.0f, std::sin(a1) * radius };
                        break;
                    default:
                        p0 = { 0.0f, std::cos(a0) * radius, std::sin(a0) * radius };
                        p1 = { 0.0f, std::cos(a1) * radius, std::sin(a1) * radius };
                        break;
                }
                Line(center + p0, center + p1, color);
            }
        };

        ring(0);
        ring(1);
        ring(2);
    }

    std::vector<DebugLineVertex> DebugDraw::TakeLines()
    {
        std::vector<DebugLineVertex> result = std::move(Buffer());
        Buffer().clear(); // Buffer() is a fresh empty vector after the move, but explicit for clarity
        return result;
    }
}
