/**
 * @file AABBBroadphaseTest.cpp
 * @brief Validates AABB computation for all collider shapes and overlap detection.
 *
 * Tests world-space AABB tightness under rotations and overlap pair detection
 * with automated assertions and interactive 3D wireframe debug visualization.
 */

#include <chrono>
#include <cmath>
#include <sstream>
#include <string>
#include <vector>

#include "RT-PhysicsCore/utils/Log.h"
#include "RT-PhysicsCore/utils/DebugDraw.h"
#include "RT-PhysicsCore/rendering/Renderer.h"
#include "RT-PhysicsCore/physics/collision/AABB.h"
#include "RT-PhysicsCore/physics/components/ColliderComponent.h"

#include <glm/gtc/quaternion.hpp>

namespace
{
    using namespace RT_PhysicsCore;

    // --- AABB Computation Tests ---

    struct AABBComputeCase
    {
        std::string name;
        ColliderComponent collider;
        glm::vec3 position;
        glm::quat rotation;
        AABB expected;
        float tolerance;
    };

    struct AABBOverlapCase
    {
        std::string name;
        AABB a;
        AABB b;
        bool expectOverlap;
    };

    bool AABBApproxEqual(const AABB& a, const AABB& b, float tol)
    {
        return glm::length(a.min - b.min) < tol && glm::length(a.max - b.max) < tol;
    }

    void DrawColliderWireframe(const ColliderComponent& cc, const glm::vec3& pos,
        const glm::quat& rot, const glm::vec3& color)
    {
        glm::mat3 rotMat = glm::mat3_cast(rot);
        switch (cc.shape)
        {
        case ColliderShape::Sphere:
            DebugDraw::Sphere(pos, cc.size.x, color);
            break;
        case ColliderShape::Box:
        {
            glm::vec3 c[8];
            int idx = 0;
            for (float sx : {-1.0f, 1.0f})
                for (float sy : {-1.0f, 1.0f})
                    for (float sz : {-1.0f, 1.0f})
                        c[idx++] = pos + rotMat[0] * sx * cc.size.x
                        + rotMat[1] * sy * cc.size.y + rotMat[2] * sz * cc.size.z;
            static const int edges[12][2] = {
                {0,1},{0,2},{0,4},{3,1},{3,2},{3,7},
                {5,1},{5,4},{5,7},{6,2},{6,4},{6,7}
            };
            for (auto& edge : edges)
                DebugDraw::Line(c[edge[0]], c[edge[1]], color);
            break;
        }
        case ColliderShape::Capsule:
        {
            glm::vec3 axis = rotMat[1] * cc.size.y;
            DebugDraw::Line(pos - axis, pos + axis, color);
            DebugDraw::Sphere(pos - axis, cc.size.x, color, 12);
            DebugDraw::Sphere(pos + axis, cc.size.x, color, 12);
            break;
        }
        }
    }

    void DrawAABBWireframe(const AABB& aabb, const glm::vec3& color)
    {
        glm::vec3 center = (aabb.min + aabb.max) * 0.5f;
        glm::vec3 halfExtents = (aabb.max - aabb.min) * 0.5f;
        DebugDraw::Box(center, halfExtents, color);
    }

    ColliderComponent MakeBoxCollider(const glm::vec3& halfExtents)
    {
        ColliderComponent cc;
        cc.shape = ColliderShape::Box;
        cc.size = halfExtents;
        return cc;
    }

    ColliderComponent MakeSphereCollider(float radius)
    {
        ColliderComponent cc;
        cc.shape = ColliderShape::Sphere;
        cc.size = { radius, 0.0f, 0.0f };
        return cc;
    }

    ColliderComponent MakeCapsuleCollider(float radius, float halfLength)
    {
        ColliderComponent cc;
        cc.shape = ColliderShape::Capsule;
        cc.size = { radius, halfLength, 0.0f };
        return cc;
    }

    std::vector<AABBComputeCase> BuildComputeCases()
    {
        std::vector<AABBComputeCase> cases;
        glm::quat identity(1.0f, 0.0f, 0.0f, 0.0f);
        glm::quat rotY45 = glm::angleAxis(glm::radians(45.0f), glm::vec3(0, 1, 0));
        glm::quat rotZ90 = glm::angleAxis(glm::radians(90.0f), glm::vec3(0, 0, 1));
        glm::quat rotArb = glm::angleAxis(glm::radians(30.0f), glm::normalize(glm::vec3(1, 1, 0)));
        float s2 = std::sqrt(2.0f);

        // Box axis-aligned
        cases.push_back({ "Box axis-aligned",
            MakeBoxCollider({1,1,1}), {0,0,0}, identity,
            {{-1,-1,-1},{1,1,1}}, 0.01f });

        // Box translated
        cases.push_back({ "Box translated",
            MakeBoxCollider({1,1,1}), {5,3,0}, identity,
            {{4,2,-1},{6,4,1}}, 0.01f });

        // Box rotated 45 deg about Y
        cases.push_back({ "Box 45 deg Y",
            MakeBoxCollider({1,0.5f,1}), {0,0,0}, rotY45,
            {{-s2, -0.5f, -s2}, {s2, 0.5f, s2}}, 0.05f });

        // Sphere at origin
        cases.push_back({ "Sphere at origin",
            MakeSphereCollider(1.5f), {0,0,0}, identity,
            {{-1.5f,-1.5f,-1.5f},{1.5f,1.5f,1.5f}}, 0.01f });

        // Sphere translated
        cases.push_back({ "Sphere translated",
            MakeSphereCollider(0.5f), {3,-2,7}, identity,
            {{2.5f,-2.5f,6.5f},{3.5f,-1.5f,7.5f}}, 0.01f });

        // Sphere rotated (should be invariant)
        cases.push_back({ "Sphere rotated (invariant)",
            MakeSphereCollider(1.0f), {0,0,0}, rotArb,
            {{-1,-1,-1},{1,1,1}}, 0.01f });

        // Capsule Y-up
        cases.push_back({ "Capsule Y-up",
            MakeCapsuleCollider(0.5f, 1.0f), {0,0,0}, identity,
            {{-0.5f,-1.5f,-0.5f},{0.5f,1.5f,0.5f}}, 0.01f });

        // Capsule rotated 90 to horizontal
        cases.push_back({ "Capsule horizontal (90 deg Z)",
            MakeCapsuleCollider(0.5f, 1.0f), {0,0,0}, rotZ90,
            {{-1.5f,-0.5f,-0.5f},{1.5f,0.5f,0.5f}}, 0.05f });

        // Capsule degenerate (halfLength=0 -> sphere)
        cases.push_back({ "Capsule degenerate (hl=0 -> sphere)",
            MakeCapsuleCollider(1.0f, 0.0f), {0,0,0}, identity,
            {{-1,-1,-1},{1,1,1}}, 0.01f });

        return cases;
    }

    std::vector<AABBOverlapCase> BuildOverlapCases()
    {
        std::vector<AABBOverlapCase> cases;

        cases.push_back({ "Clear overlap",
            {{-1,-1,-1},{1,1,1}}, {{0,0,0},{2,2,2}}, true });

        cases.push_back({ "Clearly separated",
            {{-1,-1,-1},{1,1,1}}, {{3,3,3},{5,5,5}}, false });

        cases.push_back({ "Touching face (zero gap)",
            {{0,0,0},{1,1,1}}, {{1,0,0},{2,1,1}}, true });

        cases.push_back({ "Separated on one axis",
            {{0,0,0},{1,1,1}}, {{0,0,2},{1,1,3}}, false });

        cases.push_back({ "A enclosing B",
            {{-2,-2,-2},{4,4,4}}, {{0,0,0},{1,1,1}}, true });

        cases.push_back({ "Touching at corner",
            {{0,0,0},{1,1,1}}, {{1,1,1},{2,2,2}}, true });

        return cases;
    }
}

int main()
{
    RT_LOG_INFO("AABB Broadphase Test harness starting.");

    auto computeCases = BuildComputeCases();
    auto overlapCases = BuildOverlapCases();

    // ── Phase 1: Automated assertions ──

    int passCount = 0;
    int totalCount = 0;

    for (const auto& tc : computeCases)
    {
        AABB result = ComputeWorldAABB(tc.collider, tc.position, tc.rotation);
        bool pass = AABBApproxEqual(result, tc.expected, tc.tolerance);
        if (pass)
        {
            ++passCount;
        }
        ++totalCount;

        std::ostringstream oss;
        oss << (pass ? "[PASS] " : "[FAIL] ") << tc.name
            << " min=[" << result.min.x << "," << result.min.y << "," << result.min.z << "]"
            << " max=[" << result.max.x << "," << result.max.y << "," << result.max.z << "]"
            << " expected min=[" << tc.expected.min.x << "," << tc.expected.min.y << "," << tc.expected.min.z << "]"
            << " max=[" << tc.expected.max.x << "," << tc.expected.max.y << "," << tc.expected.max.z << "]";
        RT_LOG_INFO(oss.str());
    }

    for (const auto& oc : overlapCases)
    {
        bool result = Overlaps(oc.a, oc.b);
        bool pass = (result == oc.expectOverlap);
        if (pass)
        {
            ++passCount;
        }
        ++totalCount;

        RT_LOG_INFO((pass ? "[PASS] " : "[FAIL] ") << oc.name
            << " result=" << result << " expected=" << oc.expectOverlap);
    }

    RT_LOG_INFO(passCount << "/" << totalCount << " automated checks passed.");

    // ── Phase 2: Interactive visualization ──

    Renderer renderer(1280, 720, "RT-PhysicsCore AABB Broadphase Test");
    if (!renderer.IsValid())
    {
        RT_LOG_FATAL("Renderer failed to initialize.");
        return 1;
    }

    // Combine all visual cases: compute cases first, then overlap cases
    size_t totalVisual = computeCases.size() + overlapCases.size();
    size_t current = 0;
    double elapsed = 0.0;
    constexpr double kSecondsPerTest = 3.0;
    auto lastTime = std::chrono::steady_clock::now();

    RT_LOG_INFO("Showing: " << computeCases[0].name);

    while (!renderer.ShouldClose())
    {
        auto now = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(now - lastTime).count();
        lastTime = now;
        elapsed += dt;

        if (elapsed >= kSecondsPerTest)
        {
            elapsed = 0.0;
            current = (current + 1) % totalVisual;
            if (current < computeCases.size())
            {
                RT_LOG_INFO("Showing: " << computeCases[current].name);
            }
            else
            {
                RT_LOG_INFO("Showing: " << overlapCases[current - computeCases.size()].name);
            }
        }

        if (current < computeCases.size())
        {
            // Draw compute case: collider shape + its AABB
            const auto& tc = computeCases[current];
            AABB result = ComputeWorldAABB(tc.collider, tc.position, tc.rotation);
            bool pass = AABBApproxEqual(result, tc.expected, tc.tolerance);

            DrawColliderWireframe(tc.collider, tc.position, tc.rotation, { 0.3f, 0.6f, 1.0f });
            DrawAABBWireframe(result, pass ? glm::vec3(0.2f, 0.9f, 0.3f) : glm::vec3(1.0f, 0.2f, 0.2f));
        }
        else
        {
            // Draw overlap case: two AABBs
            const auto& oc = overlapCases[current - computeCases.size()];
            bool result = Overlaps(oc.a, oc.b);
            glm::vec3 colorA = result ? glm::vec3(1.0f, 1.0f, 0.3f) : glm::vec3(0.5f, 0.5f, 0.5f);
            glm::vec3 colorB = colorA;

            DrawAABBWireframe(oc.a, colorA);
            DrawAABBWireframe(oc.b, colorB);

            // If overlapping, draw intersection box in yellow
            if (result)
            {
                AABB intersection;
                intersection.min = glm::max(oc.a.min, oc.b.min);
                intersection.max = glm::min(oc.a.max, oc.b.max);
                DrawAABBWireframe(intersection, { 1.0f, 0.8f, 0.0f });
            }
        }

        renderer.BeginFrame();
        renderer.FlushDebugDraw();
        renderer.EndFrame();
    }

    return 0;
}