/**
 * @file CollisionTest.cpp
 * @brief Standalone test harness exercising pairwise narrow-phase collision tests.
 *
 * Runs automated pass/fail verification of intersection algorithms on startup,
 * then renders an interactive cycling 3D visual debug scene with contact manifolds.
 */

#include <chrono>
#include <cmath>
#include <string>
#include <vector>

#include "RT-PhysicsCore/utils/Log.h"
#include "RT-PhysicsCore/utils/DebugDraw.h"
#include "RT-PhysicsCore/rendering/Renderer.h"
#include "RT-PhysicsCore/physics/components/ColliderComponent.h"
#include "RT-PhysicsCore/physics/collision/NarrowPhase.h"

#include <glm/gtc/quaternion.hpp>

namespace
{
    struct TestCase
    {
        std::string name;
        RT_PhysicsCore::ColliderPose a;
        RT_PhysicsCore::ColliderPose b;
        bool expectHit;
        int expectPointCount = -1;
    };

    RT_PhysicsCore::ColliderPose MakePose(RT_PhysicsCore::ColliderShape shape, const glm::vec3& size,
        const glm::vec3& pos, glm::quat rot = glm::quat(1.0f, 0.0f, 0.0f, 0.0f))
    {
        RT_PhysicsCore::ColliderPose pose;
        pose.shape = shape;
        pose.size = size;
        pose.position = pos;
        pose.rotation = glm::mat3_cast(rot);
        return pose;
    }

    void DrawCollider(const RT_PhysicsCore::ColliderPose& pose, const glm::vec3& color)
    {
        using RT_PhysicsCore::ColliderShape;
        using RT_PhysicsCore::DebugDraw;

        switch (pose.shape)
        {
        case ColliderShape::Sphere:
        {
            DebugDraw::Sphere(pose.position, pose.size.x, color);
            break;
        }

        case ColliderShape::Box:
        {
            glm::vec3 h = pose.size;
            glm::vec3 c[8];
            int idx = 0;
            for (float sx : { -1.0f, 1.0f })
            {
                for (float sy : { -1.0f, 1.0f })
                {
                    for (float sz : { -1.0f, 1.0f })
                    {
                        c[idx++] = pose.position + pose.rotation[0] * sx * h.x
                            + pose.rotation[1] * sy * h.y
                            + pose.rotation[2] * sz * h.z;
                    }
                }
            }

            static const int edges[12][2] = {
                {0,1},{0,2},{0,4},{3,1},{3,2},{3,7},
                {5,1},{5,4},{5,7},{6,2},{6,4},{6,7}
            };
            for (auto& edge : edges)
            {
                DebugDraw::Line(c[edge[0]], c[edge[1]], color);
            }
            break;
        }

        case ColliderShape::Capsule:
        {
            glm::vec3 axis = pose.rotation[1] * pose.size.y;
            glm::vec3 capA = pose.position - axis;
            glm::vec3 capB = pose.position + axis;
            DebugDraw::Line(capA, capB, color);
            DebugDraw::Sphere(capA, pose.size.x, color, 12);
            DebugDraw::Sphere(capB, pose.size.x, color, 12);
            break;
        }
        }
    }

    std::vector<TestCase> BuildTests()
    {
        using RT_PhysicsCore::ColliderShape;

        glm::quat rotZ90 = glm::angleAxis(glm::radians(90.0f), glm::vec3(0.0f, 0.0f, 1.0f));
        glm::quat rotY45 = glm::angleAxis(glm::radians(45.0f), glm::vec3(0.0f, 1.0f, 0.0f));
        glm::quat rotX45 = glm::angleAxis(glm::radians(45.0f), glm::vec3(1.0f, 0.0f, 0.0f));

        std::vector<TestCase> tests;

        tests.push_back({ "Sphere-Sphere overlap",
            MakePose(ColliderShape::Sphere, {1.0f,0,0}, {0,0,0}),
            MakePose(ColliderShape::Sphere, {1.0f,0,0}, {1.5f,0,0}),
            true, 1 });

        tests.push_back({ "Sphere-Sphere separated",
            MakePose(ColliderShape::Sphere, {1.0f,0,0}, {0,0,0}),
            MakePose(ColliderShape::Sphere, {1.0f,0,0}, {5.0f,0,0}),
            false });

        tests.push_back({ "Sphere-Box touching face",
            MakePose(ColliderShape::Box, {1,1,1}, {0,0,0}),
            MakePose(ColliderShape::Sphere, {0.5f,0,0}, {1.3f,0,0}),
            true, 1 });

        tests.push_back({ "Sphere center inside Box",
            MakePose(ColliderShape::Box, {2,2,2}, {0,0,0}),
            MakePose(ColliderShape::Sphere, {0.5f,0,0}, {0.5f,0,0}),
            true, 1 });

        tests.push_back({ "Sphere-Capsule",
            MakePose(ColliderShape::Capsule, {0.5f,1.0f,0}, {0,0,0}),
            MakePose(ColliderShape::Sphere, {0.5f,0,0}, {0.5f,0,0}),
            true, 1 });

        tests.push_back({ "Capsule-Capsule crossing",
            MakePose(ColliderShape::Capsule, {0.4f,1.0f,0}, {0,0,0}, rotZ90),
            MakePose(ColliderShape::Capsule, {0.4f,1.0f,0}, {0,0,0}),
            true, 1 });

        tests.push_back({ "Capsule-Capsule parallel resting",
            MakePose(ColliderShape::Capsule, {0.5f,1.5f,0}, {0,0,0}),
            MakePose(ColliderShape::Capsule, {0.5f,1.5f,0}, {0.7f,0,0}),
            true, 2 });

        tests.push_back({ "Box-Capsule resting on face",
            MakePose(ColliderShape::Box, {2,0.5f,2}, {0,0,0}),
            MakePose(ColliderShape::Capsule, {0.4f,1.5f,0}, {0,0.75f,0}, rotZ90),
            true, 2 });

        tests.push_back({ "Box-Box flat stack",
            MakePose(ColliderShape::Box, {2,1,2}, {0,0,0}),
            MakePose(ColliderShape::Box, {1,1,1}, {0,1.8f,0}),
            true, 4 });

        tests.push_back({ "Box-Box rotated, touching",
            MakePose(ColliderShape::Box, {1,1,1}, {0,0,0}, rotY45),
            MakePose(ColliderShape::Box, {1,1,1}, {1.6f,1.6f,0}, rotX45),
            true });

        tests.push_back({ "Box-Box separated",
            MakePose(ColliderShape::Box, {1,1,1}, {0,0,0}),
            MakePose(ColliderShape::Box, {1,1,1}, {10,0,0}),
            false });

        return tests;
    }
}

int main()
{
    RT_LOG_INFO("Collision narrow-phase test harness starting.");

    RT_PhysicsCore::Renderer renderer(1280, 720, "RT-PhysicsCore Collision Tests");
    if (!renderer.IsValid())
    {
        RT_LOG_FATAL("Renderer failed to initialize - see the errors above. Exiting.");
        return 1;
    }

    std::vector<TestCase> tests = BuildTests();

    int passCount = 0;
    for (TestCase& t : tests)
    {
        RT_PhysicsCore::Contact contact;
        bool hit = RT_PhysicsCore::TestCollision(t.a, t.b, contact);

        bool pass = (hit == t.expectHit)
            && (!t.expectHit || t.expectPointCount < 0 || contact.pointCount == t.expectPointCount);
        if (pass)
        {
            ++passCount;
        }

        RT_LOG_INFO((pass ? "[PASS] " : "[FAIL] ") << t.name
            << " - hit=" << hit << " points=" << contact.pointCount
            << " (expected hit=" << t.expectHit
            << (t.expectPointCount >= 0 ? (", points=" + std::to_string(t.expectPointCount)) : std::string())
            << ")");
    }
    RT_LOG_INFO(passCount << "/" << tests.size() << " automated checks passed.");

    size_t current = 0;
    double elapsed = 0.0;
    constexpr double kSecondsPerTest = 3.0;
    auto lastTime = std::chrono::steady_clock::now();

    RT_LOG_INFO("Showing: " << tests[current].name);

    while (!renderer.ShouldClose())
    {
        auto now = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(now - lastTime).count();
        lastTime = now;
        elapsed += dt;

        if (elapsed >= kSecondsPerTest)
        {
            elapsed = 0.0;
            current = (current + 1) % tests.size();
            RT_LOG_INFO("Showing: " << tests[current].name);
        }

        const TestCase& t = tests[current];
        RT_PhysicsCore::Contact contact;
        bool hit = RT_PhysicsCore::TestCollision(t.a, t.b, contact);

        DrawCollider(t.a, glm::vec3(0.3f, 0.6f, 1.0f));
        DrawCollider(t.b, glm::vec3(1.0f, 0.6f, 0.3f));

        if (hit)
        {
            for (int i = 0; i < contact.pointCount; ++i)
            {
                RT_PhysicsCore::DebugDraw::Sphere(contact.points[i], 0.06f, glm::vec3(1.0f, 1.0f, 0.0f), 8, false);
                RT_PhysicsCore::DebugDraw::Line(contact.points[i], contact.points[i] + contact.normal * 0.6f,
                    glm::vec3(1.0f, 0.0f, 0.0f), false);
            }
        }

        renderer.BeginFrame();
        renderer.FlushDebugDraw();
        renderer.EndFrame();
    }

    return 0;
}