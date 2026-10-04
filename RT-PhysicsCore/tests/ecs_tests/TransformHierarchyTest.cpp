/**
 * @file TransformHierarchyTest.cpp
 * @brief Validates the scene graph and TransformPropagationSystem.
 *
 * Ensures that child entities correctly inherit local translations and rotations
 * from their parents to form correct absolute WorldTransformComponents.
 */

#include <chrono>
#include <cmath>
#include <memory>
#include <sstream>
#include <string>

#include "RT-PhysicsCore/utils/Log.h"
#include "RT-PhysicsCore/utils/DebugDraw.h"
#include "RT-PhysicsCore/rendering/Renderer.h"

#include "RT-PhysicsCore/core/ecs/core/Scene.h"
#include "RT-PhysicsCore/core/ecs/systems/TransformPropagationSystem.h"
#include "RT-PhysicsCore/core/ecs/components/TransformComponent.h"

#include <glm/gtc/quaternion.hpp>

namespace
{
    using namespace RT_PhysicsCore;

    bool VecApproxEqual(const glm::vec3& a, const glm::vec3& b, float tol = 0.001f)
    {
        return glm::length(a - b) < tol;
    }
}

int main()
{
    RT_LOG_INFO("Transform Hierarchy Test harness starting.");

    int passCount = 0;
    int totalCount = 0;

    {
        ++totalCount;
        Scene scene;
        TransformPropagationSystem tps(scene);

        Entity parent = scene.CreateEntity();
        TransformComponent tp;
        tp.position = { 0.0f, 0.0f, 0.0f };
        // Rotate parent 90 degrees around Y axis
        tp.rotation = glm::angleAxis(glm::radians(90.0f), glm::vec3(0, 1, 0));
        scene.AddComponent(parent, tp);

        Entity child = scene.CreateEntity();
        TransformComponent tc;
        // Child is at local X=1.
        // Because parent is rotated 90deg Y, child world pos should be Z=-1.
        tc.position = { 1.0f, 0.0f, 0.0f };
        scene.AddComponent(child, tc);

        scene.SetParent(child, parent);

        tps.Update();

        auto* wt = scene.GetComponent<WorldTransformComponent>(child);
        glm::vec3 expectedWorldPos = { 0.0f, 0.0f, -1.0f };
        bool pass = wt && VecApproxEqual(wt->worldPosition, expectedWorldPos);
        if (pass) ++passCount;

        std::ostringstream oss;
        if (wt)
            oss << "Child WorldPos=[" << wt->worldPosition.x << "," << wt->worldPosition.y << "," << wt->worldPosition.z << "] Expected=[0,0,-1]";
        else
            oss << "No WorldTransformComponent found on child";
        RT_LOG_INFO((pass ? "[PASS] " : "[FAIL] ") << "Hierarchy Rotation Propagation - " << oss.str());
    }

    RT_LOG_INFO(passCount << "/" << totalCount << " automated checks passed.");

    // ── Phase 2: Interactive visualization ──

    Renderer renderer(1280, 720, "RT-PhysicsCore Hierarchy Test");
    if (!renderer.IsValid()) return 1;

    Scene scene;
    TransformPropagationSystem tps(scene);

    // Create a mini "solar system"
    Entity sun = scene.CreateEntity();
    TransformComponent tSun;
    tSun.position = { 0.0f, 0.0f, 0.0f };
    scene.AddComponent(sun, tSun);

    Entity earth = scene.CreateEntity();
    TransformComponent tEarth;
    tEarth.position = { 4.0f, 0.0f, 0.0f };
    scene.AddComponent(earth, tEarth);
    scene.SetParent(earth, sun);

    Entity moon = scene.CreateEntity();
    TransformComponent tMoon;
    tMoon.position = { 1.0f, 0.0f, 0.0f };
    scene.AddComponent(moon, tMoon);
    scene.SetParent(moon, earth);

    auto lastTime = std::chrono::steady_clock::now();
    float timeAccum = 0.0f;

    while (!renderer.ShouldClose())
    {
        auto now = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(now - lastTime).count();
        lastTime = now;
        timeAccum += static_cast<float>(dt);

        // Animate local rotations
        auto* sunT = scene.GetComponent<TransformComponent>(sun);
        sunT->rotation = glm::angleAxis(timeAccum * 1.0f, glm::vec3(0, 1, 0));

        auto* earthT = scene.GetComponent<TransformComponent>(earth);
        earthT->rotation = glm::angleAxis(timeAccum * 4.0f, glm::vec3(0, 1, 0));

        // Propagate down the tree
        tps.Update();

        // Draw the hierarchy
        auto* wSun = scene.GetComponent<WorldTransformComponent>(sun);
        auto* wEarth = scene.GetComponent<WorldTransformComponent>(earth);
        auto* wMoon = scene.GetComponent<WorldTransformComponent>(moon);

        DebugDraw::Sphere(wSun->worldPosition, 1.0f, { 1.0f, 0.8f, 0.0f });
        DebugDraw::Sphere(wEarth->worldPosition, 0.4f, { 0.0f, 0.5f, 1.0f });
        DebugDraw::Sphere(wMoon->worldPosition, 0.15f, { 0.7f, 0.7f, 0.7f });

        DebugDraw::Line(wSun->worldPosition, wEarth->worldPosition, { 0.3f, 0.3f, 0.3f });
        DebugDraw::Line(wEarth->worldPosition, wMoon->worldPosition, { 0.3f, 0.3f, 0.3f });

        renderer.BeginFrame();
        renderer.FlushDebugDraw();
        renderer.EndFrame();
    }

    return 0;
}