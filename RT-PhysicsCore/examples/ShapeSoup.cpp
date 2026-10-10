/**
 * @file ShapeSoup.cpp
 * @brief Stress test in an enclosed arena.
 */

#include <memory>
#include <random>
#include <GLFW/glfw3.h>

#include "RT-PhysicsCore/telemetry/TelemetryOverlay.h"
#include "RT-PhysicsCore/telemetry/TelemetryManager.h"
#include "RT-PhysicsCore/telemetry/ProfilerWindow.h"

#include "RT-PhysicsCore/core/Engine.h"
#include "RT-PhysicsCore/core/ecs/core/Scene.h"
#include "RT-PhysicsCore/core/ecs/systems/TransformPropagationSystem.h"
#include "RT-PhysicsCore/core/ecs/components/TransformComponent.h"
#include "RT-PhysicsCore/physics/systems/PhysicsSystem.h"
#include "RT-PhysicsCore/physics/components/RigidBodyComponent.h"
#include "RT-PhysicsCore/physics/MassProperties.h"
#include "RT-PhysicsCore/physics/systems/CollisionSystem.h"
#include "RT-PhysicsCore/physics/components/ColliderComponent.h"
#include "RT-PhysicsCore/physics/systems/ResolutionSystem.h"
#include "RT-PhysicsCore/physics/components/PhysicsMaterialComponent.h"
#include "RT-PhysicsCore/rendering/Renderer.h"
#include "RT-PhysicsCore/rendering/systems/RenderSystem.h"
#include "RT-PhysicsCore/rendering/components/MeshComponent.h"

int main()
{
    using namespace RT_PhysicsCore;

    TelemetryManager::Get().Initialize();
    TelemetryManager::Get().SetMode(TelemetryMode::Minimal);
    TelemetryOverlay overlay;
    ProfilerWindow profilerWindow;

    Renderer renderer(1280, 720, "Example: Shape Soup");
    if (!renderer.IsValid()) return 1;

    Engine engine(60.0);
    Scene scene;

    scene.AddSystem(std::make_unique<TransformPropagationSystem>(scene));
    scene.AddSystem(std::make_unique<PhysicsSystem>(scene));
    auto colSys = std::make_unique<CollisionSystem>(scene);
    scene.AddSystem(std::make_unique<ResolutionSystem>(scene, *colSys, ResolutionSystem::SolverMode::SequentialImpulses));
    scene.AddSystem(std::move(colSys));
    scene.AddSystem(std::make_unique<RenderSystem>(scene, renderer));

    // Arena walls
    auto makeWall = [&](glm::vec3 pos, glm::vec3 extents) {
        Entity w = scene.CreateEntity();
        TransformComponent tc; tc.position = pos; tc.scale = extents * 2.0f;
        scene.AddComponent(w, tc);
        scene.AddComponent(w, MakeStaticBody());
        MeshComponent mc; mc.shape = PrimitiveShape::Cube; mc.color = { 0.2f, 0.2f, 0.25f };
        scene.AddComponent(w, mc);
        ColliderComponent cc; cc.shape = ColliderShape::Box; cc.size = extents;
        scene.AddComponent(w, cc);
        PhysicsMaterialComponent pm; pm.material = MaterialId::Rubber;
        scene.AddComponent(w, pm);
        };

    makeWall({ 0, -0.5f, 0 }, { 8.0f, 0.5f, 8.0f }); // floor
    makeWall({ 0, 4.0f, -8.5f }, { 8.0f, 4.0f, 0.5f }); // back
    makeWall({ 0, 4.0f, 8.5f }, { 8.0f, 4.0f, 0.5f }); // front
    makeWall({ -8.5f, 4.0f, 0 }, { 0.5f, 4.0f, 8.0f }); // left
    makeWall({ 8.5f, 4.0f, 0 }, { 0.5f, 4.0f, 8.0f }); // right

    std::mt19937 rng(1337);
    std::uniform_real_distribution<float> posDist(-6.0f, 6.0f);
    std::uniform_real_distribution<float> velDist(-5.0f, 5.0f);
    std::uniform_real_distribution<float> sizeDist(0.3f, 0.6f);
    std::uniform_real_distribution<float> colDist(0.3f, 1.0f);

    for (int i = 0; i < 150; ++i)
    {
        Entity e = scene.CreateEntity();
        TransformComponent tc;
        tc.position = { posDist(rng), 2.0f + i * 0.1f, posDist(rng) };
        float s = sizeDist(rng);
        tc.scale = { s * 2, s * 2, s * 2 };
        scene.AddComponent(e, tc);

        bool isBox = posDist(rng) > 0.0f;
        MeshComponent mc; mc.shape = isBox ? PrimitiveShape::Cube : PrimitiveShape::Sphere;
        mc.color = { colDist(rng), colDist(rng), colDist(rng) };
        scene.AddComponent(e, mc);

        float mass = s * s * s * 10.0f;
        RigidBodyComponent rb = MakeDynamicBody(mass, isBox ? ComputeBoxInertia(mass, glm::vec3(s)) : ComputeSphereInertia(mass, s));
        rb.velocity = { velDist(rng), velDist(rng), velDist(rng) };
        scene.AddComponent(e, rb);

        ColliderComponent cc; cc.shape = isBox ? ColliderShape::Box : ColliderShape::Sphere; cc.size = { s, s, s };
        scene.AddComponent(e, cc);

        PhysicsMaterialComponent pm; pm.material = MaterialId::HardRubber;
        scene.AddComponent(e, pm);
    }

    engine.SetUpdateCallback([&](double dt) {
        scene.SetDeltaTime(dt);
        scene.UpdateSystems();

        if (renderer.GetInput().WasKeyPressed(GLFW_KEY_F3))
        {
            auto& tm = TelemetryManager::Get();
            if (tm.GetMode() == TelemetryMode::Minimal) {
                tm.SetMode(TelemetryMode::Full);
                if (!profilerWindow.IsOpen()) profilerWindow.Open(renderer.GetWindow());
            }
            else {
                tm.SetMode(TelemetryMode::Minimal);
                profilerWindow.Close(renderer.GetWindow());
            }
        }
        if (renderer.ShouldClose()) engine.RequestExit();
        });

    engine.SetFixedUpdateCallback([&](double dt) {
        scene.SetFixedDeltaTime(dt);
        scene.FixedUpdateSystems();
        });

    engine.SetRenderCallback([&](double) {
        renderer.BeginFrame();
        scene.RenderUpdateSystems();
        overlay.Render();
        renderer.EndFrame();
        profilerWindow.Render(renderer.GetWindow());
        });

    engine.Run();
    profilerWindow.Close(renderer.GetWindow());
    TelemetryManager::Get().Shutdown();
    return 0;
}