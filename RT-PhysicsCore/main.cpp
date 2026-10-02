/**
 * @file main.cpp
 * @brief Demonstration application showcasing physical simulation and real-time rendering.
 */

#include <memory>
#include <GLFW/glfw3.h>

#include "RT-PhysicsCore/telemetry/TelemetryOverlay.h" 
#include "RT-PhysicsCore/telemetry/TelemetryManager.h"
#include "RT-PhysicsCore/telemetry/ProfilerWindow.h"

#include "RT-PhysicsCore/core/Engine.h"
#include "RT-PhysicsCore/utils/Log.h"

#include "RT-PhysicsCore/core/ecs/core/Scene.h"
#include "RT-PhysicsCore/core/ecs/core/System.h"
#include "RT-PhysicsCore/core/ecs/systems/TransformPropagationSystem.h"
#include "RT-PhysicsCore/core/ecs/components/TransformComponent.h"

#include "RT-PhysicsCore/physics/systems/PhysicsSystem.h"
#include "RT-PhysicsCore/physics/components/RigidBodyComponent.h"
#include "RT-PhysicsCore/physics/MassProperties.h"
#include "RT-PhysicsCore/physics/systems/CollisionSystem.h"
#include "RT-PhysicsCore/physics/components/ColliderComponent.h"
#include "RT-PhysicsCore/physics/systems/ResolutionSystem.h"

#include "RT-PhysicsCore/rendering/Renderer.h"
#include "RT-PhysicsCore/rendering/systems/RenderSystem.h"
#include "RT-PhysicsCore/rendering/components/MeshComponent.h"

int main()
{
    RT_PhysicsCore::TelemetryManager::Get().Initialize();
    RT_PhysicsCore::TelemetryManager::Get().SetMode(RT_PhysicsCore::TelemetryMode::Minimal);

    RT_PhysicsCore::TelemetryOverlay overlay;
    RT_PhysicsCore::ProfilerWindow profilerWindow;

    // Optional: persist this run's log to a file in addition to the console sink
    RT_PhysicsCore::Log::AddSink(
        std::make_unique<RT_PhysicsCore::FileLogSink>(RT_PhysicsCore::DefaultLogFilePath()));

    RT_LOG_INFO("Hello RT-PhysicsCore.");

    RT_PhysicsCore::Renderer renderer(1280, 720, "RT-PhysicsCore");
    if (!renderer.IsValid())
    {
        RT_LOG_FATAL("Renderer failed to initialize - see the errors above. Exiting.");
        return 1;
    }

    RT_PhysicsCore::Engine engine(60.0); // 60 Hz physics

    RT_PhysicsCore::Scene scene;

    // Create and register core systems
    scene.AddSystem(std::make_unique<RT_PhysicsCore::TransformPropagationSystem>(scene));
    scene.AddSystem(std::make_unique<RT_PhysicsCore::PhysicsSystem>(scene));

    auto collisionSystem = std::make_unique<RT_PhysicsCore::CollisionSystem>(scene);
    RT_PhysicsCore::CollisionSystem* collisionSystemPtr = collisionSystem.get();
    scene.AddSystem(std::move(collisionSystem));

    auto resolutionSystem = std::make_unique<RT_PhysicsCore::ResolutionSystem>(
        scene, *collisionSystemPtr, RT_PhysicsCore::ResolutionSystem::SolverMode::SequentialImpulses);
    RT_PhysicsCore::ResolutionSystem* resolutionSystemPtr = resolutionSystem.get();
    scene.AddSystem(std::move(resolutionSystem));
    (void)resolutionSystemPtr;

    scene.AddSystem(std::make_unique<RT_PhysicsCore::RenderSystem>(scene, renderer));

    // Dynamic falling sphere
    RT_PhysicsCore::Entity testSphere = scene.CreateEntity();
    constexpr float sphereMass = 1.0f;
    constexpr float sphereRadius = 0.5f;

    RT_PhysicsCore::TransformComponent t;
    t.position = { -2.0f, 60.0f, 0.0f };
    t.scale = { sphereRadius * 2.0f, sphereRadius * 2.0f, sphereRadius * 2.0f };

    RT_PhysicsCore::RigidBodyComponent rb = RT_PhysicsCore::MakeDynamicBody(
        sphereMass, RT_PhysicsCore::ComputeSphereInertia(sphereMass, sphereRadius));
    rb.velocity = { 0.5f, -9.81f, 0.0f };

    RT_PhysicsCore::MeshComponent mesh;
    mesh.shape = RT_PhysicsCore::PrimitiveShape::Sphere;
    mesh.color = { 0.9f, 0.3f, 0.2f };

    RT_PhysicsCore::ColliderComponent collider;
    collider.shape = RT_PhysicsCore::ColliderShape::Sphere;
    collider.size = { sphereRadius, 0.0f, 0.0f };

    scene.AddComponent(testSphere, t);
    scene.AddComponent(testSphere, rb);
    scene.AddComponent(testSphere, mesh);
    scene.AddComponent(testSphere, collider);

    // Static ground plane
    RT_PhysicsCore::Entity ground = scene.CreateEntity();
    RT_PhysicsCore::TransformComponent groundTransform;
    groundTransform.position = { 0.0f, -0.5f, 0.0f };
    groundTransform.scale = { 10.0f, 1.0f, 10.0f };

    RT_PhysicsCore::RigidBodyComponent groundBody = RT_PhysicsCore::MakeStaticBody();

    RT_PhysicsCore::MeshComponent groundMesh;
    groundMesh.shape = RT_PhysicsCore::PrimitiveShape::Cube;
    groundMesh.color = { 0.3f, 0.45f, 0.3f };

    // Ground collider uses Box shape (separate from visual mesh PrimitiveShape).
    RT_PhysicsCore::ColliderComponent groundCollider;
    groundCollider.shape = RT_PhysicsCore::ColliderShape::Box;
    constexpr float groundHalfThickness = 0.5f;
    groundCollider.size = { groundTransform.scale.x * 0.5f, groundHalfThickness, groundTransform.scale.z * 0.5f };
    groundCollider.offset = { 0.0f, groundHalfThickness, 0.0f };

    scene.AddComponent(ground, groundTransform);
    scene.AddComponent(ground, groundBody);
    scene.AddComponent(ground, groundMesh);
    scene.AddComponent(ground, groundCollider);

    engine.SetRenderCallback([&](double /*alpha*/)
        {
            renderer.BeginFrame();

            scene.RenderUpdateSystems();
            overlay.Render();

            renderer.EndFrame();

            // Render the profiler in its own window (only does work if open)
            profilerWindow.Render(renderer.GetWindow());
        });

    engine.SetUpdateCallback([&](double deltaTime)
        {
            scene.SetDeltaTime(deltaTime);
            scene.UpdateSystems();
            // F3 cycles: Off -> Minimal -> Full -> Off
            if (renderer.GetInput().WasKeyPressed(GLFW_KEY_F3))
            {
                auto& tm = RT_PhysicsCore::TelemetryManager::Get();
                RT_PhysicsCore::TelemetryMode current = tm.GetMode();
                if (current == RT_PhysicsCore::TelemetryMode::Off)
                {
                    tm.SetMode(RT_PhysicsCore::TelemetryMode::Minimal);
                }
                else if (current == RT_PhysicsCore::TelemetryMode::Minimal)
                {
                    tm.SetMode(RT_PhysicsCore::TelemetryMode::Full);
                    if (!profilerWindow.IsOpen())
                    {
                        profilerWindow.Open(renderer.GetWindow());
                    }
                }
                else
                {
                    profilerWindow.Close(renderer.GetWindow());
                    tm.SetMode(RT_PhysicsCore::TelemetryMode::Off);
                }
            }
            if (renderer.ShouldClose() || renderer.GetInput().WasKeyPressed(GLFW_KEY_ESCAPE))
            {
                profilerWindow.Close(renderer.GetWindow());
                engine.RequestExit();
            }
        });

    engine.SetFixedUpdateCallback([&](double fixedDeltaTime)
        {
            scene.SetFixedDeltaTime(fixedDeltaTime);
            scene.FixedUpdateSystems();
        });

    RT_LOG_INFO("START!");
    engine.Run();

    profilerWindow.Close(renderer.GetWindow());
    RT_PhysicsCore::TelemetryManager::Get().Shutdown();

    return 0;
}