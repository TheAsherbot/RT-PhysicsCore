/**
 * @file FallingBoxes.cpp
 * @brief Continuous rigid body stress test.
 */

#include <memory>
#include <random>
#include <GLFW/glfw3.h>

#include "RT-PhysicsCore/telemetry/TelemetryOverlay.h"
#include "RT-PhysicsCore/telemetry/TelemetryManager.h"
#include "RT-PhysicsCore/telemetry/ProfilerWindow.h"

#include "RT-PhysicsCore/core/Engine.h"
#include "RT-PhysicsCore/utils/Log.h"
#include "RT-PhysicsCore/utils/DebugDraw.h"

#include "RT-PhysicsCore/core/ecs/core/Scene.h"
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

#include <glm/gtc/quaternion.hpp>
#include <imgui/imgui.h>

int main()
{
    using namespace RT_PhysicsCore;

    TelemetryManager::Get().Initialize();
    TelemetryManager::Get().SetMode(TelemetryMode::Minimal);
    TelemetryOverlay overlay;
    ProfilerWindow profilerWindow;

    Renderer renderer(1280, 720, "Example: Falling Boxes");
    if (!renderer.IsValid()) return 1;

    Engine engine(60.0);
    Scene scene;

    scene.AddSystem(std::make_unique<TransformPropagationSystem>(scene));
    scene.AddSystem(std::make_unique<PhysicsSystem>(scene));
    auto colSys = std::make_unique<CollisionSystem>(scene);
    CollisionSystem* colSysPtr = colSys.get();
    scene.AddSystem(std::move(colSys));
    scene.AddSystem(std::make_unique<ResolutionSystem>(scene, *colSysPtr, ResolutionSystem::SolverMode::SequentialImpulses));
    scene.AddSystem(std::make_unique<RenderSystem>(scene, renderer));

    // Ground
    Entity ground = scene.CreateEntity();
    TransformComponent gtc;
    gtc.position = { 0.0f, -0.5f, 0.0f };
    gtc.scale = { 20.0f, 1.0f, 20.0f };
    scene.AddComponent(ground, gtc);
    scene.AddComponent(ground, MakeStaticBody());
    MeshComponent gm;
    gm.shape = PrimitiveShape::Cube;
    gm.color = { 0.3f, 0.4f, 0.3f };
    scene.AddComponent(ground, gm);
    ColliderComponent gcc;
    gcc.shape = ColliderShape::Box;
    gcc.size = { 10.0f, 0.5f, 10.0f };
    scene.AddComponent(ground, gcc);

    std::mt19937 rng(42);
    std::uniform_real_distribution<float> posDist(-4.0f, 4.0f);
    std::uniform_real_distribution<float> sizeDist(0.4f, 0.8f);
    std::uniform_real_distribution<float> colorDist(0.4f, 1.0f);
    std::uniform_real_distribution<float> rotDist(-3.14f, 3.14f);

    int objCount = 0;
    double spawnTimer = 0.0;
    bool drawContacts = false;
    bool drawNormals = false;

    engine.SetUpdateCallback([&](double dt) {
        scene.SetDeltaTime(dt);
        scene.UpdateSystems();

        if (renderer.GetInput().WasKeyPressed(GLFW_KEY_F1)) drawContacts = !drawContacts;
        if (renderer.GetInput().WasKeyPressed(GLFW_KEY_F2)) drawNormals = !drawNormals;

        if (renderer.ShouldClose() || renderer.GetInput().WasKeyPressed(GLFW_KEY_ESCAPE))
        {
            engine.RequestExit();
        }
        });

    engine.SetFixedUpdateCallback([&](double dt) {
        scene.SetFixedDeltaTime(dt);
        scene.FixedUpdateSystems();

        spawnTimer += dt;
        if (spawnTimer > 0.2 && objCount < 200)
        {
            spawnTimer = 0.0;
            Entity e = scene.CreateEntity();
            float size = sizeDist(rng);
            bool isBox = posDist(rng) > 0.0f;

            TransformComponent tc;
            tc.position = { posDist(rng), 12.0f, posDist(rng) };
            tc.scale = { size * 2, size * 2, size * 2 };
            tc.rotation = glm::quat(glm::vec3(rotDist(rng), rotDist(rng), rotDist(rng)));
            scene.AddComponent(e, tc);

            MeshComponent mc;
            mc.shape = isBox ? PrimitiveShape::Cube : PrimitiveShape::Sphere;
            mc.color = { colorDist(rng), colorDist(rng), colorDist(rng) };
            scene.AddComponent(e, mc);

            float mass = size * size * size * 10.0f;
            scene.AddComponent(e, MakeDynamicBody(mass,
                isBox ? ComputeBoxInertia(mass, glm::vec3(size)) : ComputeSphereInertia(mass, size)));

            ColliderComponent cc;
            cc.shape = isBox ? ColliderShape::Box : ColliderShape::Sphere;
            cc.size = { size, size, size };
            scene.AddComponent(e, cc);

            ++objCount;
        }
        });

    engine.SetRenderCallback([&](double /*alpha*/) {
        renderer.BeginFrame();
        scene.RenderUpdateSystems();

        if (drawContacts || drawNormals)
        {
            for (const auto& c : colSysPtr->GetContacts())
            {
                for (int i = 0; i < c.pointCount; ++i)
                {
                    if (drawContacts) DebugDraw::Sphere(c.points[i], 0.05f, { 1.0f, 1.0f, 0.0f });
                    if (drawNormals) DebugDraw::Line(c.points[i], c.points[i] + c.normal * 0.4f, { 1.0f, 0.0f, 0.0f });
                }
            }
        }

        renderer.FlushDebugDraw();
        overlay.Render();
        renderer.EndFrame();
        });

    engine.Run();
    TelemetryManager::Get().Shutdown();
    return 0;
}