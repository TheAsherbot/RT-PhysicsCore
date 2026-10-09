/**
 * @file NewtonsCradle.cpp
 * @brief Momentum and energy conservation in elastic 1D collisions.
 */

#include <memory>
#include <vector>
#include <GLFW/glfw3.h>

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
#include "RT-PhysicsCore/utils/DebugDraw.h"

int main()
{
    using namespace RT_PhysicsCore;

    Renderer renderer(1280, 720, "Example: Newton's Cradle");
    if (!renderer.IsValid()) return 1;

    Engine engine(60.0);
    Scene scene;

    scene.AddSystem(std::make_unique<TransformPropagationSystem>(scene));
    auto phys = std::make_unique<PhysicsSystem>(scene);
    phys->SetGravity({ 0.0f, 0.0f, 0.0f }); // Zero gravity
    scene.AddSystem(std::move(phys));

    auto colSys = std::make_unique<CollisionSystem>(scene);
    scene.AddSystem(std::make_unique<ResolutionSystem>(scene, *colSys, ResolutionSystem::SolverMode::SequentialImpulses));
    scene.AddSystem(std::move(colSys));
    scene.AddSystem(std::make_unique<RenderSystem>(scene, renderer));

    std::vector<Entity> balls;
    auto setup = [&](int pullCount) {
        for (Entity e : balls) scene.DestroyEntity(e);
        balls.clear();

        for (int i = 0; i < 5; ++i)
        {
            Entity e = scene.CreateEntity();
            TransformComponent tc;

            // If it's one of the pulled balls, move it back on X and give it velocity
            if (i < pullCount)
            {
                tc.position = { -3.0f - (pullCount - 1 - i) * 1.0f, 0.0f, 0.0f };
            }
            else
            {
                tc.position = { i * 1.0f, 0.0f, 0.0f };
            }
            tc.scale = { 1.0f, 1.0f, 1.0f };
            scene.AddComponent(e, tc);

            RigidBodyComponent rb = MakeDynamicBody(1.0f, ComputeSphereInertia(1.0f, 0.5f));
            if (i < pullCount) rb.velocity = { 5.0f, 0.0f, 0.0f };
            scene.AddComponent(e, rb);

            ColliderComponent cc; cc.shape = ColliderShape::Sphere; cc.size = { 0.5f, 0, 0 };
            scene.AddComponent(e, cc);

            MeshComponent mc; mc.shape = PrimitiveShape::Sphere; mc.color = { 0.7f, 0.8f, 0.9f };
            scene.AddComponent(e, mc);

            PhysicsMaterialComponent pm; pm.material = MaterialId::BouncyIce; // e=1.0, mu=0
            scene.AddComponent(e, pm);

            balls.push_back(e);
        }
        };
    setup(1);

    engine.SetUpdateCallback([&](double dt) {
        scene.SetDeltaTime(dt);
        scene.UpdateSystems();

        auto& input = renderer.GetInput();
        if (input.WasKeyPressed(GLFW_KEY_1)) setup(1);
        if (input.WasKeyPressed(GLFW_KEY_2)) setup(2);
        if (input.WasKeyPressed(GLFW_KEY_3)) setup(3);

        if (renderer.ShouldClose()) engine.RequestExit();
        });

    engine.SetFixedUpdateCallback([&](double dt) {
        scene.SetFixedDeltaTime(dt);
        scene.FixedUpdateSystems();

        // Constrain to 1D motion to emulate the suspension cables
        for (Entity e : balls)
        {
            auto* rb = scene.GetComponent<RigidBodyComponent>(e);
            auto* tc = scene.GetComponent<TransformComponent>(e);
            rb->velocity.y = 0.0f;
            rb->velocity.z = 0.0f;
            rb->angularVelocity = { 0.0f, 0.0f, 0.0f };
            tc->position.y = 0.0f;
            tc->position.z = 0.0f;
        }
        });

    engine.SetRenderCallback([&](double) {
        renderer.BeginFrame();
        scene.RenderUpdateSystems();

        DebugDraw::Line({ -10.0f, 0.0f, 0.0f }, { 10.0f, 0.0f, 0.0f }, { 0.3f, 0.3f, 0.3f });

        for (Entity e : balls)
        {
            auto* tc = scene.GetComponent<TransformComponent>(e);
            auto* rb = scene.GetComponent<RigidBodyComponent>(e);
            if (glm::length(rb->velocity) > 0.1f)
            {
                DebugDraw::Line(tc->position + glm::vec3(0, 0.6f, 0),
                    tc->position + glm::vec3(0, 0.6f, 0) + rb->velocity * 0.2f,
                    { 0.0f, 1.0f, 1.0f }, false);
            }
        }

        renderer.FlushDebugDraw();
        renderer.EndFrame();
        });

    engine.Run();
    return 0;
}