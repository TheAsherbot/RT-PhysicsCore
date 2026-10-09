/**
 * @file DominoChain.cpp
 * @brief Box collision and rotation dynamics chain reaction.
 */

#include <memory>
#include <cmath>
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

#include <glm/gtc/quaternion.hpp>

int main()
{
    using namespace RT_PhysicsCore;

    Renderer renderer(1280, 720, "Example: Domino Chain");
    if (!renderer.IsValid()) return 1;

    Engine engine(60.0);
    Scene scene;

    scene.AddSystem(std::make_unique<TransformPropagationSystem>(scene));
    scene.AddSystem(std::make_unique<PhysicsSystem>(scene));
    auto colSys = std::make_unique<CollisionSystem>(scene);
    scene.AddSystem(std::make_unique<ResolutionSystem>(scene, *colSys, ResolutionSystem::SolverMode::SequentialImpulses));
    scene.AddSystem(std::move(colSys));
    scene.AddSystem(std::make_unique<RenderSystem>(scene, renderer));

    auto setup = [&]() {
        std::vector<Entity> toDestroy = scene.GetEntities();
        for (Entity e : toDestroy) scene.DestroyEntity(e);

        // Ground
        Entity ground = scene.CreateEntity();
        TransformComponent gtc; gtc.position = { 0.0f, -0.5f, 0.0f }; gtc.scale = { 30.0f, 1.0f, 30.0f };
        scene.AddComponent(ground, gtc);
        scene.AddComponent(ground, MakeStaticBody());
        MeshComponent gm; gm.shape = PrimitiveShape::Cube; gm.color = { 0.2f, 0.2f, 0.2f };
        scene.AddComponent(ground, gm);
        ColliderComponent gcc; gcc.shape = ColliderShape::Box; gcc.size = { 15.0f, 0.5f, 15.0f };
        scene.AddComponent(ground, gcc);

        // Dominos
        for (int i = 0; i < 40; ++i)
        {
            float t = i / 39.0f;
            float x = -6.0f + 12.0f * t;
            float z = 3.0f * std::sin(t * glm::pi<float>() * 2.0f);

            // Derivative for tangent vector
            float dx = 12.0f;
            float dz = 3.0f * glm::pi<float>() * 2.0f * std::cos(t * glm::pi<float>() * 2.0f);
            float angle = std::atan2(dz, dx);

            Entity e = scene.CreateEntity();
            TransformComponent tc;
            tc.position = { x, 1.0f, z };
            tc.scale = { 0.8f, 2.0f, 0.2f };
            // Orient so the thin axis (Z) faces along the path
            tc.rotation = glm::angleAxis(-angle + glm::pi<float>() * 0.5f, glm::vec3(0, 1, 0));
            scene.AddComponent(e, tc);

            scene.AddComponent(e, MakeDynamicBody(0.5f, ComputeBoxInertia(0.5f, { 0.4f, 1.0f, 0.1f })));
            ColliderComponent cc; cc.shape = ColliderShape::Box; cc.size = { 0.4f, 1.0f, 0.1f };
            scene.AddComponent(e, cc);

            MeshComponent mc; mc.shape = PrimitiveShape::Cube;
            mc.color = { 0.1f, 0.3f + 0.7f * t, 0.9f - 0.7f * t };
            scene.AddComponent(e, mc);

            PhysicsMaterialComponent pm; pm.material = MaterialId::Domino;
            scene.AddComponent(e, pm);
        }

        // Trigger ball
        Entity trigger = scene.CreateEntity();
        TransformComponent ttc; ttc.position = { -7.5f, 1.5f, 0.0f }; ttc.scale = { 0.6f, 0.6f, 0.6f };
        scene.AddComponent(trigger, ttc);
        RigidBodyComponent trb = MakeDynamicBody(2.0f, ComputeSphereInertia(2.0f, 0.3f));
        trb.velocity = { 3.0f, 0.0f, 0.0f };
        scene.AddComponent(trigger, trb);
        ColliderComponent tcc; tcc.shape = ColliderShape::Sphere; tcc.size = { 0.3f, 0, 0 };
        scene.AddComponent(trigger, tcc);
        MeshComponent tmc; tmc.shape = PrimitiveShape::Sphere; tmc.color = { 1.0f, 0.2f, 0.2f };
        scene.AddComponent(trigger, tmc);
        };
    setup();

    engine.SetUpdateCallback([&](double dt) {
        scene.SetDeltaTime(dt);
        scene.UpdateSystems();
        if (renderer.GetInput().WasKeyPressed(GLFW_KEY_R)) setup();
        if (renderer.ShouldClose()) engine.RequestExit();
        });

    engine.SetFixedUpdateCallback([&](double dt) {
        scene.SetFixedDeltaTime(dt);
        scene.FixedUpdateSystems();
        });

    engine.SetRenderCallback([&](double) {
        renderer.BeginFrame();
        scene.RenderUpdateSystems();
        renderer.EndFrame();
        });

    engine.Run();
    return 0;
}