/**
 * @file WreckingBall.cpp
 * @brief Demonstrates combining ECS transform hierarchies with physical simulation.
 */

#include <memory>
#include <vector>
#include <GLFW/glfw3.h>

#include "RT-PhysicsCore/core/Engine.h"
#include "RT-PhysicsCore/core/ecs/core/Scene.h"
#include "RT-PhysicsCore/core/ecs/systems/TransformPropagationSystem.h"
#include "RT-PhysicsCore/core/ecs/components/TransformComponent.h"
#include "RT-PhysicsCore/core/ecs/components/HierarchyComponent.h"
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

int main()
{
    using namespace RT_PhysicsCore;

    Renderer renderer(1280, 720, "Example: Wrecking Ball");
    if (!renderer.IsValid()) return 1;

    Engine engine(60.0);
    Scene scene;

    auto tps = std::make_unique<TransformPropagationSystem>(scene);
    TransformPropagationSystem* tpsPtr = tps.get();
    scene.AddSystem(std::move(tps));
    scene.AddSystem(std::make_unique<PhysicsSystem>(scene));
    auto colSys = std::make_unique<CollisionSystem>(scene);
    scene.AddSystem(std::make_unique<ResolutionSystem>(scene, *colSys, ResolutionSystem::SolverMode::SequentialImpulses));
    scene.AddSystem(std::move(colSys));
    scene.AddSystem(std::make_unique<RenderSystem>(scene, renderer));

    // Ground
    Entity ground = scene.CreateEntity();
    TransformComponent gtc; gtc.position = { 0.0f, -0.5f, 0.0f }; gtc.scale = { 40.0f, 1.0f, 40.0f };
    scene.AddComponent(ground, gtc);
    scene.AddComponent(ground, MakeStaticBody());
    MeshComponent gm; gm.shape = PrimitiveShape::Cube; gm.color = { 0.2f, 0.3f, 0.2f };
    scene.AddComponent(ground, gm);
    ColliderComponent gcc; gcc.shape = ColliderShape::Box; gcc.size = { 20.0f, 0.5f, 20.0f };
    scene.AddComponent(ground, gcc);

    // --- The Kinematic Crane Hierarchy (Visuals Only) ---
    Entity craneBase = scene.CreateEntity();
    TransformComponent tcBase; tcBase.position = { 0.0f, 3.0f, 0.0f }; tcBase.scale = { 2.0f, 15.0f, 2.0f };
    scene.AddComponent(craneBase, tcBase);
    MeshComponent mcBase; mcBase.shape = PrimitiveShape::Cube; mcBase.color = { 0.3f, 0.3f, 0.3f };
    scene.AddComponent(craneBase, mcBase);

    Entity craneArm = scene.CreateEntity();
    TransformComponent tcArm; tcArm.position = { 4.0f / tcBase.scale.x, (15.0f + 0.6) / (tcBase.scale.y) / 2, 0.0f }; tcArm.scale = { 8.0f / tcBase.scale.x, 0.6f / tcBase.scale.y, 0.6f / tcBase.scale.z };
    scene.AddComponent(craneArm, tcArm);
    MeshComponent mcArm; mcArm.shape = PrimitiveShape::Cube; mcArm.color = { 0.8f, 0.8f, 0.2f };
    scene.AddComponent(craneArm, mcArm);
    scene.SetParent(craneArm, craneBase);

    Entity cable = scene.CreateEntity();
    TransformComponent tcCable; tcCable.position = { 4.1f / (tcArm.scale.x) / (tcBase.scale.x), -4.0f / (tcArm.scale.y) / (tcBase.scale.y), 0.0f}; 
    tcCable.scale = {0.1f / (tcArm.scale.x) / (tcBase.scale.x), 8.0f / (tcArm.scale.y) / (tcBase.scale.y), 0.1f / (tcArm.scale.z) / (tcBase.scale.z) };
    scene.AddComponent(cable, tcCable);
    MeshComponent mcCable; mcCable.shape = PrimitiveShape::Cube; mcCable.color = { 0.1f, 0.1f, 0.1f };
    scene.AddComponent(cable, mcCable);
    scene.SetParent(cable, craneArm);

    // --- The Physical Wrecking Ball ---
    // Root entity so Physics/Collision evaluate it correctly in world space
    Entity wreckingBall = scene.CreateEntity();
    TransformComponent tcBall; tcBall.scale = { 1.6f, 1.6f, 1.6f };
    scene.AddComponent(wreckingBall, tcBall);
    MeshComponent mcBall; mcBall.shape = PrimitiveShape::Sphere; mcBall.color = { 0.8f, 0.2f, 0.2f };
    scene.AddComponent(wreckingBall, mcBall);
    // Infinite mass kinematic body so it plows through the wall!
    scene.AddComponent(wreckingBall, MakeStaticBody());
    ColliderComponent ccBall; ccBall.shape = ColliderShape::Sphere; ccBall.size = { 0.8f, 0, 0 };
    scene.AddComponent(wreckingBall, ccBall);
    tcBall.position = { 0.0f, 0.0f, 0.0f};

    // --- The Wall of Boxes ---
    for (int y = 0; y < 5; ++y)
    {
        for (int z = 0; z < 6; ++z)
        {
            Entity b = scene.CreateEntity();
            TransformComponent tc; tc.position = {0, 0.5f + y * 1.0f,  -7.5 - 2.5f + z * 1.0f }; tc.scale = { 1.0f, 1.0f, 1.0f };
            scene.AddComponent(b, tc);
            scene.AddComponent(b, MakeDynamicBody(1.0f, ComputeBoxInertia(1.0f, { 0.5f, 0.5f, 0.5f })));
            ColliderComponent cc; cc.shape = ColliderShape::Box; cc.size = { 0.5f, 0.5f, 0.5f };
            scene.AddComponent(b, cc);
            MeshComponent mc; mc.shape = PrimitiveShape::Cube; mc.color = { 0.8f, 0.4f + y * 0.1f, 0.2f };
            scene.AddComponent(b, mc);
        }
    }

    engine.SetUpdateCallback([&](double dt) {
        scene.SetDeltaTime(dt);
        scene.UpdateSystems();
        if (renderer.ShouldClose()) engine.RequestExit();
        });

    float rotationAngle = 0.0f;
    glm::vec3 lastBallPos = { 0,0,0 };

    engine.SetFixedUpdateCallback([&](double dt) {
        // Rotate the crane base
        rotationAngle += 1.0f * static_cast<float>(dt);
        auto* baseTc = scene.GetComponent<TransformComponent>(craneBase);
        baseTc->rotation = glm::angleAxis(rotationAngle, glm::vec3(0, 1, 0));

        // Let the TransformPropagationSystem evaluate the new hierarchy world transforms
        tpsPtr->Update();

        // Sync the Wrecking Ball's physics transform to the bottom of the cable
        auto* cableWorld = scene.GetComponent<WorldTransformComponent>(cable);
        auto* ballTc = scene.GetComponent<TransformComponent>(wreckingBall);
        auto* ballRb = scene.GetComponent<RigidBodyComponent>(wreckingBall);

        glm::vec3 targetPos = cableWorld->worldPosition + glm::vec3(0, -4.0f, 0); // Tip of the cable

        // Inject velocity so it can transfer momentum to the wall
        if (dt > 0.0)
        {
            ballRb->velocity = (targetPos - lastBallPos) / static_cast<float>(dt);
        }

        ballTc->position = targetPos;
        lastBallPos = targetPos;

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