/**
 * @file WreckingBall.cpp
 * @brief Demonstrates combining ECS transform hierarchies with physical simulation.
 */

#include <memory>
#include <vector>
#include <GLFW/glfw3.h>
#include <imgui/imgui.h>

#include "RT-PhysicsCore/core/Engine.h"
#include "RT-PhysicsCore/core/ecs/core/Scene.h"
#include "RT-PhysicsCore/core/ecs/systems/TransformPropagationSystem.h"
#include "RT-PhysicsCore/core/ecs/components/TransformComponent.h"
#include "RT-PhysicsCore/core/ecs/components/HierarchyComponent.h"
#include "RT-PhysicsCore/physics/systems/PhysicsPipelineSystem.h"
#include "RT-PhysicsCore/physics/components/RigidBodyComponent.h"
#include "RT-PhysicsCore/physics/MassProperties.h"
#include "RT-PhysicsCore/physics/components/ColliderComponent.h"
#include "RT-PhysicsCore/rendering/Renderer.h"
#include "RT-PhysicsCore/rendering/systems/RenderSystem.h"
#include "RT-PhysicsCore/rendering/components/MeshComponent.h"

#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/constants.hpp>

int main()
{
    using namespace RT_PhysicsCore;

    Renderer renderer(1280, 720, "Example: Wrecking Ball");
    if (!renderer.IsValid())
    {
        return 1;
    }

    Engine engine(60.0);
    Scene scene;

    auto tps = std::make_unique<TransformPropagationSystem>(scene);
    TransformPropagationSystem* tpsPtr = tps.get();
    scene.AddSystem(std::move(tps));

    scene.AddSystem(std::make_unique<PhysicsPipelineSystem>(
        scene, ResolutionSystem::SolverMode::SequentialImpulses, 4));
    scene.AddSystem(std::make_unique<RenderSystem>(scene, renderer));

    // Entity handles
    Entity craneBase = invalidEntity;
    Entity craneArm = invalidEntity;
    Entity cable = invalidEntity;
    Entity wreckingBall = invalidEntity;

    float rotationAngle = glm::pi<float>(); // Starts 180 degrees away from the wall
    glm::vec3 lastBallPos = { -8.1f, 2.8f, 0.0f };
    bool showFps = false;

    auto setup = [&]()
        {
            std::vector<Entity> toDestroy = scene.GetEntities();
            for (Entity e : toDestroy)
            {
                scene.DestroyEntity(e);
            }

            // Ground
            Entity ground = scene.CreateEntity();
            TransformComponent gtc;
            gtc.position = { 0.0f, -0.5f, 0.0f };
            gtc.scale = { 40.0f, 1.0f, 40.0f };
            scene.AddComponent(ground, gtc);
            scene.AddComponent(ground, MakeStaticBody());
            MeshComponent gm;
            gm.shape = PrimitiveShape::Cube;
            gm.color = { 0.2f, 0.3f, 0.2f };
            scene.AddComponent(ground, gm);
            ColliderComponent gcc;
            gcc.shape = ColliderShape::Box;
            gcc.size = { 20.0f, 0.5f, 20.0f };
            scene.AddComponent(ground, gcc);

            // --- The Kinematic Crane Hierarchy (Visuals Only) ---
            craneBase = scene.CreateEntity();
            TransformComponent tcBase;
            tcBase.position = { 0.0f, 3.0f, 0.0f };
            tcBase.scale = { 2.0f, 15.0f, 2.0f };
            // Start crane facing left (180 degrees rotated)
            tcBase.rotation = glm::angleAxis(glm::pi<float>(), glm::vec3(0, 1, 0));
            scene.AddComponent(craneBase, tcBase);
            MeshComponent mcBase;
            mcBase.shape = PrimitiveShape::Cube;
            mcBase.color = { 0.3f, 0.3f, 0.3f };
            scene.AddComponent(craneBase, mcBase);

            craneArm = scene.CreateEntity();
            TransformComponent tcArm;
            tcArm.position = { 4.0f / tcBase.scale.x, (15.0f + 0.6f) / (tcBase.scale.y) / 2.0f, 0.0f };
            tcArm.scale = { 8.0f / tcBase.scale.x, 0.6f / tcBase.scale.y, 0.6f / tcBase.scale.z };
            scene.AddComponent(craneArm, tcArm);
            MeshComponent mcArm;
            mcArm.shape = PrimitiveShape::Cube;
            mcArm.color = { 0.8f, 0.8f, 0.2f };
            scene.AddComponent(craneArm, mcArm);
            scene.SetParent(craneArm, craneBase);

            cable = scene.CreateEntity();
            TransformComponent tcCable;
            tcCable.position = { 4.1f / (tcArm.scale.x) / (tcBase.scale.x), -4.0f / (tcArm.scale.y) / (tcBase.scale.y), 0.0f };
            tcCable.scale = { 0.1f / (tcArm.scale.x) / (tcBase.scale.x), 8.0f / (tcArm.scale.y) / (tcBase.scale.y), 0.1f / (tcArm.scale.z) / (tcBase.scale.z) };
            scene.AddComponent(cable, tcCable);
            MeshComponent mcCable;
            mcCable.shape = PrimitiveShape::Cube;
            mcCable.color = { 0.1f, 0.1f, 0.1f };
            scene.AddComponent(cable, mcCable);
            scene.SetParent(cable, craneArm);

            // Propagate transforms so the cable tip world position is valid
            tpsPtr->Update();

            // --- The Physical Wrecking Ball ---
            // Starts on the left side of the screen (x = -8.1m)
            wreckingBall = scene.CreateEntity();
            TransformComponent tcBall;
            tcBall.position = { -8.1f, 2.8f, 0.0f };
            tcBall.scale = { 1.6f, 1.6f, 1.6f };
            scene.AddComponent(wreckingBall, tcBall);
            MeshComponent mcBall;
            mcBall.shape = PrimitiveShape::Sphere;
            mcBall.color = { 0.8f, 0.2f, 0.2f };
            scene.AddComponent(wreckingBall, mcBall);
            // Infinite mass kinematic body so it plows through the wall
            scene.AddComponent(wreckingBall, MakeStaticBody());
            ColliderComponent ccBall;
            ccBall.shape = ColliderShape::Sphere;
            ccBall.size = { 0.8f, 0.0f, 0.0f };
            scene.AddComponent(wreckingBall, ccBall);

            // --- The Wall of Boxes (Right Side of Screen) ---
            // Centered at x = 8.1m along the sweep arc, spanning x in [5.6, 10.6] and y in [0.5, 4.5]
            for (int y = 0; y < 5; ++y)
            {
                for (int x = 0; x < 6; ++x)
                {
                    Entity b = scene.CreateEntity();
                    TransformComponent tc;
                    tc.position = { 5.6f + static_cast<float>(x) * 1.0f, 0.5f + static_cast<float>(y) * 1.0f, 0.0f };
                    tc.scale = { 1.0f, 1.0f, 1.0f };
                    scene.AddComponent(b, tc);
                    scene.AddComponent(b, MakeDynamicBody(1.0f, ComputeBoxInertia(1.0f, { 0.5f, 0.5f, 0.5f })));
                    ColliderComponent cc;
                    cc.shape = ColliderShape::Box;
                    cc.size = { 0.5f, 0.5f, 0.5f };
                    scene.AddComponent(b, cc);
                    MeshComponent mc;
                    mc.shape = PrimitiveShape::Cube;
                    mc.color = { 0.8f, 0.4f + static_cast<float>(y) * 0.1f, 0.2f };
                    scene.AddComponent(b, mc);
                }
            }

            rotationAngle = glm::pi<float>();
            lastBallPos = { -8.1f, 2.8f, 0.0f };
        };

    setup();

    engine.SetUpdateCallback([&](double dt)
        {
            scene.SetDeltaTime(dt);
            scene.UpdateSystems();

            auto& input = renderer.GetInput();
            if (input.WasKeyPressed(GLFW_KEY_F3))
            {
                showFps = !showFps;
            }
            if (input.WasKeyPressed(GLFW_KEY_R))
            {
                setup();
            }
            if (input.WasKeyPressed(GLFW_KEY_ESCAPE) || renderer.ShouldClose())
            {
                engine.RequestExit();
            }
        });

    engine.SetFixedUpdateCallback([&](double dt)
        {
            // Rotate the crane base (takes ~3.14 seconds to rotate the full 180 degrees into the wall)
            rotationAngle += 1.0f * static_cast<float>(dt);
            auto* baseTc = scene.GetComponent<TransformComponent>(craneBase);
            if (baseTc)
            {
                baseTc->rotation = glm::angleAxis(rotationAngle, glm::vec3(0, 1, 0));
            }

            // Let the TransformPropagationSystem evaluate the new hierarchy world transforms
            tpsPtr->Update();

            // Sync the Wrecking Ball's physics transform to the bottom of the cable
            auto* cableWorld = scene.GetComponent<WorldTransformComponent>(cable);
            auto* ballTc = scene.GetComponent<TransformComponent>(wreckingBall);
            auto* ballRb = scene.GetComponent<RigidBodyComponent>(wreckingBall);

            if (cableWorld && ballTc && ballRb)
            {
                glm::vec3 targetPos = cableWorld->worldPosition + glm::vec3(0.0f, -4.0f, 0.0f);

                // Inject velocity so it transfers realistic momentum into the wall
                if (dt > 0.0)
                {
                    ballRb->velocity = (targetPos - lastBallPos) / static_cast<float>(dt);
                }

                ballTc->position = targetPos;
                lastBallPos = targetPos;
            }

            scene.SetFixedDeltaTime(dt);
            scene.FixedUpdateSystems();
        });

    engine.SetRenderCallback([&](double /*alpha*/)
        {
            renderer.BeginFrame();
            scene.RenderUpdateSystems();

            if (showFps)
            {
                ImGui::SetNextWindowPos(ImVec2(10.0f, 10.0f), ImGuiCond_Always);
                ImGui::SetNextWindowBgAlpha(0.65f);
                ImGuiWindowFlags flags = ImGuiWindowFlags_NoDecoration |
                    ImGuiWindowFlags_AlwaysAutoResize |
                    ImGuiWindowFlags_NoSavedSettings |
                    ImGuiWindowFlags_NoFocusOnAppearing |
                    ImGuiWindowFlags_NoNav |
                    ImGuiWindowFlags_NoMove;
                if (ImGui::Begin("##FPSOverlay", nullptr, flags))
                {
                    ImGui::TextColored(ImVec4(0.2f, 1.0f, 0.4f, 1.0f), "FPS: %.1f (%.2f ms)",
                        ImGui::GetIO().Framerate, 1000.0f / ImGui::GetIO().Framerate);
                }
                ImGui::End();
            }

            renderer.EndFrame();
        });

    engine.Run();
    return 0;
}