/**
 * @file DominoChain.cpp
 * @brief Box collision and rotation dynamics chain reaction with arc-length spacing and chord alignment.
 */

#include <cmath>
#include <memory>
#include <vector>
#include <GLFW/glfw3.h>
#include <imgui/imgui.h>

#include "RT-PhysicsCore/core/Engine.h"
#include "RT-PhysicsCore/core/ecs/core/Scene.h"
#include "RT-PhysicsCore/core/ecs/systems/TransformPropagationSystem.h"
#include "RT-PhysicsCore/core/ecs/components/TransformComponent.h"
#include "RT-PhysicsCore/physics/systems/PhysicsPipelineSystem.h"
#include "RT-PhysicsCore/physics/components/RigidBodyComponent.h"
#include "RT-PhysicsCore/physics/MassProperties.h"
#include "RT-PhysicsCore/physics/components/ColliderComponent.h"
#include "RT-PhysicsCore/physics/components/PhysicsMaterialComponent.h"
#include "RT-PhysicsCore/rendering/Renderer.h"
#include "RT-PhysicsCore/rendering/systems/RenderSystem.h"
#include "RT-PhysicsCore/rendering/components/MeshComponent.h"

#include <glm/gtc/quaternion.hpp>

int main()
{
    using namespace RT_PhysicsCore;

    Renderer renderer(1280, 720, "Example: Domino Chain");
    if (!renderer.IsValid())
    {
        return 1;
    }

    Engine engine(60.0);
    Scene scene;

    scene.AddSystem(std::make_unique<TransformPropagationSystem>(scene));
    scene.AddSystem(std::make_unique<PhysicsPipelineSystem>(
        scene, ResolutionSystem::SolverMode::SequentialImpulses, 6));
    scene.AddSystem(std::make_unique<RenderSystem>(scene, renderer));

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
            gtc.position = { 25.0f, -0.5f, 0.0f };
            gtc.scale = { 100.0f, 1.0f, 30.0f };
            scene.AddComponent(ground, gtc);
            scene.AddComponent(ground, MakeStaticBody());
            MeshComponent gm;
            gm.shape = PrimitiveShape::Cube;
            gm.color = { 0.2f, 0.2f, 0.2f };
            scene.AddComponent(ground, gm);
            ColliderComponent gcc;
            gcc.shape = ColliderShape::Box;
            gcc.size = { 50.0f, 0.5f, 50.0f };
            scene.AddComponent(ground, gcc);

            // ── Uniform Arc-Length S-Curve Sampling ──
            constexpr int kDominoCount = 44;
            constexpr int kHighResSamples = 1000;
            constexpr float kAmplitude = 2.0f; // Smooth, sweeping curve amplitude
            constexpr float kStartX = -6.0f;
            constexpr float kLengthX = 14.0f;

            std::vector<glm::vec3> denseCurve(kHighResSamples);
            std::vector<float> cumulativeDist(kHighResSamples, 0.0f);

            float totalArcLength = 0.0f;
            for (int i = 0; i < kHighResSamples; ++i)
            {
                float t = static_cast<float>(i) / (kHighResSamples - 1.0f);
                float x = kStartX + kLengthX * t;
                float z = kAmplitude * std::sin(t * glm::pi<float>() * 2.0f);
                denseCurve[i] = { x, 1.0f, z };

                if (i > 0)
                {
                    totalArcLength += glm::length(denseCurve[i] - denseCurve[i - 1]);
                    cumulativeDist[i] = totalArcLength;
                }
            }

            std::vector<glm::vec3> dominoPositions;
            dominoPositions.reserve(kDominoCount);
            float stepDistance = totalArcLength / (kDominoCount - 1.0f);

            for (int i = 0; i < kDominoCount; ++i)
            {
                float targetDist = i * stepDistance;
                int segIdx = 0;
                while (segIdx < kHighResSamples - 1 && cumulativeDist[segIdx + 1] < targetDist)
                {
                    segIdx++;
                }

                float segLength = cumulativeDist[segIdx + 1] - cumulativeDist[segIdx];
                float fraction = (segLength > 1e-6f) ? (targetDist - cumulativeDist[segIdx]) / segLength : 0.0f;
                glm::vec3 pos = glm::mix(denseCurve[segIdx], denseCurve[segIdx + 1], fraction);
                dominoPositions.push_back(pos);
            }

            // ── Domino Spawning with Chord Look-Ahead Alignment ──
            for (int i = 0; i < kDominoCount; ++i)
            {
                float tFraction = static_cast<float>(i) / (kDominoCount - 1.0f);
                glm::vec3 pos = dominoPositions[i];

                // Chord alignment: Orient forward along the vector directly pointing to the next domino
                glm::vec3 forwardVec;
                if (i < kDominoCount - 1)
                {
                    forwardVec = glm::normalize(dominoPositions[i + 1] - dominoPositions[i]);
                }
                else
                {
                    forwardVec = glm::normalize(dominoPositions[i] - dominoPositions[i - 1]);
                }

                float yawAngle = std::atan2(forwardVec.z, forwardVec.x);

                Entity e = scene.CreateEntity();
                TransformComponent tc;
                tc.position = pos;
                tc.scale = { 0.7f, 2.0f, 0.2f };
                // Thin Z axis oriented along the chord path toward the next domino
                tc.rotation = glm::angleAxis(-yawAngle + glm::pi<float>() * 0.5f, glm::vec3(0, 1, 0));
                scene.AddComponent(e, tc);

                scene.AddComponent(e, MakeDynamicBody(0.5f, ComputeBoxInertia(0.5f, { 0.35f, 1.0f, 0.1f })));
                ColliderComponent cc;
                cc.shape = ColliderShape::Box;
                cc.size = { 0.35f, 1.0f, 0.1f };
                scene.AddComponent(e, cc);

                MeshComponent mc;
                mc.shape = PrimitiveShape::Cube;
                mc.color = { 0.1f, 0.3f + 0.7f * tFraction, 0.9f - 0.7f * tFraction };
                scene.AddComponent(e, mc);

                // Wood material provides ideal low-friction face sliding without sticking
                PhysicsMaterialComponent pm;
                pm.material = MaterialId::Wood;
                scene.AddComponent(e, pm);
            }

            // ── Trigger Projectile ──
            Entity trigger = scene.CreateEntity();
            TransformComponent ttc;
            glm::vec3 firstDominoDir = glm::normalize(dominoPositions[1] - dominoPositions[0]);
            ttc.position = dominoPositions[0] - firstDominoDir * 1.5f + glm::vec3(0.0f, 0.4f, 0.0f);
            ttc.scale = { 0.6f, 0.6f, 0.6f };
            scene.AddComponent(trigger, ttc);

            RigidBodyComponent trb = MakeDynamicBody(2.5f, ComputeSphereInertia(2.5f, 0.3f));
            trb.velocity = firstDominoDir * 4.5f;
            scene.AddComponent(trigger, trb);

            ColliderComponent tcc;
            tcc.shape = ColliderShape::Sphere;
            tcc.size = { 0.3f, 0.0f, 0.0f };
            scene.AddComponent(trigger, tcc);

            MeshComponent tmc;
            tmc.shape = PrimitiveShape::Sphere;
            tmc.color = { 1.0f, 0.2f, 0.2f };
            scene.AddComponent(trigger, tmc);
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