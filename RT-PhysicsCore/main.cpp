// main.cpp : Defines the entry point for the application.
//

#include <memory>

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
	// Optional: persist this run's log to a file too, in addition to the
	// console sink Log registers automatically. Remove this line if you
	// don't want a logs/ folder created next to the executable.
	RT_PhysicsCore::Log::AddSink(
		std::make_unique<RT_PhysicsCore::FileLogSink>(RT_PhysicsCore::DefaultLogFilePath()));

	RT_LOG_INFO("Hello RT-PhysicsCore.");

	RT_PhysicsCore::Renderer renderer(1280, 720, "RT-PhysicsCore");
	if (!renderer.IsValid())
	{
		RT_LOG_FATAL("Renderer failed to initialize - see the errors above. Exiting.");
		return 1;
	}

	RT_PhysicsCore::Engine engine(60.0); // 60 Hz

	RT_PhysicsCore::Scene scene;

	// Create systems
	scene.AddSystem(std::make_unique<RT_PhysicsCore::TransformPropagationSystem>(scene));
	scene.AddSystem(std::make_unique<RT_PhysicsCore::PhysicsSystem>(scene));

	// CollisionSystem is owned by the scene once moved in, but the raw
	// pointer grabbed beforehand stays valid - the unique_ptr moving
	// doesn't relocate the CollisionSystem object itself, only the
	// pointer bookkeeping. ResolutionSystem needs that reference to read
	// this step's contacts, the same way RenderSystem needs Renderer&.
	auto collisionSystem = std::make_unique<RT_PhysicsCore::CollisionSystem>(scene);
	RT_PhysicsCore::CollisionSystem* collisionSystemPtr = collisionSystem.get();
	scene.AddSystem(std::move(collisionSystem));

	// Solver/iteration mode default to SequentialImpulses + Fixed - flip
	// with resolutionSystemPtr->SetSolverMode(...)/SetIterationMode(...)
	// once there's a hotkey (or anything else) calling them.
	auto resolutionSystem = std::make_unique<RT_PhysicsCore::ResolutionSystem>(scene, *collisionSystemPtr, RT_PhysicsCore::ResolutionSystem::SolverMode::SequentialImpulses);
	RT_PhysicsCore::ResolutionSystem* resolutionSystemPtr = resolutionSystem.get();
	scene.AddSystem(std::move(resolutionSystem));
	(void)resolutionSystemPtr; // unused until a hotkey handler calls SetSolverMode/SetIterationMode on it

	scene.AddSystem(std::make_unique<RT_PhysicsCore::RenderSystem>(scene, renderer));

	// Create an entity
	RT_PhysicsCore::Entity testShpere = scene.CreateEntity();
	constexpr float sphereMass = 1.0f;
	constexpr float sphereRadius = 0.5f;

	RT_PhysicsCore::TransformComponent t;
	t.position = { -2.0f, 60.0f, 0.0f };
	t.scale = { sphereRadius * 2.0f, sphereRadius * 2.0f, sphereRadius * 2.0f };

	// Unit sphere - radius matches TransformComponent's default scale of 1.
	RT_PhysicsCore::RigidBodyComponent rb = RT_PhysicsCore::MakeDynamicBody(
		sphereMass, RT_PhysicsCore::ComputeSphereInertia(sphereMass, sphereRadius));
	rb.velocity = { 0.5f, -9.81f, 0.0f };

	RT_PhysicsCore::MeshComponent mesh;
	mesh.shape = RT_PhysicsCore::PrimitiveShape::Sphere;
	mesh.color = { 0.9f, 0.3f, 0.2f };

	RT_PhysicsCore::ColliderComponent collider;
	collider.shape = RT_PhysicsCore::ColliderShape::Sphere;
	collider.size = { sphereRadius, 0.0f, 0.0f };

	scene.AddComponent(testShpere, t);
	scene.AddComponent(testShpere, rb);
	scene.AddComponent(testShpere, mesh);
	scene.AddComponent(testShpere, collider);

	// Ground plane - static rigid body (infinite mass/inertia), so
	// ResolutionSystem doesn't need to special-case "collider with no
	// RigidBodyComponent" anywhere.
	RT_PhysicsCore::Entity ground = scene.CreateEntity();
	RT_PhysicsCore::TransformComponent groundTransform;
	groundTransform.position = { 0.0f, -0.5f, 0.0f };
	groundTransform.scale = { 10.0f, 1.0f, 10.0f };

	RT_PhysicsCore::RigidBodyComponent groundBody = RT_PhysicsCore::MakeStaticBody();

	RT_PhysicsCore::MeshComponent groundMesh;
	groundMesh.shape = RT_PhysicsCore::PrimitiveShape::Cube;
	groundMesh.color = { 0.3f, 0.45f, 0.3f };

	// ColliderShape). X/Z footprint matches the visual scale; Y thickness
	// is deliberately NOT tied to scale.y - a plane has no visual
	// thickness to match, so a thin collider there only invites tunneling
	// with no visual benefit. offset shifts the collider's center down by
	// its own half-extent so its TOP face still lands exactly at y=0
	// (the visible surface) instead of the collider's center sitting
	// there and the top floating half a unit above it. At 1 unit of
	// total thickness, an object needs to cross more than ~60 units/sec
	// in one fixed step (60 Hz) to have a shot at tunneling through -
	// comfortable for most things; revisit (or add CCD) for anything
	// genuinely bullet-fast.
	RT_PhysicsCore::ColliderComponent groundCollider;
	groundCollider.shape = RT_PhysicsCore::ColliderShape::Box;
	constexpr float groundHalfThickness = 0.5f;
	groundCollider.size = { groundTransform.scale.x * 0.5f, groundHalfThickness, groundTransform.scale.z * 0.5f };
	groundCollider.offset = { 0.0f, groundHalfThickness, 0.0f };

	scene.AddComponent(ground, groundTransform);
	scene.AddComponent(ground, groundBody);
	scene.AddComponent(ground, groundMesh);
	scene.AddComponent(ground, groundCollider);

	engine.SetRenderCallback([&](double alpha) {
		scene.RenderUpdateSystems();
		});
	engine.SetUpdateCallback([&](double deltaTime) {
		scene.SetDeltaTime(deltaTime);
		scene.UpdateSystems();

		// A real window is the thing to watch now, not the console: quit
		// when the user closes it (or GLFW otherwise signals close).
		if (renderer.ShouldClose())
		{
			engine.RequestExit();
		}
		});
	engine.SetFixedUpdateCallback([&](double fixedDeltaTime) {
		scene.SetFixedDeltaTime(fixedDeltaTime);
		scene.FixedUpdateSystems();
		});

	RT_LOG_INFO("START!");
	engine.Run();

	return 0;
}