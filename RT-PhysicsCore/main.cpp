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
	scene.AddSystem(std::make_unique<RT_PhysicsCore::RenderSystem>(scene, renderer));

	// Create an entity
	RT_PhysicsCore::Entity e = scene.CreateEntity();

	RT_PhysicsCore::TransformComponent t;
	t.position = { -2.0f, 60.0f, 0.0f };

	// Unit sphere - radius matches TransformComponent's default scale of 1.
	constexpr float sphereMass = 1.0f;
	constexpr float sphereRadius = 1.0f;
	RT_PhysicsCore::RigidBodyComponent rb = RT_PhysicsCore::MakeDynamicBody(
		sphereMass, RT_PhysicsCore::ComputeSphereInertia(sphereMass, sphereRadius));
	rb.velocity = { 0.5f, -9.81f, 0.0f };

	RT_PhysicsCore::MeshComponent mesh;
	mesh.shape = RT_PhysicsCore::PrimitiveShape::Sphere;
	mesh.color = { 0.9f, 0.3f, 0.2f };

	scene.AddComponent(e, t);
	scene.AddComponent(e, rb);
	scene.AddComponent(e, mesh);

	// A visual-only ground plane - no RigidBodyComponent, so PhysicsSystem
	// leaves it alone; it just sits there so falling objects have
	// something to fall past.
	RT_PhysicsCore::Entity ground = scene.CreateEntity();
	RT_PhysicsCore::TransformComponent groundTransform;
	groundTransform.position = { 0.0f, 0.0f, 0.0f };
	groundTransform.scale = { 50.0f, 1.0f, 50.0f };
	RT_PhysicsCore::MeshComponent groundMesh;
	groundMesh.shape = RT_PhysicsCore::PrimitiveShape::Plane;
	groundMesh.color = { 0.3f, 0.45f, 0.3f };
	scene.AddComponent(ground, groundTransform);
	scene.AddComponent(ground, groundMesh);

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
