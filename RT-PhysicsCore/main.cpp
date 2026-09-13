// main.cpp : Defines the entry point for the application.
//

#include <memory>

#include "RT-PhysicsCore/core/Engine.h"
#include "RT-PhysicsCore/utils/Log.h"
#include "RT-PhysicsCore/utils/ConsoleInput.h"

#include "RT-PhysicsCore/core/ecs/core/Scene.h"
#include "RT-PhysicsCore/core/ecs/core/System.h"
#include "RT-PhysicsCore/core/ecs/systems/TransformPropagationSystem.h"
#include "RT-PhysicsCore/core/ecs/components/TransformComponent.h"

#include "RT-PhysicsCore/physics/systems/PhysicsSystem.h"
#include "RT-PhysicsCore/physics/components/RigidBodyComponent.h"

#include "RT-PhysicsCore/rendering/systems/RenderSystem.h"

int main()
{
	// Optional: persist this run's log to a file too, in addition to the
	// console sink Log registers automatically. Remove this line if you
	// don't want a logs/ folder created next to the executable.
	RT_PhysicsCore::Log::AddSink(
		std::make_unique<RT_PhysicsCore::FileLogSink>(RT_PhysicsCore::DefaultLogFilePath()));

	RT_LOG_INFO("Hello RT-PhysicsCore.");

	RT_PhysicsCore::Engine engine(60.0); // 60 Hz

	RT_PhysicsCore::Scene scene;

	// Create systems
	scene.AddSystem(std::make_unique<RT_PhysicsCore::TransformPropagationSystem>(scene));
	scene.AddSystem(std::make_unique<RT_PhysicsCore::PhysicsSystem>(scene));
	scene.AddSystem(std::make_unique<RT_PhysicsCore::RenderSystem>(scene));

	// Create an entity
	RT_PhysicsCore::Entity e = scene.CreateEntity();

    RT_PhysicsCore::TransformComponent t;
    t.position = { -2.0f, 60.0f, 0.0f };

    RT_PhysicsCore::RigidBodyComponent rb;
    rb.mass = 1.0f;
    rb.velocity = { 0.5f, -9.81f, 0.0f };

	scene.AddComponent(e, t);
	scene.AddComponent(e, rb);

	engine.SetRenderCallback([&](double alpha) {
		scene.RenderUpdateSystems();
		});

	engine.SetUpdateCallback([&](double deltaTime) {
		scene.SetDeltaTime(deltaTime);
		scene.UpdateSystems();

		if (RT_PhysicsCore::ConsumeKeyPress())
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
