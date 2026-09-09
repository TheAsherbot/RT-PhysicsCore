// main.cpp : Defines the entry point for the application.
//

#include <conio.h>

#include "iostream"
#include "include/RT-PhysicsCore/core/Engine.h"


#include "include/RT-PhysicsCore/core/ecs/core/Scene.h"
#include "include/RT-PhysicsCore/core/ecs/core/System.h"
#include "include/RT-PhysicsCore/core/ecs/systems/TransformPropagationSystem.h"
#include "include/RT-PhysicsCore/core/ecs/systems/PhysicsSystem.h"
#include "include/RT-PhysicsCore/core/ecs/systems/RenderSystem.h"


int main()
{
	std::cout << "Hello RT-PhysicsCore." << std::endl;

	RT_PhysicsCore::Engine engine(60.0); // 60 Hz
	
    RT_PhysicsCore::Scene scene;

    // Create systems
    scene.AddSystem(std::make_unique<RT_PhysicsCore::TransformPropagationSystem>(scene));
    scene.AddSystem(std::make_unique<RT_PhysicsCore::PhysicsSystem>(scene));
    scene.AddSystem(std::make_unique<RT_PhysicsCore::RenderSystem>(scene));

    // Create an entity
    RT_PhysicsCore::Entity e = scene.CreateEntity();

    RT_PhysicsCore::TransformComponent t;
    t.position = { 0.0f, 0.0f, 0.0f };

    RT_PhysicsCore::RigidBodyComponent rb;
    rb.mass = 1.0f;
    rb.velocity = { 1.0f, 0.0f, 0.0f };

    scene.AddComponent(e, t);
    scene.AddComponent(e, rb);


	engine.SetRenderCallback([&](double alpha) {
		scene.RenderUpdateSystems();
		// Render logic here
		// std::cout << "Rendering with alpha: " << alpha << std::endl;
		});
	engine.SetUpdateCallback([&](double deltaTime) {

		scene.SetDeltaTime(deltaTime);
		scene.UpdateSystems();
		// Update logic here
		// std::cout << "Updating with delta time: " << deltaTime << std::endl;

		if (_kbhit())
		{
			_getch();
			engine.RequestExit(); // Exit the engine if input is detected
		}
		});
	engine.SetFixedUpdateCallback([&](double fixedDeltaTime) {
		scene.SetFixedDeltaTime(fixedDeltaTime);
		scene.FixedUpdateSystems();
		// Update logic here
		// std::cout << "Updating with fixed delta time: " << fixedDeltaTime << std::endl;
		});
	
		std::cout << "START!: " << std::endl;
	engine.Run();

	return 0;
}
