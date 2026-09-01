// main.cpp : Defines the entry point for the application.
//

#include <conio.h>

#include "iostream"
#include "include/RT-PhysicsCore/core/Engine.h"


int main()
{
	std::cout << "Hello RT-PhysicsCore." << std::endl;

	RT_PhysicsCore::Engine engine(60.0); // 60 Hz
	
	engine.SetRenderCallback([](double alpha) {
		// Render logic here
		std::cout << "Rendering with alpha: " << alpha << std::endl;
		});
	engine.SetUpdateCallback([&engine](double deltaTime) {
		// Update logic here
		std::cout << "Updating with delta time: " << deltaTime << std::endl;

		if (_kbhit())
		{
			_getch();
			engine.RequestExit(); // Exit the engine if input is detected
		}
		});
	engine.SetFixedUpdateCallback([](double fixedDeltaTime) {
		// Update logic here
		std::cout << "Updating with fixed delta time: " << fixedDeltaTime << std::endl;
		});
	
	engine.Run();

	return 0;
}
