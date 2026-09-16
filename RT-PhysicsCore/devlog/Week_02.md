# Week 2 Dev Log — OpenGL Visualization Layer

## Goals
- Add ECS design
- Add Logging system
- Begin window creation for OpenGL layer
- Basic Shader Compilation
- Rendering simple 3D objects (cube, sphere, etc)

## What I accomplished
- Added the foundation for the ECS design
- Added a logging system
- Reorginized the project structure to be more modular and easier to navigate
- Added Cross-Platform Input detection.

## Challenges
- Never looked into the backend of ECS design before, so it was a bit of a challenge to understand how to implement it in a way that is efficient and flexible.
- Was using Windows Only Input to determine if a key was pressed to stop the engine, but I want to make it cross platform, so I need to look into a better way to handle input.

## Solutions / Decisions
- Looked up youtube videos, and at other open source engines to learn how to implement ECS design.
- Make ConsoleInput class that will handle input for windows in a #ifdef, and a #else for POSIX (Linux/Mac OS) systems.

## Next Week
- Debug drawing
- Camera
- 3D model loading?