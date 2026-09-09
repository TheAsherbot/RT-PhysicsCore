# Week 1 Dev Log — Core Engine Architecture

## Goals
- Implement update loop
- Build fixed time-step manager
- Make math library (Vec2/Vec3)
- begin ECS design
- Add logging system

## What I accomplished
- Built Engine.cpp with main loop, render loop, and fixed update loop
- Added FixedTimestep.cpp with accumulator logic
- Added GLM for math library

## Challenges
- I did not understand how CMakeLists.txt worked
- I did not understand how to add GLM to CMake to auto build.
- I ran out of time before my 5 Labor Day weekend trip started.

## Solutions / Decisions
- I had to spend a lot of time learning CMake and how to add GLM to the project. I decided to use the CMake FetchContent module to download and build GLM automatically.

## Next Week
- Add ECS design
- Add Logging system
- Begin window creation for OpenGL layer