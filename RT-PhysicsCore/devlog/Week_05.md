# Week 5 Dev Log — Telemetry, Profiling & Test Suite

## Goals
- Memory Optimization
- CPU + GPU optimization
- Material and friction system
- Expand test coverage and documentation

## What I accomplished
- Added material system with Coulomb friction cone and restitution thresholds
- Built custom `MemoryTracker` with overloaded `new`/`delete` to monitor allocations and peak heap usage
- Added scoped profiling (`RT_PROFILE_SCOPE`) and `TelemetryManager` exporting Chrome Tracing JSON files
- Integrated Dear ImGui for an in-engine HUD (F3 toggle) showing FPS, frame timings, and memory stats
- Created a standalone profiling tool (`RT-ProfilerViewer.exe`) to load, scrub, and inspect historical session traces
- Added wireframe cylinder debug drawing
- Added tuned material presets (clay, wood, rubber, domino, super ball, etc.)
- Added 15+ automated test suites covering ECS, collisions, solvers, materials, and performance benchmarks
- Overhauled documentation with comprehensive Doxygen comments and separated engine and physics design docs

## Challenges
- Tracking heap memory without causing infinite loops inside overloaded `new`/`delete` or adding heavy overhead to the physics step.
- Integrating Dear ImGui with GLFW and OpenGL cleanly without corrupting the simulation's render state.
- Found minor bugs while expanding tests, like an unfreed `FixedTimestep` in the Engine destructor and AABB offsets using raw positions instead of collider centers.

## Solutions / Decisions
- Used atomic counters inside `MemoryTracker` to keep memory monitoring fast and thread-safe.
- Created a `ScopedRenderFrame` RAII wrapper to safely handle OpenGL and ImGui context lifecycles.
- Decided to build the profiler session inspector as an independent tool (`RT-ProfilerViewer`) so historical trace analysis doesn't impact engine performance.
- Cleaned up collider center offsets for AABBs and separated documentation into `engine_core.md` and `physics_design.md`.

## Next Week
- Build interactive physics demo sandboxes
- Optimize ECS component storage for cache locality
- Improve angular / rotational integration to stop energy drift