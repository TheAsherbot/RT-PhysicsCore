# RT-PhysicsCore

A real-time 3D rigid body physics engine built from scratch in **C++17** with a custom Entity-Component-System architecture, high-precision double-precision dynamics, island-based constraint solvers, continuous collision detection, telemetry profiling tools, and an integrated OpenGL 3.3 debug renderer.

> **Status:** Active Development · [Physics Design Doc](RT-PhysicsCore/docs/physics_design.md) · [Engine Architecture Doc](RT-PhysicsCore/docs/engine_core.md)

---

## Showcase

| Symplectic DLM Integration (Dzhanibekov Effect) | Contact Stacking & Warm-Starting |
|:---:|:---:|
| ![Dzhanibekov Tumbling](RT-PhysicsCore/docs/media/DzhanibekovDemo.gif) | ![Domino Chain](RT-PhysicsCore/docs/media/DominoChain.gif) |
| **Restitution & Momentum Transfer** | **Real-Time Profiler & Telemetry HUD** |
| ![Newton's Cradle](RT-PhysicsCore/docs/media/NewtonsCradle.gif) | ![Telemetry Overlay](RT-PhysicsCore/docs/media/ProfilerViewer.gif) |

---

## Features

### Physics Simulation

- **High-Precision Dynamics Layer** — Core integration and constraint calculations execute in double precision (`Real`, `Vec3`, `Quat`, `Mat3`), preventing rounding accumulation during tight constraint solves while seamlessly interoperating with single-precision GLM.
- **Master Substepping Pipeline** — Divides every engine tick into sub-steps (default: 4 substeps) for stiff stacking, high-speed stability, and robust constraint satisfaction.
- **6-DOF Rigid Body Dynamics** — Full linear and angular state with analytical inertia tensors for boxes, spheres, cylinders, and capsules (including parallel-axis theorem decomposition for capsule end-caps).
- **DLM Symplectic Rotational Integration** — Dullweber–Leimkuhler–McLachlan (DLM 1997) 5-step Strang splitting ($R_1(h/2) \circ R_2(h/2) \circ R_3(h) \circ R_2(h/2) \circ R_1(h/2)$). Strictly bounds kinetic energy drift to $<0.01\%$, exactly conserves angular momentum $\|\mathbf{L}\|$, and naturally captures intermediate-axis tumbling (the Dzhanibekov / tennis-racket effect).
- **Semi-Implicit Linear Euler** — Symplectic velocity-first integration preserves phase space energy for linear motion and resting contacts over long simulation runs.
- **Accumulator-Based Fixed Timestep** — Decouples 60 Hz physics from variable render rates with spiral-of-death clamping and sub-frame interpolation factor output.

### Collision Detection & CCD

- **Broad Phase** — Swept AABB overlap testing with oriented-box reprojection and velocity expansion to capture fast-moving candidates before narrow phase.
- **Narrow Phase** — Exact geometric primitive pair testing:
  | Pair | Method |
  |---|---|
  | Sphere–Sphere | Center distance test |
  | Sphere–Box | Closest-point clamping |
  | Sphere–Capsule | Point–segment distance |
  | Capsule–Capsule | Ericson segment–segment closest points |
  | Box–Capsule | Alternating projection with contact-plane interval detection |
  | Box–Box | 15-axis SAT with Sutherland–Hodgman face clipping |
- **Extremal 4-Point Manifold Reduction** — Clips contact faces to at most 4 extremal support points maximizing support area, maintaining physical stability while keeping LCP matrices compact.
- **Continuous Collision Detection (CCD)** — Bilateral conservative advancement via GJK distance queries calculates exact time-of-impact (TOI) and clamps motion fractions to prevent tunneling through thin geometry.
- **Persistent Contact Caching & Warm-Starting** — Caches manifolds across sub-steps with canonical pair hashing and local anchor matching, re-applying historical normal and friction impulses for instant stack stability.

### Constraint Resolution & Solvers

- **Simulation Island Partitioning** — Disjoint-set Union-Find algorithm groups interacting dynamic bodies into independent islands bounded by static bodies, isolating solver systems.
- **Decoupled Solver Backends** — Switchable constraint solvers implementing the `ISolverBackend` interface:
  - **Sequential Impulses (Projected Gauss-Seidel)**: Accumulated-impulse clamping allowing earlier overshoots to be undone across iterations.
  - **Exact Lemke LCP Solver**: Direct Delassus mobility matrix formulation solved simultaneously via Lemke's pivoting algorithm with Bland's rule cycle prevention in double precision. Eliminates contact ordering bias.
- **True 2D Coulomb Friction** — Coupled tangential impulse solving via Cramer's rule with circular cone clamping ($\|\boldsymbol{\lambda}_t\| \leq \mu \lambda_n$), replacing the standard decoupled square-pyramid approximation.
- **Split-Impulse Position Correction** — Resolves penetration by integrating pseudo-velocities directly into position and orientation without injecting artificial kinetic energy.

### Engine Architecture

- **Custom ECS** — Sparse-set component storage with contiguous memory layout for cache-friendly iteration, $O(1)$ lookup/insert/remove, free-list entity recycling, and fast variadic `Query<A, B, ...>()` via C++17 fold expressions.
- **Transform Hierarchy** — Automatic world-space propagation of position, rotation, and scale through parent-child relationships.
- **Modular System Pipeline** — `ISystem` base with `FixedUpdate` / `Update` / `RenderUpdate` hooks. Physics, collision, resolution, transform propagation, and rendering each run as independent systems.

### Telemetry & Profiling

- **Real-Time Instrumentation** — RAII time-block profiling (`RT_PROFILE_SCOPE`), frame-stage timings, and active memory allocation tracking (`MemoryTracker`).
- **In-Engine HUD Overlay** — Dear ImGui on-screen metrics display (`TelemetryOverlay`) showing FPS, step durations, and memory usage.
- **Dedicated Profiler Tool** — Interactive offline session inspector (`RT-ProfilerViewer`) and integrated runtime profiler window (`ProfilerWindow`).

### Rendering & Debug Tools

- **OpenGL 3.3 Core Renderer** — Directional lighting with ambient, proper normal matrix transforms ($M^{-T}$), and shared GPU primitive buffers. Fully encapsulated behind a Pimpl interface — no GL headers leak into the engine API.
- **Immediate-Mode Debug Drawing** — Lines, wireframe boxes, spheres, and cylinders with depth-tested and always-on-top passes.
- **Dual-Mode Camera** — FreeFly and Orbit modes with smooth mouse-look and scroll zoom.
- **Multi-Sink Logger** — ANSI color-coded console + timestamped file output with compile-time level stripping via `RT_LOG_ACTIVE_LEVEL`.

---

## Building

### Prerequisites

| Dependency | Version | Notes |
|---|---|---|
| C++ Compiler | C++17 support | MSVC, GCC, or Clang |
| CMake | ≥ 3.11 | Uses FetchContent |
| OpenGL | 3.3+ | GPU driver support |

GLFW and GLM are fetched automatically via CMake FetchContent. GLAD and Dear ImGui are vendored directly in `RT-PhysicsCore/external/` — no manual dependency installation is required.

### Build Steps

```bash
# Clone the repository
git clone https://github.com/TheAsherbot/RT-PhysicsCore.git
cd RT-PhysicsCore

# Configure and build
cmake -B build -S .
cmake --build build --config Release
```

### Build Targets

| Target | Type | Description |
|---|---|---|
| `RT_PhysicsEngine` | Static Library | Core engine: ECS, physics, rendering, telemetry, utilities |
| `RT-PhysicsCore` | Executable | Interactive demo application |
| `RT-ProfilerViewer` | Executable | Standalone GUI telemetry and profile session viewer |
| `PhysicsTests_Collision` | Executable | Narrow-phase collision test suite (11 automated tests + visual harness) |
| `PhysicsTests_*` | Executables | Dedicated unit suites for AABB, Friction, Integration, MassProperties, PositionCorrection, Restitution, SolverComparison, SolverStability |
| `ECSTests_TransformHierarchy` | Executable | ECS scene-graph hierarchy propagation verification |
| `PerformanceTests_*` | Executables | Benchmarks for Broadphase, Narrowphase throughput, Solver scaling, Pile stress, ECS churn, Adaptive iteration |
| `Example_*` | Executables | 8 interactive sandbox demos: FallingBoxes, BouncingBalls, DominoChain, GyroscopeDemo, InclinedPlane, NewtonsCradle, ShapeSoup, WreckingBall |

---

## Demo Controls

### Mouse

| Input | Action |
|---|---|
| Hold Right Mouse Button | Capture cursor and enable free-look |
| Release Right Mouse Button | Unlock cursor |
| Scroll Wheel | Adjust zoom distance (Orbit mode, 1–100 units) |

### Keyboard

| Key | FreeFly Mode | Orbit Mode |
|---|---|---|
| `W` / `S` | Fly forward / backward | Pan pivot forward / backward |
| `A` / `D` | Strafe left / right | Pan pivot left / right |
| `Space` | Ascend (+Y) | Pan pivot up |
| `Left Ctrl` | Descend (−Y) | Pan pivot down |
| `Tab` | Toggle between FreeFly ↔ Orbit modes | Toggle between FreeFly ↔ Orbit modes |
| `F3` | Cycle telemetry overlay: Off → Minimal → Full (opens Profiler) → Off | |
| `Escape` | Close the application | Close the application |

---

## Architecture

```
┌───────────────────────────────────────────────────────────┐
│                       Engine Loop                         │
│  Accumulator ──► FixedUpdate (0-N) ──► Update ──► Render  │
└─────────────────────────────┬─────────────────────────────┘
                              │
                    ┌─────────▼──────────┐
                    │PhysicsPipelineSystem│ Master Substepping (N substeps)
                    └─────────┬──────────┘
                              │
       ┌──────────────────────┼──────────────────────┐
       │                      │                      │
┌──────▼───────┐       ┌──────▼───────┐       ┌──────▼───────┐
│Broad & Narrow│       │ CCD & Cache  │       │Island Solver │
│   Phase      │       │ Warm-Starting│       │(PGS / Lemke) │
└──────────────┘       └──────────────┘       └──────────────┘
```

### Project Structure

```
RT-PhysicsCore/
├── include/RT-PhysicsCore/
│   ├── core/              # Engine loop, ECS framework, MathTypes (Real, Vec3, Mat3, Quat)
│   │   └── ecs/           # Entity, Scene, ComponentStorage, Systems
│   ├── physics/           # Dynamics, materials, collision, solvers
│   │   ├── collision/     # AABB, NarrowPhase, Contact, ContactCache, GJK, ShapeCast
│   │   ├── components/    # RigidBody, Collider, PhysicsMaterial
│   │   ├── solver/        # ISolverBackend, IterativeSolver, ExactSolver, IslandBuilder
│   │   └── systems/       # PhysicsPipelineSystem, PhysicsSystem, CollisionSystem, ResolutionSystem
│   ├── rendering/         # Renderer (Pimpl), Camera, Input, RenderSystem
│   ├── telemetry/         # Profiler, MemoryTracker, TelemetryOverlay, Sessions
│   └── utils/             # Logging, DebugDraw, ConsoleInput
├── src/                   # All implementations (.cpp)
├── examples/              # Standalone demonstration sandboxes (8 examples)
├── tests/                 # Test suites (Physics, ECS, Performance)
├── tools/                 # Standalone tools (RT-ProfilerViewer)
├── docs/                  # Technical design documents
├── devlog/                # Weekly development logs
└── external/              # Vendored dependencies (GLAD, Dear ImGui)
```

---

## Documentation

Detailed technical documentation lives in [`RT-PhysicsCore/docs/`](RT-PhysicsCore/docs/):

- [**Architecture Specification**](RT-PhysicsCore/docs/architecture.md) — Multi-layer engine architecture, lifecycle flow, ECS mechanics, and system coordination.
- [**Performance & Validation**](RT-PhysicsCore/docs/benchmarks.md) — Narrowphase query throughput, ECS benchmarks, and physical conservation metrics.
- [**Engine Core**](RT-PhysicsCore/docs/engine_core.md) — ECS architecture, transform hierarchy math, fixed timestep loop, rendering pipeline, input system, and debug draw specifications.
- [**Physics Design**](RT-PhysicsCore/docs/physics_design.md) — Dynamics integration, DLM splitting, inertia tensors, collision algorithms, CCD, warm-starting, island graph partitioning, LCP formulation, Coulomb cone friction, and solver theory.
- [**Telemetry & Profiling**](RT-PhysicsCore/docs/telemetry_system.md) — Scoped profiling macros, lock-free memory tracking, Chrome Tracing serialization, dual-window OpenGL context sharing, and offline viewer replay.

---

## License

This project is open source. See [LICENSE](LICENSE) for details.