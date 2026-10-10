# RT-PhysicsCore

A real-time 3D rigid body physics engine built from scratch in **C++17** with a custom Entity-Component-System architecture, advanced constraint solvers, telemetry profiling tools, and an integrated OpenGL 3.3 debug renderer.

> **Status:** Active Development · [Physics Design Doc](RT-PhysicsCore/docs/physics_design.md) · [Engine Architecture Doc](RT-PhysicsCore/docs/engine_core.md)

<!-- TODO: Add a GIF/screenshot of the demo here -->
<!-- ![RT-PhysicsCore Demo](docs/images/demo.gif) -->

---

## Features

### Physics Simulation

- **6-DOF Rigid Body Dynamics** — Full linear and angular state with analytical inertia tensors for boxes, spheres, cylinders, and capsules (including parallel-axis theorem decomposition for capsule end-caps).
- **DLM Symplectic Rotational Integration** — Decomposes rigid-body angular motion using the Dullweber–Leimkuhler–McLachlan (DLM 1997) symplectic splitting scheme ($R_1(h/2) \circ R_2(h/2) \circ R_3(h) \circ R_2(h/2) \circ R_1(h/2)$). Strictly bounds kinetic energy drift to $<0.01\%$, exactly conserves angular momentum $\|\mathbf{L}\|$, and naturally captures the tennis-racket / Dzhanibekov effect without numerical damping.
- **Semi-Implicit Linear Euler** — Velocity-first integration maintains symplectic energy conservation for linear motion and resting contacts over long simulation runs.
- **Accumulator-Based Fixed Timestep** — Decouples 60 Hz physics from variable render rates with spiral-of-death clamping and sub-frame interpolation factor output.

### Collision Detection

- **Broad Phase** — AABB overlap testing with oriented-box reprojection and capsule bounding via hemisphere union.
- **Narrow Phase** — Six primitive pair handlers:
  | Pair | Method |
  |---|---|
  | Sphere–Sphere | Distance check |
  | Sphere–Box | Closest-point clamping |
  | Sphere–Capsule | Point–segment distance |
  | Capsule–Capsule | Ericson segment–segment closest points |
  | Box–Capsule | Alternating projection (segment to box surface) with contact-plane overlap interval detection |
  | Box–Box | 15-axis SAT with Sutherland–Hodgman face clipping |

- **Multi-Point Contact Manifolds** — Box–Box generates up to 8 contact points via polygon clipping for stable flat stacking. Parallel capsule and capsule-on-face contacts emit 2-point manifolds to prevent unrealistic rolling.

### Constraint Resolution

- **Sequential Impulses (Projected Gauss-Seidel)** — Accumulated-impulse clamping allows correction of earlier overshoots across solver iterations.
- **Exact Lemke LCP Solver** — Formulates all contact constraints as a Linear Complementarity Problem ($w = Mz + q$) and solves simultaneously via Lemke's pivoting algorithm with Bland's rule cycle prevention in double precision. Eliminates contact ordering bias.
- **True 2D Coulomb Friction** — Coupled tangential impulse solving via Cramer's rule with circular cone clamping ($\|\lambda_t\| \leq \mu \cdot \lambda_n$), replacing the standard decoupled square-pyramid approximation.
- **Split-Impulse Position Correction** — Resolves penetration by directly adjusting positions and orientations without injecting artificial kinetic energy (no Baumgarte stabilization).
- **Adaptive Iteration Budgets** — Optional time-budgeted solver mode dynamically scales iteration counts within configurable millisecond caps.

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
└──────────┬────────────────┬─────────────────┬─────────────┘
           │                │                 │
     ┌─────▼──────┐    ┌────▼────┐     ┌──────▼──────┐
     │  Physics   │    │Transform│     │   Render    │
     │  Pipeline  │    │  Prop.  │     │   System    │
     └─────┬──────┘    └─────────┘     └─────────────┘
           │
    ┌──────▼───────┐
    │ PhysicsSystem│  Symplectic Euler linear + DLM rotational splitting
    └──────┬───────┘
    ┌──────▼───────┐
    │  Collision   │  Broad phase (AABB) → Narrow phase (SAT / Primitives)
    │   System     │
    └──────┬───────┘
    ┌──────▼───────┐
    │  Resolution  │  Sequential Impulses  or  Lemke LCP
    │   System     │  + Coulomb friction + position correction
    └──────────────┘
```

### Project Structure

```
RT-PhysicsCore/
├── include/RT-PhysicsCore/
│   ├── core/              # Engine loop, ECS framework
│   │   └── ecs/           # Entity, Scene, ComponentStorage, Systems
│   ├── physics/           # Dynamics, materials, collision, solvers
│   │   ├── collision/     # AABB, NarrowPhase, Contact
│   │   ├── components/    # RigidBody, Collider, PhysicsMaterial
│   │   └── systems/       # PhysicsSystem, CollisionSystem, ResolutionSystem
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

- [**Physics Design**](RT-PhysicsCore/docs/physics_design.md) — Dynamics integration, DLM splitting, inertia tensors, collision algorithms, LCP formulation, Coulomb cone friction, and solver theory.
- [**Engine Core**](RT-PhysicsCore/docs/engine_core.md) — ECS architecture, transform hierarchy math, fixed timestep loop, rendering pipeline, input system, and debug draw specifications.

---

## License

This project is open source. See [LICENSE](LICENSE) for details.
