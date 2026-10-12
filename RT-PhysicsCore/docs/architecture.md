# RT-PhysicsCore — Architecture Specification

Comprehensive architectural breakdown of the RT-PhysicsCore engine, detailing layer boundaries, execution lifecycles, memory structures, and data flows.

---

## 1. System Layers

```
┌────────────────────────────────────────────────────────────┐
│                    Application & Demos                     │
├────────────────────────────────────────────────────────────┤
│                     Systems Pipeline                       │
│  PhysicsPipelineSystem │ TransformPropagation │ Render     │
├────────────────────────────────────────────────────────────┤
│                 Core Physics Subsystems                    │
│  BroadPhase │ NarrowPhase │ CCD │ IslandBuilder │ Solvers  │
├────────────────────────────────────────────────────────────┤
│                     ECS Foundation                         │
│       Scene  │  Entity  │  ComponentStorage (SparseSet)    │
├────────────────────────────────────────────────────────────┤
│                  Math & Utilities Layer                    │
│   MathTypes (Real, Vec3, Mat3, Quat) │ Profiler │ Log      │
└────────────────────────────────────────────────────────────┘
```

1. **Math & Utilities Layer**: High-precision mathematics (`Real`, `Vec3`, `Mat3`, `Quat`) executing in double precision, mixed-precision vector arithmetic, RAII scoped profiling macros, and multi-sink logging.
2. **ECS Foundation**: Cache-coherent entity-component-system utilizing sparse sets with contiguous array packing.
3. **Core Physics Subsystems**: Modular collision detection, conservative advancement CCD, contact manifold caching, island graph partitioning, and interchangeable solver backends.
4. **Systems Pipeline**: `ISystem` instances executed across `FixedUpdate`, `Update`, and `RenderUpdate` stages.
5. **Application & Demos**: Sandbox examples, automated test harnesses, and telemetry tools.

---

## 2. Core Execution Lifecycle

The engine coordinates variable rendering rates with fixed-rate physics via an accumulator loop (`Engine.cpp`):

```
                       Frame Start
                            │
               ┌────────────▼───────────┐
               │    Poll Input & Events │
               └────────────┬───────────┘
                            │
               ┌────────────▼───────────┐
               │ Accumulate Elapsed Time│
               └────────────┬───────────┘
                            │
              ┌─────────────▼─────────────┐
        ┌────►│ While (accum >= fixedDt)  │◄───┐
        │     └─────────────┬─────────────┘    │
        │                   │ Yes              │
        │     ┌─────────────▼─────────────┐    │
        │     │  FixedUpdateSystems(dt)   │────┘ (Repeats N substeps)
        │     └───────────────────────────┘
        │ No
 ┌──────▼──────┐
 │ UpdateSystems│ (Variable logic, camera, scene scripts)
 └──────┬──────┘
        │
 ┌──────▼────────────┐
 │RenderUpdateSystems│ (Interpolation, OpenGL 3.3 draw calls, ImGui HUD)
 └──────┬────────────┘
        │
    Swap Buffers
```

---

## 3. ECS Architecture (`ComponentStorage.h`, `Scene.h`)

- **Sparse-Set Storage**: Each component type is stored in a dedicated `ComponentStorage<T>` consisting of:
  - `dense`: Contiguous `std::vector<T>` enabling cache-friendly linear iteration and vectorization.
  - `sparse`: Direct lookup vector mapping entity IDs to dense indices.
  - `denseEntities`: Parallel array mapping dense indices back to owning entity IDs.
- **Complexity**: $O(1)$ component lookup, insertion, and deletion. Deletions swap with the last element to maintain contiguous packing.
- **Queries**: Variadic `Scene::Query<A, B, ...>()` evaluates matching entity intersections using C++17 fold expressions, iterating the smallest component pool first to minimize overhead.

---

## 4. Physics Substepping Pipeline (`PhysicsPipelineSystem.h`)

Each fixed tick $\Delta t$ executes $N$ substeps (default: $N = 4$, $h = \Delta t / N$). Within each substep:

1. **Integration**: `PhysicsSystem` applies forces and updates predictive velocities.
2. **Broad-Phase**: `CollisionSystem` swivels oriented boxes and expands swept AABBs by $\mathbf{v} h$.
3. **Narrow-Phase**: SAT and primitive routines generate contact points, reduced to 4 extremal vertices by `ManifoldReduction`.
4. **Continuous Collision**: `ContinuousCollisionSystem` evaluates bilateral conservative advancement via GJK and clamps `motionFraction`.
5. **Contact Caching**: `ContactCache` matches manifold points to warm-start normal and friction impulses.
6. **Island Partitioning**: `IslandBuilder` merges connected bodies into independent simulation islands using Union-Find.
7. **Constraint Solving**: `ResolutionSystem` delegates each island to the active solver backend (`IterativeSolver` or `ExactSolver`).
8. **Position Advance**: Final positions and orientations advance via DLM splitting, followed by pseudo-velocity position correction.