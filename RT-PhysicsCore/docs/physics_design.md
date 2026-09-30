# RT-PhysicsCore — Engine Core

Design decisions and the formulas behind them: ECS, transform hierarchy,
engine loop, rendering, input, debug drawing. Physics lives in a separate
`physics.md`.

## Engine Loop

- Frame order: `FixedUpdate` (0+ times) → `Update` → `Render`. Order
  matters: `FixedUpdate` writes `TransformComponent`, `Update`
  (`TransformPropagationSystem`) reads it and writes
  `WorldTransformComponent`, `Render` (`RenderSystem`) reads that. Wrong
  order = rendering one frame stale.
- `FixedTimestep` accumulator: each call, measure real elapsed time, clamp
  to `4× fixedDelta` (spiral-of-death guard — a debugger break or OS hitch
  can't cause a runaway catch-up loop), add to the accumulator, subtract
  `fixedDelta` repeatedly counting whole steps, remainder stays in the
  accumulator.
- `Alpha = accumulator / fixedDelta` — interpolation fraction; computed but
  not yet used for render interpolation (render just draws the latest
  post-physics state as-is).

## ECS Core

- `Entity` = plain `uint32_t`, `0` = invalid. IDs recycled via a free-list
  (`DestroyEntity` pushes, `CreateEntity` pops before incrementing the
  counter).
- Component storage = sparse set: dense `vector<T>` + parallel
  `vector<Entity>` + `unordered_map<Entity, index>`. O(1) add/get/has;
  remove is swap-and-pop (doesn't preserve order).
- Storages are type-erased behind an `IComponentStorage` virtual interface
  (`Has`/`Remove`) so `Scene` can hold many component types in one map and
  strip all of an entity's components without knowing their concrete
  types. (`unique_ptr<void>` doesn't work for this — `delete` on `void*` is
  ill-formed.)
- `Query<Ts...>()` iterates the *first* listed type's storage, filters by
  the rest — not the smallest storage. List the rarest component first for
  speed.
- `ISystem` holds a `Scene&` injected once at construction; three virtual
  hooks (`Update`/`FixedUpdate`/`RenderUpdate`), empty by default —
  override only what you need.
- `DestroyEntity` fixes up hierarchy links *before* removing components —
  fix-up needs to read the entity's own `HierarchyComponent` first.

## Transform Hierarchy

- `TransformComponent` = local position/rotation(quat)/scale.
  `WorldTransformComponent` = computed world-space equivalent.
- `TransformPropagationSystem` formula:
  ```
  worldPos   = parentWorldPos + parentWorldRot * (parentWorldScale * localPos)
  worldRot   = parentWorldRot * localRot
  worldScale = parentWorldScale * localScale        (component-wise)
  ```
- The `worldPos` term matters: naive `parentPos + localPos` ignores parent
  rotation/scale, so rotating a parent wouldn't carry its children around
  with it. This rotates/scales the local offset into world space first.
- Limitation: exactly equivalent to full `T·R·S` matrix composition only
  when the parent's scale is *uniform*. Non-uniform parent scale + a
  rotated child introduces shear a position/rotation/scale triple can't
  represent. Keep parent scale uniform if rotated children need to inherit
  it correctly.

## Rendering

- Camera orientation: yaw/pitch (degrees) → facing direction, spherical→Cartesian:
  ```
  front = normalize(cos(yaw)*cos(pitch), sin(pitch), sin(yaw)*cos(pitch))
  right = normalize(cross(front, worldUp))
  up    = normalize(cross(right, front))
  ```
  Default `yaw=-90°, pitch=0°` so default facing is `-Z` (OpenGL
  convention). View = `lookAt(pos, pos+front, up)`; projection = standard
  perspective.
- Model matrix: `model = translate(worldPos) * mat4_cast(worldRot) *
  scale(worldScale)` — same T·R·S order as the transform hierarchy, so a
  mesh always renders where its `WorldTransformComponent` says it is.
- Normals transform by `mat3(transpose(inverse(model)))`, not the plain
  model matrix — under non-uniform scale, the plain matrix doesn't
  preserve the angle between a surface and its normal. Matters here
  because mesh scale (reused from `TransformComponent::scale`) is often
  non-uniform.
- Shading: one hardcoded directional light, Lambertian + ambient floor —
  `color = 0.3*base + 0.7*max(dot(normal,-lightDir),0)*base`. No
  specular/shadows/multi-light, by design.
- Depth isn't linear in distance — with near/far `0.1/500`, a point 1 unit
  away already sits ~90% through the `[0,1]` depth range. Watch for
  z-fighting if far gets pushed out a lot.
- Primitives (cube/sphere/plane) are generated once at startup into shared
  VAO/VBOs, redrawn per-entity with just a model matrix + color — never
  regenerated per-entity or per-frame.
- Winding convention: triangles wind so `cross(edge1,edge2)` matches the
  stored normal (CCW-from-outside). Doesn't matter yet (culling is off)
  but will the moment culling is enabled. The plane's winding was found
  backwards and fixed; cube and sphere were already correct.

## Input

- `Input` owns raw keyboard/mouse state + cursor-capture mode
  (`glfwSetInputMode`); `Camera` owns *when* to use it — captures only
  while the right mouse button is held, so the cursor stays free
  otherwise. Mechanism vs. policy, kept separate on purpose.
- Polling-based (`glfwGetKey` etc. every frame), not GLFW callbacks —
  simpler, no C-callback-to-C++-object trampoline.
- `WasKeyPressed` = this frame's state true, last frame's false — fires
  once per press, not once per held frame.
- On `SetCursorCaptured(true)`, the mouse position is immediately re-read
  and the pending delta zeroed — otherwise the frame capture begins on
  would report a jump from wherever the cursor drifted while free.

## Debug Drawing

- `DebugDraw` is pure data — `Line`/`Box`/`Sphere` just append to a
  buffer, zero OpenGL dependency. Lives in `utils/`, not `rendering/`, so
  non-rendering code can queue debug visuals without depending on the
  rendering module.
- `Renderer` drains the buffer once per frame (`TakeLines()`) and is the
  only thing that turns it into draw calls.
- `Box` → 12 edges from 8 corners. `Sphere` → 3 orthogonal great-circle
  rings (XY/XZ/YZ), each built from straight segments around a parametric
  circle.