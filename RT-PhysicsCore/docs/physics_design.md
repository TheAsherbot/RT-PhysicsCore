# RT-PhysicsCore — Physics

Design decisions and the formulas behind them: rigid body state,
integration, collision detection, contact resolution, friction and
restitution, materials. Engine/ECS/rendering live in `engine_core.md`.

## Rigid Body State

- `RigidBodyComponent`: `mass`/`invMass` (`invMass == 0` = static/
  kinematic), `inertiaBody`/`invInertiaBody` (body-space, about the
  center of mass), `velocity`/`forceAccum`, `angularMomentum`/
  `torqueAccum`. Force/torque accumulators are cleared every
  `PhysicsSystem::FixedUpdate`.
- Position/orientation are not duplicated here — `TransformComponent`
  already owns both; `PhysicsSystem` integrates directly into it.
- Linear state is velocity, not momentum — with constant mass the two
  are always trivially interchangeable, so there's no accuracy reason to
  prefer momentum.
- Angular state is angular momentum (L), not angular velocity (ω). A body's
  world-space inertia isn't constant (it rotates with the body:
  `invInertiaWorld = R * invInertiaBody * R^T`). Angular momentum is
  what `dL/dt = torque` actually conserves when torque is zero; angular
  velocity is not conserved for any asymmetric body.
- Caches: `angularVelocity` and `invInertiaWorld` are cached on the component
  for consumers like `ResolutionSystem`. They are computed from the **final**
  orientation of the step so downstream contact solvers see up-to-date,
  synchronized inertia data rather than one-step-stale values.

## Mass Properties (`MassProperties.h/.cpp`)

- All formulas are body-space, about the shape's own center of mass,
  uniform density, diagonal (shapes are defined aligned to their own
  principal axes).
- Box (half-extents `hx,hy,hz`):
  ```
  Ixx = (m/3)(hy^2+hz^2)   Iyy = (m/3)(hx^2+hz^2)   Izz = (m/3)(hx^2+hy^2)
  ```
- Sphere (radius `r`): `I = (2/5) m r^2`, isotropic.
- Cylinder (radius `r`, full height `H`, axis local +Y):
  ```
  axial (Y) = 0.5 m r^2
  perpendicular (X,Z) = (m/12)(3r^2 + H^2)
  ```
- Capsule (radius `r`, cylindrical half-length `halfLength`, axis local
  +Y): decomposed into a cylinder + two hemispherical caps, mass split
  between them by volume, hemisphere terms shifted to the capsule's
  center via the parallel-axis theorem (a solid hemisphere's own
  centroid sits `3r/8` from its flat face):
  ```
  H = 2*halfLength
  m_cyl  = m * (r^2*H)        / (r^2*H + (4/3)r^3)
  m_hemi = m * ((2/3)r^3)     / (r^2*H + (4/3)r^3)     (each of the two caps)
  axial         = 0.5*m_cyl*r^2 + (4/5)*m_hemi*r^2
  perpendicular = m_cyl*(3r^2+H^2)/12 + 2*m_hemi*(0.4r^2 + 0.25H^2 + 0.375*H*r)
  ```
- `MakeDynamicBody(mass, inertiaBody)` computes `invMass`/
  `invInertiaBody` once (guards `mass <= 0` by falling back to a static
  body, logged). `MakeStaticBody()` sets `invMass = 0`, `invInertiaBody`
  = the zero matrix.

## Integration (`PhysicsSystem`)

- Fixed timestep only (see `engine_core.md`'s `FixedTimestep`) —
  Euler-family integrators assume a known, constant `dt`.
- Gravity defaults to `(0, -9.80665, 0)` m/s^2, configurable via
  `SetGravity`. Applied as a force, `forceAccum += mass * gravity`,
  before dividing by mass — that division is what makes every mass fall
  at the same rate. Contributes no torque (acts uniformly through the
  volume).
- Static bodies (`invMass <= 0`) are skipped entirely; their force/torque
  accumulators are still cleared so nothing carries over if they ever
  become dynamic.
- Linear: semi-implicit (symplectic) Euler —
  ```
  velocity += (forceAccum * invMass) * dt
  position += velocity * dt          // uses the just-updated velocity
  ```
  Updating velocity first and using the new value for position (not the
  reverse) is what makes this symplectic rather than explicit Euler — it
  keeps long-running resting/oscillating contacts from slowly gaining
  energy.
-  Angular: DLM Symplectic Splitting (Dullweber, Leimkuhler, McLachlan 1997) —
  Explicit Euler updates (`q += 0.5 * dt * ω * q`) evaluate ω once per step while
  orientation and inertia are coupled. This artificially pumps rotational kinetic
  energy into the system (+769% over 60s), driving asymmetric bodies to settle
  into their minimum-inertia axis and completely destroying intermediate-axis
  tumbling (the Dzhanibekov / tennis-racket effect).
  The engine instead uses the second-order, time-reversible DLM symplectic
  splitting algorithm in the body frame:
  1. External torque kick on world angular momentum:
     `angularMomentum += torqueAccum * dt`
  2. Transform momentum into the body frame:
     `bodyMomentum = R^T * angularMomentum`
  3. Perform a 5-step Strang splitting across principal axes:
     `R1(h/2) ∘ R2(h/2) ∘ R3(h) ∘ R2(h/2) ∘ R1(h/2)`
     For each step along axis `k` with duration `tau`:
     - Rotation angle: `theta = tau * bodyMomentum[k] * invInertiaBody[k][k]`
     - Compose local rotation: `q = normalize(q * quat(axis k, theta))`
     - Rotate body momentum: `bodyMomentum = rotate_axis(bodyMomentum, k, -theta)`
  4. Synchronize derived caches from the final step orientation:
     ```
     invInertiaWorld = R_final * invInertiaBody * R_final^T
     angularVelocity = invInertiaWorld * angularMomentum
     ```
  This preserves the phase space Hamiltonian, bounds energy drift to <0.01%,
  conserves |L| exactly, and reproduces perpetual intermediate-axis tumbling.

## Collision Detection — Broad Phase (`AABB.h/.cpp`, `CollisionSystem`)

- `ColliderComponent`: `shape` (Box/Sphere/Capsule), `size` (meaning
  depends on shape — Box: half-extents; Sphere: radius = `size.x`;
  Capsule: radius = `size.x`, cylindrical half-length = `size.y`, axis =
  local +Y), `offset` (local-space center offset from the entity's
  `TransformComponent::position` — see Collider Placement below).
- World AABB for a Box: rotate-then-reproject, not the raw local
  half-extents —
  ```
  extent = |R_col0|*hx + |R_col1|*hy + |R_col2|*hz    (per world axis)
  ```
  Skipping the `abs()` (or using the un-rotated half-extents directly)
  gives an AABB that's too small for a rotated box and misses real
  overlaps.
- Sphere AABB: `center ± radius` (rotation-invariant). Capsule AABB:
  union of the two cap-sphere AABBs.
- `CollisionSystem::FixedUpdate` runs a naive O(n^2) pairwise AABB test
  each step (every surviving pair goes to narrow phase). Correct but not
  scalable — swap in sweep-and-prune if body counts ever make this the
  bottleneck; it changes only how fast non-colliding pairs get rejected,
  not which pairs end up colliding.

## Collision Detection — Narrow Phase (`NarrowPhase.h/.cpp`)

- `TestCollision(a, b)` dispatches to one of six shape-pair functions and
  fills a `Contact`. Convention: `Contact::normal` always points from
  `a` toward `b` — each pair function is written for one fixed argument
  order and the dispatcher flips the normal when it has to swap them.
- Sphere-Sphere: distance test; contact point is the midpoint of the two
  surface points; `penetration = radiusSum - distance`.
- Sphere-Box: clamp the sphere center into the box's local frame. If
  clamping didn't move it (center already inside the box), push out
  toward whichever face is closest instead. Known gap: this "nearest
  face" choice assumes normal-speed entry — if a single fixed step moves
  the sphere far enough that it samples past the box's midpoint, this
  picks the wrong face and pushes the wrong direction (see the CCD note
  below).
- Sphere-Capsule: closest point on the capsule's segment, then
  Sphere-Sphere against that point.
- Capsule-Capsule: general case is closest-points-between-two-segments
  (Ericson, *Real-Time Collision Detection* 5.1.9), then Sphere-Sphere.
  If the two segments are near-parallel (`|dot(dirA,dirB)| > 0.999`),
  generates two contact points spanning the overlapping interval instead
  of one — a single point lets a capsule resting along another roll.
- Box-Capsule: alternating projection (segment → box surface → segment,
  2 iterations) finds the closest point on the capsule's axis to the
  box. If that axis lies roughly in the contact plane (within ~11° of
  perpendicular to the normal — resting along the face, not just
  touching near one end), replaces the single point with two spanning
  wherever the axis actually overlaps the box's projected extent along
  that direction (same interval idea as Capsule-Capsule, using the box's
  SAT-style projected half-width).
- Box-Box: exact 3D SAT — 15 candidate axes (6 face normals + 9
  edge-edge cross products), tracking the axis of minimum overlap:
  ```
  axisHalfWidth = h.x*|dot(axis,right)| + h.y*|dot(axis,up)| + h.z*|dot(axis,forward)|
  overlap = (halfWidthA + halfWidthB) - |dot(centerB-centerA, axis)|
  ```
  Any axis with `overlap < 0` means separated, no collision. If the
  winning axis is a face axis (index < 6): the incident box's nearest
  face is clipped against the reference face's 4 side planes
  (Sutherland-Hodgman) to produce up to 8 contact points, each with its
  own penetration depth — what lets a box rest flat without rocking (a
  single point can't resist torque). If the winning axis is an edge-edge
  axis: the two specific edges involved are rebuilt as segments and
  their closest points (same Ericson routine as above) become the one
  contact point.
- No continuous collision detection (CCD): everything above is discrete,
  sampled once per fixed step. A fast body relative to a thin collider
  can tunnel through with no sampled overlap at all, or — worse — land
  with its sampled position already past the collider's midpoint and get
  pushed the wrong way by the nearest-face heuristics above. Static
  colliders are given extra thickness as a partial, non-CCD mitigation
  (see Collider Placement).

## Contact Resolution (`ResolutionSystem`)

- Two independent, runtime-switchable choices: `SolverMode`
  (`SequentialImpulses`, the default, or `Exact`) and `IterationMode`
  (`Fixed`, the default, or `Adaptive`, which keeps iterating within a
  time budget instead of a fixed count). Defaults: 8 velocity iterations
  (4-20 if adaptive, 1.0 ms budget), 3 position iterations (1-8, 0.3 ms
  budget).
- Effective mass along an axis at a contact point:
  ```
  K = invMassA + invMassB
    + dot(axis, cross(invInertiaWorldA * cross(rA, axis), rA))
    + dot(axis, cross(invInertiaWorldB * cross(rB, axis), rB))
  ```
  (`rA`/`rB` = contact point minus each body's `TransformComponent::position`.)
  The rotational terms are what let an off-center impulse correctly
  impart spin instead of pure translation.
- Sequential impulses: for every contact point, every iteration, solve
  the normal impulse then immediately the friction impulse (same point,
  same pass):
  ```
  lambda = -(relVelN + restitutionBias) / K
  accumNormal = max(0, accumNormal + lambda)   // clamp the RUNNING TOTAL
  ```
  Clamping the running total (not each increment) to `>= 0` — a contact
  can only push — is what lets an earlier iteration's overshoot get
  partially undone by a later one instead of getting stuck.
- Restitution bias is captured once, at the very start of the step, from
  the initial relative velocity (`v_rel_n` before any impulses this
  step): `restitutionBias = e * v_rel_n_initial`, but only when
  `v_rel_n_initial < -0.5` (`restitutionVelocityThreshold`) — otherwise
  treated as `e = 0`. Without that threshold a resting body's tiny
  per-step gravity-drift closing velocity would read as a bounce and it
  would never actually settle.
- Friction solves both tangent directions (`t1`,`t2`) jointly, not
  independently, via a 2x2 system (Cramer's rule):
  ```
  Kt1t2 = dot(t1, cross(invInertiaWorldA * cross(rA,t2), rA)) + (same for B)
  det = Kt1t1*Kt2t2 - Kt1t2^2
  deltaT1 = (-vt1*Kt2t2 + Kt1t2*vt2) / det
  deltaT2 = (-Kt1t1*vt2 + vt1*Kt1t2) / det
  ```
  then the combined `(accumT1, accumT2)` vector is clamped to a disc of
  radius `frictionCoeff * accumNormal` — the true Coulomb cone, not the
  cheaper square-pyramid approximation (independent per-axis clamps),
  chosen because the sim sends objects in arbitrary directions where the
  pyramid over-grips on the diagonals. Falls back to independent axes
  only if the 2x2 system is near-singular.
  - Tangent basis `t1`/`t2`: fixed, derived from the normal alone (not
    velocity-aligned), so it stays well-defined at zero sliding speed —
    needed since static friction has to be well-defined exactly there.
  - Friction coefficient (static vs. kinetic) is chosen once per point
    per step, from the initial tangential speed vs.
    `frictionVelocityThreshold = 0.01` — not re-decided every iteration,
    which would cause visible chattering between the two regimes.
- Exact mode: builds one Linear Complementarity Problem across every
  contact point in the step at once (`w = Mz + q`, `w>=0`, `z>=0`,
  `z.w=0`) and solves it with Lemke's algorithm (`LCPSolver.cpp`)
  instead of iterating:
  ```
  M[i][j] = effect on point i's relative normal velocity from a unit
            impulse at point j (nonzero only if the two points share a
            body; M[i][i] reduces to exactly the K above)
  q[i]    = (1 + e) * v_rel_n_initial            (same resting-contact guard)
  ```
  Normal impulses (the LCP's `z`) are all applied simultaneously, not
  one at a time, so point ordering can't bias the result. Friction still
  runs as a separate iterative pass afterward, using those now-fixed
  normal impulses as the bound — jointly solving normal-and-friction
  "exactly" is a genuinely harder, nonlinear problem this doesn't
  attempt. Falls back to sequential impulses (logged) if Lemke's
  algorithm doesn't converge within the pivot cap (default `4n+50`).
- Lemke's algorithm: tableau/pivot method with a unit covering vector,
  Bland's-rule tie-breaking in the minimum-ratio test (provably
  cycle-free), tableau arithmetic done in `double` even though the
  public API is `float` — Lemke's algorithm chains many pivots and is
  documented as numerically sensitive; `float`'s ~7 digits isn't much
  margin for that much compounding rounding.
- Position correction is a fully separate pass from the velocity solve
  (split impulse) and never touches `velocity`/`angularMomentum` — a
  bias baked into the velocity solve (Baumgarte) would inject real
  kinetic energy; nudging position/orientation directly can't. Tracks a
  running `separation` estimate per point (starts at `-penetration`,
  incremented by each applied correction) instead of re-running narrow
  phase every iteration:
  ```
  correction = clamp(0.2 * (-separation - 0.005), 0, 0.2)   // beta, slop, max-per-iteration
  push = (correction / K) * normal
  ```
  applied to both position and orientation (reusing the cached
  `invInertiaWorld` rather than rebuilding it from the very slightly
  shifting orientation each iteration — a small, deliberate
  approximation).
- No cross-frame warm-starting: accumulated impulses reset to zero every
  step. Real warm-starting needs a stable identity for "this frame's
  contact point" vs. "last frame's" (feature IDs — which vertex/edge/
  face pair generated it), which the narrow phase doesn't produce today.
  Costs convergence speed in tightly-coupled scenes (stacks, many
  simultaneous contacts), not correctness.

## Materials (`PhysicsMaterial.h/.cpp`, `PhysicsMaterialComponent`)

- `PhysicsMaterialComponent` is optional — `CollisionSystem` treats a
  missing one as `MaterialId::Default`. Only one call site ever reads it
  (`CollisionSystem`, once per contact), so there's no reason to force
  it onto every entity the way `RigidBodyComponent` is.
- `GetPairProperties(a, b)` checks an explicit pair table first (order
  doesn't matter — the pair is canonicalized before lookup). Falls back
  to combining each material's solo properties: geometric mean for both
  frictions, arithmetic mean for restitution. No combination rule is
  physically exact for a real material pair — these are deliberate,
  reasonable defaults, not derived results.
- Combined values are computed once per contact, in `CollisionSystem`,
  and stored directly on `Contact` (`restitution`, `staticFriction`,
  `kineticFriction`) — `ResolutionSystem` never touches
  `PhysicsMaterialComponent` or the material tables at all.
- Eight built-in materials are defined in `SoloProperties()`:
  - `Default`: restitution 0.3, static friction 0.6, kinetic friction 0.4
  - `Clay`: restitution 0.0, static friction 0.8, kinetic friction 0.6 (heavy, zero bounce)
  - `Wood`: restitution 0.25, static friction 0.5, kinetic friction 0.4 (slight bounce)
  - `Rubber`: restitution 0.5, static friction 0.8, kinetic friction 0.6 (bouncy)
  - `HardRubber`: restitution 0.75, static friction 0.8, kinetic friction 0.6 (high bounce)
  - `SuperBall`: restitution 1.0, static friction 0.6, kinetic friction 0.4 (perfect restitution)
  - `BouncyIce`: restitution 1.0, static friction 0.0, kinetic friction 0.0 (frictionless bounce for Newton's Cradle)
  - `Domino`: restitution 0.1, static friction 0.8, kinetic friction 0.6 (high grip, low bounce for stable chain reaction)

## Collider Placement

- `ColliderComponent::offset` lets the collision volume's center differ
  from the entity's `TransformComponent::position` (rotated into world
  space: `worldCenter = position + rotation * offset`). Applied in both
  `AABB.h`'s `ComputeWorldAABB` and `CollisionSystem`'s narrow-phase
  pose — narrow phase itself never needs to know an offset exists, it
  just sees an already-adjusted world center.
- Existing use: a thin visual ground plane has no real thickness to give
  its collider, and a paper-thin collider tunnels easily. The ground's
  collider keeps a fixed, generous Y half-extent independent of the
  visual scale, offset downward by exactly that half-extent so the top
  face still lands at the visible surface while the bulk of the slab
  sits invisibly underground, absorbing the tunneling risk a thin
  collider wouldn't.
- Resolution's `rA`/`rB` (offset from contact point to each body's
  center of mass) still measure from the entity's actual
  `TransformComponent::position`, not the offset collider center — the
  offset moves the collision shape, not where the body's mass actually
  is. If `offset` is ever used on a dynamic body, the inertia tensor
  would need the parallel-axis theorem applied to stay accurate (not a
  concern for a static body, which has none).