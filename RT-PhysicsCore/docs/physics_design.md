# RT-PhysicsCore — Physics

Design decisions, numerical methods, and mathematical formulations behind RT-PhysicsCore: high-precision math, rigid body state, analytical mass properties, substepping pipeline, numerical integration, collision detection, continuous collision detection (CCD), persistent contact caching, island partitioning, constraint solvers, materials, and collider placement. Engine, ECS, and rendering details live in `engine_core.md`.

---

## 1. High-Precision Math Layer (`MathTypes.h/.cpp`)

To prevent truncation, rounding drift, and ill-conditioned constraint matrices during multi-contact and LCP solving, physics dynamics and geometric algorithms execute in double precision (`double`):

- **`Real`**: Scalar floating-point type mapped to `double`.
- **`Vec3`**: High-precision 3D vector `{ Real x, y, z; }` providing bidirectional implicit conversion to and from single-precision `glm::vec3` and double-precision `glm::dvec3`.
- **`Quat`**: High-precision orientation quaternion `{ Real w, x, y, z; }` with bidirectional conversion to and from `glm::quat` and `glm::dquat`.
- **`Mat3`**: High-precision 3x3 matrix providing bidirectional conversion to and from `glm::mat3` and `glm::dmat3`.
- **Mixed-Precision Operators**: Overloaded operators allow direct multiplication and linear combination between `Real` and `float`/`glm::vec3`/`glm::mat3` without explicit casting.

---

## 2. Rigid Body State (`RigidBodyComponent.h`)

- `RigidBodyComponent`: `mass`/`invMass` (`invMass == 0` = static/kinematic), `inertiaBody`/`invInertiaBody` (body-space, about center of mass, `Mat3`), `velocity`/`forceAccum` (`Vec3`), `angularMomentum`/`torqueAccum` (`Vec3`). Force and torque accumulators are cleared every substep.
- `pseudoVelocity` and `pseudoAngularVelocity` (`Vec3`): Dedicated linear and angular channels for split-impulse position correction. Resolves overlap by directly advancing position and orientation at the end of the step without altering `velocity` or `angularMomentum`, preventing artificial kinetic energy injection (Baumgarte stabilization).
- `motionFraction` (`Real`, $[0, 1]$): Fraction of the substep time budget traversed before continuous collision impact. Used by conservative advancement to prevent interpenetration.
- Position and orientation are not duplicated on the body component — `TransformComponent` owns both; `PhysicsSystem` integrates directly into it.
- Linear state is velocity, not momentum — with constant mass the two are always trivially interchangeable, so there is no accuracy reason to prefer momentum.
- Angular state is angular momentum ($\mathbf{L}$), not angular velocity ($\boldsymbol{\omega}$). A body's world-space inertia is not constant (it rotates with the body: $\mathbf{I}_{\text{world}}^{-1} = \mathbf{R} \mathbf{I}_{\text{body}}^{-1} \mathbf{R}^T$). Angular momentum is what $d\mathbf{L}/dt = \boldsymbol{\tau}$ actually conserves when torque is zero; angular velocity is not conserved for any asymmetric body.
- Caches: `angularVelocity` and `invInertiaWorld` are cached on the component for consumers like `ResolutionSystem`. They are computed from the **final** orientation of the step so downstream contact solvers see up-to-date, synchronized inertia data rather than one-step-stale values.

---

## 3. Mass Properties (`MassProperties.h/.cpp`)

- All formulas are body-space, about the shape's own center of mass, uniform density, diagonal (shapes are defined aligned to their own principal axes).
- **Box** (half-extents $h_x, h_y, h_z$):
  ```
  Ixx = (m / 3) * (hy^2 + hz^2)
  Iyy = (m / 3) * (hx^2 + hz^2)
  Izz = (m / 3) * (hx^2 + hy^2)
  ```
- **Sphere** (radius $r$): isotropic matrix with diagonal entries $I = \frac{2}{5} m r^2$.
- **Cylinder** (radius $r$, full height $H$, axis local $+Y$):
  ```
  axial (Y)         = 0.5 * m * r^2
  perpendicular (X,Z) = (m / 12) * (3 * r^2 + H^2)
  ```
- **Capsule** (radius $r$, cylindrical half-length $\text{halfLength}$, axis local $+Y$): decomposed into a cylinder of height $H = 2 \cdot \text{halfLength}$ plus two hemispherical caps. Mass is split between them by volume ($V_{\text{cyl}} = \pi r^2 H$, $V_{\text{hemi}} = \frac{2}{3} \pi r^3$). Hemisphere terms are shifted to the capsule center via the parallel-axis theorem (a solid hemisphere's centroid sits $\frac{3}{8}r$ from its flat face):
  ```
  H = 2 * halfLength
  m_cyl  = m * (r^2 * H)     / (r^2 * H + (4/3) * r^3)
  m_hemi = m * ((2/3) * r^3) / (r^2 * H + (4/3) * r^3)   (each cap)

  axial         = 0.5 * m_cyl * r^2 + (4/5) * m_hemi * r^2
  perpendicular = m_cyl * (3 * r^2 + H^2) / 12 + 2 * m_hemi * (0.4 * r^2 + 0.25 * H^2 + 0.375 * H * r)
  ```
- `MakeDynamicBody(mass, inertiaBody)` computes `invMass` and `invInertiaBody = glm::inverse(inertiaBody)` (guards `mass <= 0` by falling back to a static body, logged). `MakeStaticBody()` sets `invMass = 0` and `invInertiaBody` to the zero matrix.

---

## 4. Master Substepping Loop (`PhysicsPipelineSystem.h/.cpp`)

To maintain high simulation stability, stiff contact resolution, and robust continuous collision detection, the engine subdivides every fixed engine tick $\Delta t$ into $N$ discrete sub-steps (default: $N = 4$, yielding substep duration $h = \Delta t / N$):

1. **Force Integration**: Applies gravity and external forces to update velocities: $\mathbf{v}^* = \mathbf{v} + h \mathbf{M}^{-1} \mathbf{F}_{\text{ext}}$.
2. **Swept Broad-Phase**: Generates expanded AABBs swollen by swept velocity bounds ($\mathbf{v}^* h$) to catch all potential collisions.
3. **Narrow-Phase & Speculative Contacts**: Detects discrete contacts and generates speculative contacts for surfaces within `speculativeMargin`. Sutherland–Hodgman contact faces are reduced to 4 extremal support points.
4. **Continuous Collision Detection (CCD)**: Bilateral conservative advancement calculates time-of-impact (TOI) and clamps `motionFraction` for fast-moving bodies to prevent tunneling.
5. **Persistent Contact Caching & Warm-Starting**: Matches current contacts against the persistent cache (`ContactCache`) using canonical body keys and local anchor points. Historical normal and friction impulses are re-applied as initial solver guesses.
6. **Island Partitioning**: Groups connected bodies and contact manifolds into disjoint simulation islands using Union-Find (`IslandBuilder`).
7. **Constraint Resolution**: Dispatches each island to the active solver backend (`IterativeSolver` or `ExactSolver`) to solve normal impulses, Coulomb friction, and split-impulse position correction.
8. **Position & Pseudo-Velocity Integration**: Integrates positions and DLM rotational splitting using actual velocities multiplied by `motionFraction * h`, followed by pseudo-velocity positional correction.

---

## 5. Numerical Integration (`PhysicsSystem.h/.cpp`)

- Fixed timestep only (see `engine_core.md`'s `FixedTimestep`) — Euler-family integrators assume a known, constant $h$.
- Gravity defaults to $(0, -9.80665, 0)\,\text{m/s}^2$, configurable via `SetGravity`. Applied as a force: $\mathbf{F}_{\text{accum}} += m \mathbf{g}$. Division by mass in the integrator ensures all masses accelerate uniformly. Contributes zero torque (acts uniformly through the volume).
- Static bodies (`invMass <= 0`) are skipped entirely; their force and torque accumulators are cleared so nothing carries over if they ever become dynamic.
- **Linear**: Semi-implicit (symplectic) Euler:
  ```
  velocity += (forceAccum * invMass) * h
  position += velocity * (motionFraction * h)
  ```
  Updating velocity first and using the new value for position is what makes this symplectic rather than explicit Euler — it bounds energy drift and keeps long-running resting contacts from gaining energy.
- **Angular: DLM Symplectic Splitting** (Dullweber, Leimkuhler, McLachlan 1997):
  Explicit Euler updates ($\mathbf{q} += 0.5 h \boldsymbol{\omega} \mathbf{q}$) evaluate $\boldsymbol{\omega}$ once per step while orientation and inertia are coupled. This artificially injects rotational kinetic energy (+769% over 60s in an asymmetric body), driving bodies into their minimum-inertia axis and destroying intermediate-axis tumbling (the Dzhanibekov / tennis-racket effect).
  The engine uses the second-order, time-reversible DLM symplectic splitting algorithm in the body frame:
  1. External torque kick on world angular momentum:
     $$\mathbf{L} += \boldsymbol{\tau}_{\text{accum}} h$$
  2. Transform momentum into the body frame:
     $$\mathbf{L}_{\text{body}} = \mathbf{R}^T \mathbf{L}$$
  3. Perform a 5-step Strang splitting across principal axes:
     $$R_1(h/2) \circ R_2(h/2) \circ R_3(h) \circ R_2(h/2) \circ R_1(h/2)$$
     For each step along axis $k \in \{0, 1, 2\}$ with duration $\tau$:
     - Rotation angle: $\theta = \tau L_{\text{body}}[k] \mathbf{I}_{\text{body}}^{-1}[k][k]$
     - Compose local rotation: $\mathbf{q} = \text{normalize}(\mathbf{q} \cdot \text{quat}(\text{axis}_k, \theta))$
     - Rotate body momentum: $\mathbf{L}_{\text{body}} = \text{rotate\_axis}(\mathbf{L}_{\text{body}}, k, -\theta)$
  4. Synchronize derived caches from the final step orientation:
     ```
     invInertiaWorld = R_final * invInertiaBody * R_final^T
     angularVelocity = invInertiaWorld * angularMomentum
     ```
  This preserves the phase space Hamiltonian, bounds energy drift to $<0.01\%$, conserves $\|\mathbf{L}\|$ exactly, and reproduces perpetual intermediate-axis tumbling.

---

## 6. Collision Detection — Broad Phase (`AABB.h/.cpp`, `CollisionSystem.h/.cpp`)

- `ColliderComponent`: `shape` (Box/Sphere/Capsule), `size` (Box: half-extents; Sphere: radius = `size.x`; Capsule: radius = `size.x`, cylindrical half-length = `size.y`, axis = local $+Y$), `offset` (local-space center offset from `TransformComponent::position`).
- **World AABB for a Box**: Rotate-then-reproject using absolute rotation matrix columns:
  ```
  extent = |R_col0| * hx + |R_col1| * hy + |R_col2| * hz    (per world axis)
  ```
  Skipping the `abs()` (or using un-rotated extents) gives an AABB that is too small for rotated boxes and misses overlaps.
- **Sphere AABB**: $\text{center} \pm r$ (rotation-invariant).
- **Capsule AABB**: Union of the two cap-sphere AABBs.
- **Swept Bounds**: During substepping, each body's AABB is expanded by its displacement vector $\mathbf{v} h$ plus a small speculative padding margin. This broad-phase expansion ensures fast-moving bodies are never culled prior to narrow-phase and continuous collision tests.

---

## 7. Collision Detection — Narrow Phase (`NarrowPhase.h/.cpp`, `ManifoldReduction.h/.cpp`)

- `TestCollision(a, b)` dispatches to one of six primitive pair functions and fills a `ContactManifold`. Convention: `normal` points from body $A$ toward body $B$.
- **Sphere–Sphere**: Distance test; contact point is the midpoint of the two surface points; $\text{penetration} = (r_A + r_B) - \text{distance}$.
- **Sphere–Box**: Clamp the sphere center into the box's local frame. If clamping did not move it (center already inside the box), push out toward whichever face is closest.
- **Sphere–Capsule**: Closest point on the capsule's line segment, followed by Sphere–Sphere against that point.
- **Capsule–Capsule**: Closest points between two line segments (Ericson, *Real-Time Collision Detection* 5.1.9), then Sphere–Sphere. If segments are near-parallel ($|\mathbf{d}_A \cdot \mathbf{d}_B| > 0.999$), generates two contact points spanning the overlapping interval instead of one, preventing rolling.
- **Box–Capsule**: Alternating projection (segment $\to$ box surface $\to$ segment, 2 iterations) finds the closest point on the capsule axis to the box. If the axis lies roughly in the contact plane (within $\sim 11^\circ$ of perpendicular to the normal), generates two contact points across the projected overlap interval.
- **Box–Box**: Exact 3D Separating Axis Theorem (SAT) across 15 candidate axes (6 face normals + 9 edge-edge cross products), tracking the axis of minimum overlap:
  ```
  axisHalfWidth = h.x * |dot(axis, right)| + h.y * |dot(axis, up)| + h.z * |dot(axis, forward)|
  overlap = (halfWidthA + halfWidthB) - |dot(centerB - centerA, axis)|
  ```
  Any axis with $\text{overlap} < 0$ indicates separation. If the winning axis is a face axis, the incident face is clipped against the reference face's 4 side planes (Sutherland–Hodgman), generating up to 8 candidate contact points. If an edge-edge axis wins, the two participating edges are reconstructed as line segments and their closest points form a single contact.
- **Extremal 4-Point Manifold Reduction (`ManifoldReduction.h/.cpp`)**:
  While face clipping produces up to 8 polygon vertices, solving more than 4 coplanar points adds redundant constraints, increases solver cost, and leads to ill-conditioned LCP matrices. `ManifoldReduction::Reduce` selects at most 4 extremal points (`kMaxContactPoints = 4`) that maximize the enclosed polygon area:
  1. Pick the deepest penetration point.
  2. Pick the point furthest from point 1.
  3. Pick the point furthest from the line between points 1 and 2 (maximizing triangle area).
  4. Pick the point on the opposite side of the line that maximizes total quadrilateral area.
- **Speculative Contacts**:
  When two bodies approach within `speculativeMargin`, a contact point is generated with negative penetration depth ($\text{penetration} = -\text{distance}$). The constraint solver uses this distance to brake the approaching body before penetration occurs.

---

## 8. Continuous Collision Detection (`ContinuousCollisionSystem.h/.cpp`, `ShapeCast.h/.cpp`)

Discrete collision sampling can allow fast bodies to tunnel through geometry or sample past midpoints. The engine implements Continuous Collision Detection (CCD) via **Bilateral Conservative Advancement**:

1. **Convex Distance Queries**: Gilbert–Johnson–Keerthi (GJK) evaluates the exact signed separation distance $d$ between convex geometries.
2. **Time-of-Impact (TOI) Iteration**: Given relative velocity $\mathbf{v}_{\text{rel}}$, the maximum safe advancement step is $\Delta \tau = (d - \text{skinGap}) / (\mathbf{v}_{\text{rel}} \cdot (-\mathbf{n}))$.
3. **Motion Clamping**: Iterates until separation reaches `skinGap` or total substep time $h$ is consumed. If impact occurs at fraction $\alpha \in [0, 1)$, both bodies have their `motionFraction` clamped to $\alpha$. The solver then resolves contact impulses at the exact point of contact without interpenetration.

---

## 9. Persistent Contact Caching & Warm-Starting (`ContactCache.h/.cpp`)

- Multi-contact stacks require several iterations to propagate support forces from the ground upward. Cold-starting every frame causes stacked objects to bounce and drift.
- `ContactCache` maintains contact manifolds across substeps indexed by a canonical 64-bit pair key `(min(EntityA, EntityB), max(EntityA, EntityB))`.
- When a new manifold is produced, points are matched against cached points by comparing local anchor positions ($\mathbf{r}_A, \mathbf{r}_B$) and verifying normal alignment ($\mathbf{n}_{\text{new}} \cdot \mathbf{n}_{\text{old}} > 0.9$).
- Matched points inherit their previous normal and friction impulses. Seeding these warm-start impulses directly into the solver accelerates convergence from 8–10 iterations down to 2–3 iterations.

---

## 10. Simulation Island Partitioning (`IslandBuilder.h/.cpp`)

- Global solver loops over an entire scene suffer poor cache locality and cannot easily parallelize.
- `IslandBuilder` uses a disjoint-set (Union-Find) algorithm to partition active rigid bodies and contact manifolds into isolated simulation islands:
  - Dynamic bodies connected by active contacts are merged into the same island.
  - Immovable static bodies (`invMass == 0`) act as island boundaries and never merge islands together.
- Solvers execute independently per island. Sleeping islands (where all bodies have velocity below sleep thresholds) can be skipped entirely.

---

## 11. Constraint Resolution Architecture (`ResolutionSystem.h/.cpp`, `ISolverBackend.h`)

Contact constraints are decoupled from the system lifecycle through the `ISolverBackend` interface. Two backends are available at runtime:

### Effective Mass Formulation
For every contact point, the effective mass $K$ along an arbitrary unit axis $\mathbf{u}$ is:
$$K = m_A^{-1} + m_B^{-1} + \mathbf{u} \cdot (\mathbf{I}_A^{-1}(\mathbf{r}_A \times \mathbf{u}) \times \mathbf{r}_A) + \mathbf{u} \cdot (\mathbf{I}_B^{-1}(\mathbf{r}_B \times \mathbf{u}) \times \mathbf{r}_B)$$
where $\mathbf{r}_A, \mathbf{r}_B$ are the vectors from each body's center of mass to the contact point.

### Backend 1: Iterative Solver (Projected Gauss-Seidel) (`IterativeSolver.h/.cpp`)
- **Normal Impulse**: For each contact point, calculates the impulse increment:
  $$\Delta \lambda_n = -\frac{v_{\text{rel}, n} + \text{restitutionBias}}{K_n}$$
  Clamps the **accumulated running total**: $\lambda_n = \max(0, \lambda_n + \Delta \lambda_n)$. Clamping the running total allows earlier iteration overshoots to be undone.
- **Restitution Bias**: Evaluated once at the start of the substep: $\text{bias} = e \cdot v_{\text{rel}, n}$, active only when $v_{\text{rel}, n} < -0.5\,\text{m/s}$ (`restitutionVelocityThreshold`). Below this threshold, $\text{bias} = 0$, preventing resting jitter under gravity.
- **Coupled 2D Friction**: Friction is solved jointly across both tangent vectors ($\mathbf{t}_1, \mathbf{t}_2$) via a $2 \times 2$ system (Cramer's rule):
  $$K_{t1t2} = \mathbf{t}_1 \cdot (\mathbf{I}_A^{-1}(\mathbf{r}_A \times \mathbf{t}_2) \times \mathbf{r}_A) + \mathbf{t}_1 \cdot (\mathbf{I}_B^{-1}(\mathbf{r}_B \times \mathbf{t}_2) \times \mathbf{r}_B)$$
  $$\text{det} = K_{t1t1} K_{t2t2} - K_{t1t2}^2$$
  $$\Delta \lambda_{t1} = \frac{-v_{t1} K_{t2t2} + K_{t1t2} v_{t2}}{\text{det}}, \quad \Delta \lambda_{t2} = \frac{-K_{t1t1} v_{t2} + v_{t1} K_{t1t2}}{\text{det}}$$
  The accumulated vector $(\lambda_{t1}, \lambda_{t2})$ is clamped to a circular disc of radius $\mu \lambda_n$ (the true Coulomb cone, avoiding diagonal over-grip from square-pyramid approximations).

### Backend 2: Exact Solver (Lemke LCP) (`ExactSolver.h/.cpp`, `LCPSolver.h/.cpp`)
- Constructs the coupled Delassus mobility matrix $\mathbf{M} \in \mathbb{R}^{m \times m}$ across all $m$ contact points in an island simultaneously:
  $$M_{ij} = \mathbf{n}_i^T \left[ (\delta_{A_i A_j} m_{A_i}^{-1} + \delta_{B_i B_j} m_{B_i}^{-1})\mathbf{I} - (\mathbf{r}_{A_i}^\times \mathbf{I}_{A_i}^{-1} \mathbf{r}_{A_j}^\times + \mathbf{r}_{B_i}^\times \mathbf{I}_{B_i}^{-1} \mathbf{r}_{B_j}^\times) \right] \mathbf{n}_j$$
- Solves the Linear Complementarity Problem simultaneously in double precision:
  $$\mathbf{w} = \mathbf{M} \mathbf{z} + \mathbf{q}, \quad \mathbf{w} \ge 0, \quad \mathbf{z} \ge 0, \quad \mathbf{z}^T \mathbf{w} = 0$$
  using Lemke's pivoting algorithm with Bland's rule cycle prevention (`kLcpEpsilon = 1e-9`). Eliminates contact ordering bias entirely.
- Coulomb friction is coupled via fixed-point outer-loop iterations. If the pivot cap is exceeded ($4m + 50$), the system logs a warning and falls back to the iterative solver.

### Split-Impulse Position Correction
Position correction is decoupled from the velocity solve. Instead of Baumgarte stabilization (which adds artificial kinetic energy), penetration is corrected via separate pseudo-velocities:
$$\Delta \lambda_p = \frac{\max(0, \text{penetration} - \text{slop})}{K_n} \cdot \beta$$
Pseudo-velocities are accumulated and integrated directly into `TransformComponent` at the end of the step without altering physical linear or angular momentum.

---

## 12. Physics Materials (`PhysicsMaterial.h/.cpp`, `PhysicsMaterialComponent.h`)

- `PhysicsMaterialComponent` is optional — `CollisionSystem` treats a missing component as `MaterialId::Default`.
- `GetPairProperties(a, b)` checks an explicit pair table first (canonicalized by ID). If no override exists, it combines solo properties: geometric mean for friction ($\mu = \sqrt{\mu_A \mu_B}$), arithmetic mean for restitution ($e = 0.5(e_A + e_B)$).
- Combined values are computed once per contact in `CollisionSystem` and stored on `ContactManifold`.
- Built-in materials defined in `SoloProperties()`:
  - `Default`: restitution 0.3, static friction 0.6, kinetic friction 0.4
  - `Clay`: restitution 0.0, static friction 0.8, kinetic friction 0.6 (zero bounce)
  - `Wood`: restitution 0.25, static friction 0.5, kinetic friction 0.4
  - `Rubber`: restitution 0.5, static friction 0.8, kinetic friction 0.6
  - `HardRubber`: restitution 0.75, static friction 0.8, kinetic friction 0.6
  - `SuperBall`: restitution 1.0, static friction 0.6, kinetic friction 0.4 (perfect restitution)
  - `BouncyIce`: restitution 1.0, static friction 0.0, kinetic friction 0.0 (frictionless bounce)
  - `Domino`: restitution 0.1, static friction 0.8, kinetic friction 0.6 (stable chain reaction)

---

## 13. Collider Placement

- `ColliderComponent::offset` allows the collision shape center to differ from `TransformComponent::position` in local space ($\mathbf{c}_{\text{world}} = \mathbf{p} + \mathbf{R} \mathbf{o}$). Applied in both AABB generation and narrow-phase pose construction.
- **Ground Plane Tunneling Protection**: A paper-thin visual floor collider tunnels easily under fast impacts. Ground colliders use a generous vertical half-extent (e.g. 5.0m) offset downward by that same half-extent so the top face aligns with the visible floor while a thick slab extends underground, absorbing tunneling risk.
- **Center of Mass Lever Arms**: Resolution lever arms $\mathbf{r}_A, \mathbf{r}_B$ are measured from the entity's actual `TransformComponent::position` (the center of mass), **not** the offset collider center. Offsets displace the geometric contact surface without moving the body's mass. (If offsets are ever used on dynamic bodies, the parallel-axis theorem must be applied to the inertia tensor).