# RT-PhysicsCore — Performance & Validation Benchmarks

This document details the throughput benchmarks, memory performance, and physical validation metrics for RT-PhysicsCore v1.0.0.

All benchmarks were measured on Windows x64 (MSVC C++17) using high-resolution steady clocks.

---

## 1. Narrowphase Collision Throughput
Tested across 10,000 queries per primitive pair (5,000 overlapping, 5,000 separated):

| Primitive Pair | Algorithm / Implementation | Total Time (10k queries) | Throughput (per query) |
|---|---|---|---|
| **Sphere – Sphere** | Center distance check | 0.90 ms | **0.090 µs** |
| **Sphere – Box** | Closest-point clamping | 5.47 ms | **0.547 µs** |
| **Sphere – Capsule** | Point-to-segment distance | 2.01 ms | **0.201 µs** |
| **Capsule – Capsule** | Ericson segment closest points | 3.94 ms | **0.394 µs** |
| **Box – Capsule** | Interval detection / projection | 16.71 ms | **1.671 µs** |
| **Box – Box** | 15-axis SAT + Sutherland–Hodgman | 47.42 ms | **4.742 µs** |

---

## 2. ECS Architecture Scalability
Measured using flat sparse-set component storage with dense vector iterations on 100,000 entities:

| Operation | Scale | Total Time | Per-Operation Cost |
|---|---|---|---|
| **Entity Creation** | 100,000 entities | 10.98 ms | **0.11 µs / entity** |
| **Component Attachment** | 3 components / entity (300k total) | 191.41 ms | **1.91 µs / op** |
| **Multi-Component Query** | `Query<Transform, RigidBody>()` | 7.39 ms | **0.07 µs / match** |
| **Entity Recycling / Deletion**| Free-list recycling & swap-and-pop | 84.62 ms | **0.85 µs / entity** |

---

## 3. Physical Validation Suite (Automated CTest)
The engine includes 21 automated validation suites executing without a window context in CI:

- **Energy Conservation**: Unconstrained asymmetric rigid body tumbling (Dzhanibekov effect) maintains kinetic energy drift under **<0.001%** across 3,600 ticks using the DLM symplectic scheme.
- **Angular Momentum Conservation**: Total momentum magnitude $\|\mathbf{L}\|$ is strictly conserved ($<0.01$ drift over 600 ticks).
- **Restitution Accuracy**: Normal separation velocities after impact match analytical coefficient of restitution targets within **<5%** error.
- **Coulomb Cone Friction**: Box deceleration distances on flat planes match Coulomb analytical sliding distance $s = \frac{v_0^2}{2 \mu g}$ within **<8%** relative error.