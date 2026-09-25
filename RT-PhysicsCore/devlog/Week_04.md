# Week 4 Dev Log — Physics Core

## Goals
- Collision detection
- Collision resolution
- Friction + restitution

## What I accomplished
- Added AABB collision detection, and tested it with simple primitives.
- Added Narrow-phase collision detection for primitives.
- Added a dedicated Collision test executable to test collision detection.
- Added collision resolution using sequential impulses.
- Added colision resolution using exact resolution.
- Added switching between exact resolution and sequential impulses, and sequential impulse iteration limiting.

## Challenges
- I did understand how many ways to implement collision detection, and how complex it can get.
- Current collision detection Only supports primitives, and does not support complex shapes like meshes.
- No safe guard for fast moving objects, which can cause tunneling issues.
- In collision resolution, I could either do exact resolution or Sequential impulses. Exact resolution is more accurate, but sequential impulses are faster and more stable. Exact has no guarantee maximum number of iterations, while sequential impulses can be limited to a maximum number of iterations.
- At the end of a physics loop I often have a lot of extra unused time. The Sequential impulses get's more accurate the more iterations you do, but it takes more time.

## Solutions / Decisions
- Decided to implement AABB collision detection first, and then move to narrow-phase collision detection.
- Thinking I should add GJK/EPA for narrow-phase collision detection on complex shapes, but I need to read up on it more.
- Need to implement continuous collision detection to prevent tunneling issues.
- Implement both exact resolution and sequential impulses, and allow the user to choose which one to use.
- Allow users to allow Sequential impulses to use the rest of the time in the physics loop to do more iterations, or limit it to a maximum number of iterations.

## Next Week
- Memory Optimization
- CPU + GPU optimization