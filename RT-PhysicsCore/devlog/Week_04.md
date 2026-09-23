# Week 4 Dev Log — Physics Core

## Goals
- Collision detection
- Collision resolution
- Friction + restitution

## What I accomplished
- Added AABB collision detection, and tested it with simple primitives.
- Added Narrow-phase collision detection for primitives.
- Added a dedicated Collision test executable to test collision detection.

## Challenges
- I did understand how many ways to implement collision detection, and how complex it can get.
- Current collision detection Only supports primitives, and does not support complex shapes like meshes.
- No safe guard for fast moving objects, which can cause tunneling issues.

## Solutions / Decisions
- Decided to implement AABB collision detection first, and then move to narrow-phase collision detection.
- Thinking I should add GJK/EPA for narrow-phase collision detection on complex shapes, but I need to read up on it more.
- Need to implement continuous collision detection to prevent tunneling issues.

## Next Week
- Memory Optimization
- CPU + GPU optimization