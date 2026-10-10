# Week 6 Dev Log — Demos, ECS Optimization & Symplectic Integration

## Goals
- Build interactive physics demo sandboxes
- Optimize ECS component storage for cache locality
- Improve angular / rotational integration to stop energy drift
- Expand validation testing

## What I accomplished
- Added 8 standalone interactive demo applications: FallingBoxes, BouncingBalls, DominoChain, NewtonsCradle, InclinedPlane, GyroscopeDemo, ShapeSoup, and WreckingBall
- Optimized ECS storage from `std::unordered_map` to flat sparse-sets with dense vectors for fast $O(1)$ lookups and contiguous iteration
- Implemented DLM (Dullweber–Leimkuhler–McLachlan) symplectic rotational splitting for rigid bodies
- Strictly bounded rotational energy drift ($<0.01\%$) and conserved angular momentum, naturally capturing the Dzhanibekov effect
- Stabilized demo scenes by using sequential impulses for domino chaining and tuning boundary colliders in ShapeSoup
- Significantly expanded automated test suites for friction, restitution, and split-impulse position correction
- Updated engine documentation and README with the complete feature set and architecture details

## Challenges
- Standard Euler integration for rotation was suffering from numerical drift, causing spinning bodies like the gyroscope to gain artificial kinetic energy.
- Dense scenes with lots of simultaneous contacts (like the domino chain and shape soup) were prone to instability when using exact LCP solving.
- ECS component lookup and deletion had overhead from hash map lookups and unordered iteration.

## Solutions / Decisions
- Implemented the DLM body-frame Strang splitting scheme using angular momentum instead of angular velocity to maintain symplectic energy conservation.
- Switched large stacking scenes and dominoes to use the sequential impulses solver with split-impulse position correction for stability.
- Replaced component map lookups with a flat sparse-set and dense payload arrays using swap-and-pop removal and entity ID recycling.
- Added interactive reset controls and parameter adjustments to demo applications for fast runtime testing.

## Next Week
- Continuous collision detection (CCD) to prevent fast-moving tunneling
- Broadphase spatial partitioning (Dynamic AABB Tree / BVH) to replace all-pairs checks
- Multithreaded physics solver experiments
- Final polish and demonstration recordings