#pragma once

#include <glm/glm.hpp>
#include "RT-PhysicsCore/physics/components/RigidBodyComponent.h"

namespace RT_PhysicsCore
{
    // Body-space inertia tensors for standard primitives (about each
    // shape's own center of mass, principal-axis frame - diagonal, uniform
    // density), plus factory functions for building a correctly-initialized
    // RigidBodyComponent. Independent of any collider component - not
    // needed until something actually constructs a body.

    glm::mat3 ComputeBoxInertia(float mass, const glm::vec3& halfExtents);
    glm::mat3 ComputeSphereInertia(float mass, float radius);
    glm::mat3 ComputeCylinderInertia(float mass, float radius, float height);   // axis = local +Y

    // Cylinder of cylinderLength (straight section only) + two hemisphere
    // caps of the given radius, axis = local +Y.
    glm::mat3 ComputeCapsuleInertia(float mass, float radius, float cylinderLength);

    // invMass = 1/mass, invInertiaBody = inverse(inertiaBody). Falls back to
    // MakeStaticBody() (with a warning) if mass <= 0.
    RigidBodyComponent MakeDynamicBody(float mass, const glm::mat3& inertiaBody);

    // invMass = 0, invInertiaBody = 0 - PhysicsSystem skips integration entirely.
    RigidBodyComponent MakeStaticBody();
}
