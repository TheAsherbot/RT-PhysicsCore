/**
 * @file MassProperties.h
 * @brief Analytical inertia tensor calculations and rigid body construction helpers.
 *
 * Computes body-space principal-axis inertia tensors for standard geometric primitives
 * (boxes, spheres, cylinders, capsules) and constructs initialized RigidBodyComponent instances.
 */

#pragma once

#include <glm/glm.hpp>
#include "RT-PhysicsCore/physics/components/RigidBodyComponent.h"

namespace RT_PhysicsCore
{
    /**
     * @brief Computes the body-space inertia tensor for a solid uniform box.
     * @param mass Total mass of the box in kilograms.
     * @param halfExtents Half-dimensions of the box along X, Y, and Z axes.
     * @return Diagonal 3x3 inertia tensor matrix about the center of mass.
     */
    glm::mat3 ComputeBoxInertia(float mass, const glm::vec3& halfExtents);

    /**
     * @brief Computes the body-space inertia tensor for a solid uniform sphere.
     * @param mass Total mass of the sphere in kilograms.
     * @param radius Radius of the sphere in meters.
     * @return Isotropic 3x3 inertia tensor matrix: (2/5) * mass * radius^2 * I.
     */
    glm::mat3 ComputeSphereInertia(float mass, float radius);

    /**
     * @brief Computes the body-space inertia tensor for a solid uniform cylinder.
     * @param mass Total mass of the cylinder in kilograms.
     * @param radius Radius of the circular cross-section in meters.
     * @param height Total cylinder height along the local +Y axis in meters.
     * @return Diagonal 3x3 inertia tensor matrix.
     */
    glm::mat3 ComputeCylinderInertia(float mass, float radius, float height);

    /**
     * @brief Computes the body-space inertia tensor for a capped capsule aligned with local +Y.
     *
     * Evaluates the combined inertia of the central cylindrical sleeve and two hemispherical
     * end caps shifted using the parallel-axis theorem.
     *
     * @param mass Total mass of the capsule in kilograms.
     * @param radius Radius of the cylinder and hemispherical end caps.
     * @param halfLength Half-length of the inner cylinder segment (matches ColliderComponent size.y).
     * @return Diagonal 3x3 inertia tensor matrix.
     */
    glm::mat3 ComputeCapsuleInertia(float mass, float radius, float halfLength);

    /**
     * @brief Factory creating a dynamic RigidBodyComponent with mass and inverse inertia.
     * @note If mass <= 0.0f, a warning is logged and a static body is returned instead.
     * @param mass Positive non-zero mass in kilograms.
     * @param inertiaBody Body-space principal-axis inertia tensor.
     * @return Fully initialized dynamic RigidBodyComponent.
     */
    RigidBodyComponent MakeDynamicBody(float mass, const glm::mat3& inertiaBody);

    /**
     * @brief Factory creating an immovable static RigidBodyComponent with zero inverse mass.
     * @return Static RigidBodyComponent skipped by velocity and positional integration.
     */
    RigidBodyComponent MakeStaticBody();
}