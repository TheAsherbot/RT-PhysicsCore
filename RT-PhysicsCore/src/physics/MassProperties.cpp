#include "RT-PhysicsCore/physics/MassProperties.h"
#include "RT-PhysicsCore/utils/Log.h"

namespace RT_PhysicsCore
{
    glm::mat3 ComputeBoxInertia(float mass, const glm::vec3& halfExtents)
    {
        const glm::vec3 h2 = halfExtents * halfExtents;

        glm::mat3 inertia(0.0f);
        inertia[0][0] = (mass / 3.0f) * (h2.y + h2.z);
        inertia[1][1] = (mass / 3.0f) * (h2.x + h2.z);
        inertia[2][2] = (mass / 3.0f) * (h2.x + h2.y);
        return inertia;
    }

    glm::mat3 ComputeSphereInertia(float mass, float radius)
    {
        return glm::mat3(0.4f * mass * radius * radius); // (2/5) m r^2, isotropic
    }

    glm::mat3 ComputeCylinderInertia(float mass, float radius, float height)
    {
        float rSq = radius * radius;
        float axial = 0.5f * mass * rSq;
        float perp  = (mass / 12.0f) * (3.0f * rSq + height * height);

        glm::mat3 inertia(0.0f);
        inertia[0][0] = perp;
        inertia[1][1] = axial;
        inertia[2][2] = perp;
        return inertia;
    }

    glm::mat3 ComputeCapsuleInertia(float mass, float radius, float cylinderLength)
    {
        // Cylinder + two hemisphere caps, mass split by volume; hemisphere
        // terms shifted to the capsule's center via the parallel-axis
        // theorem (each hemisphere's own centroid sits 3R/8 from its flat face).
        const float R = radius;
        const float H = cylinderLength;
        const float rSq = R * R;

        const float volumeCylinder      = rSq * H;
        const float volumeOneHemisphere = (2.0f / 3.0f) * rSq * R;
        const float volumeTotal         = volumeCylinder + 2.0f * volumeOneHemisphere;

        const float massCylinder      = mass * (volumeCylinder / volumeTotal);
        const float massOneHemisphere = mass * (volumeOneHemisphere / volumeTotal);

        const float axial = 0.5f * massCylinder * rSq
                           + 2.0f * (0.4f * massOneHemisphere * rSq);

        const float perpCylinder    = (massCylinder / 12.0f) * (3.0f * rSq + H * H);
        const float perpHemispheres = 2.0f * massOneHemisphere
                                     * (0.4f * rSq + 0.25f * H * H + 0.375f * H * R);

        glm::mat3 inertia(0.0f);
        inertia[0][0] = perpCylinder + perpHemispheres;
        inertia[1][1] = axial;
        inertia[2][2] = perpCylinder + perpHemispheres;
        return inertia;
    }

    RigidBodyComponent MakeDynamicBody(float mass, const glm::mat3& inertiaBody)
    {
        if (mass <= 0.0f)
        {
            RT_LOG_WARN("MakeDynamicBody called with mass <= 0 (" << mass << ") - returning a static body instead.");
            return MakeStaticBody();
        }

        RigidBodyComponent body;
        body.mass = mass;
        body.invMass = 1.0f / mass;
        body.inertiaBody = inertiaBody;
        body.invInertiaBody = glm::inverse(inertiaBody);
        return body;
    }

    RigidBodyComponent MakeStaticBody()
    {
        RigidBodyComponent body;
        body.mass = 0.0f;
        body.invMass = 0.0f;
        body.inertiaBody = glm::mat3(0.0f);
        body.invInertiaBody = glm::mat3(0.0f);
        return body;
    }
}
