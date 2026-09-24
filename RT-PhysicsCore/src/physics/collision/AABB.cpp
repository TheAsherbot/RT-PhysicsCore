#include "RT-PhysicsCore/physics/collision/AABB.h"

namespace RT_PhysicsCore
{
    namespace
    {
        AABB BoxAABB(const glm::vec3& halfExtents, const glm::vec3& position, const glm::mat3& rotation)
        {
            glm::vec3 extent = glm::abs(rotation[0]) * halfExtents.x
                              + glm::abs(rotation[1]) * halfExtents.y
                              + glm::abs(rotation[2]) * halfExtents.z;
            return { position - extent, position + extent };
        }

        AABB SphereAABB(float radius, const glm::vec3& position)
        {
            glm::vec3 r(radius);
            return { position - r, position + r };
        }

        AABB CapsuleAABB(float radius, float halfLength, const glm::vec3& position, const glm::mat3& rotation)
        {
            glm::vec3 axis = rotation[1] * halfLength;
            glm::vec3 a = position - axis;
            glm::vec3 b = position + axis;

            glm::vec3 r(radius);
            return { glm::min(a, b) - r, glm::max(a, b) + r };
        }
    }

    AABB ComputeWorldAABB(const ColliderComponent& collider, const glm::vec3& position, const glm::quat& rotation)
    {
        glm::mat3 r = glm::mat3_cast(rotation);

        switch (collider.shape)
        {
            case ColliderShape::Box:     return BoxAABB(collider.size, position, r);
            case ColliderShape::Sphere:  return SphereAABB(collider.size.x, position);
            case ColliderShape::Capsule: return CapsuleAABB(collider.size.x, collider.size.y, position, r);
        }
        return { position, position };
    }

    bool Overlaps(const AABB& a, const AABB& b)
    {
        return (a.min.x <= b.max.x && a.max.x >= b.min.x)
            && (a.min.y <= b.max.y && a.max.y >= b.min.y)
            && (a.min.z <= b.max.z && a.max.z >= b.min.z);
    }
}
