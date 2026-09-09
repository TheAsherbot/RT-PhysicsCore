#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

namespace RT_PhysicsCore
{
    struct TransformComponent
    {
        glm::vec3 position{0.0f, 0.0f, 0.0f};
        glm::quat rotation{1.0f, 0.0f, 0.0f, 0.0f};
        glm::vec3 scale{1.0f, 1.0f, 1.0f};
    };

    struct WorldTransformComponent
    {
        glm::vec3 worldPosition{0.0f, 0.0f, 0.0f};
        glm::quat worldRotation{1.0f, 0.0f, 0.0f, 0.0f};
        glm::vec3 worldScale{1.0f, 1.0f, 1.0f};
    };
}
