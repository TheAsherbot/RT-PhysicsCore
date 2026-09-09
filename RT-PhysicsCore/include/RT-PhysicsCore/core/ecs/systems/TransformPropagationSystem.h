#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include "RT-PhysicsCore/core/ecs/core/System.h"
#include "RT-PhysicsCore/core/ecs/core/Entity.h"

namespace RT_PhysicsCore
{
    class TransformPropagationSystem : public ISystem
    {
    public:
        explicit TransformPropagationSystem(Scene& scene);

        void Update() override;

    private:
        void Propagate(Entity entity,
                        const glm::vec3& parentWorldPos,
                        const glm::quat& parentWorldRot,
                        const glm::vec3& parentWorldScale);
    };
}
