#pragma once

#include "RT-PhysicsCore/core/ecs/core/System.h"
#include <glm/glm.hpp>

namespace RT_PhysicsCore
{
    class PhysicsSystem : public ISystem
    {
    public:
        explicit PhysicsSystem(Scene& scene);

        void FixedUpdate(double dt) override;

        // Applied as mass * gravity in forceAccum, not a direct velocity
        // change - see FixedUpdate. Defaults to 9.80665 m/s^2, -Y.
        void SetGravity(const glm::vec3& gravity);
        const glm::vec3& GetGravity() const;

    private:
        glm::vec3 gravity{ 0.0f, -9.80665f, 0.0f };
    };
}
