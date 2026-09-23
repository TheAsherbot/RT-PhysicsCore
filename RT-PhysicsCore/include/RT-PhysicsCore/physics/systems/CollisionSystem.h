#pragma once

#include "RT-PhysicsCore/core/ecs/core/System.h"
#include "RT-PhysicsCore/physics/collision/Contact.h"
#include <vector>

namespace RT_PhysicsCore
{
    class CollisionSystem : public ISystem
    {
    public:
        explicit CollisionSystem(Scene& scene);

        void FixedUpdate(double dt) override;

        // Valid for the rest of this fixed step, until the next FixedUpdate
        // rebuilds it. A future resolution system reads this.
        const std::vector<Contact>& GetContacts() const { return contacts; }

    private:
        std::vector<Contact> contacts;
    };
}
