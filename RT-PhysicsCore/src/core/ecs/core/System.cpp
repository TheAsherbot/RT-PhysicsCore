#include "RT-PhysicsCore/core/ecs/core/System.h"

namespace RT_PhysicsCore
{
    ISystem::ISystem(Scene& scene)
        : scene(scene)
    {}

    ISystem::~ISystem() = default;

    void ISystem::Update() {}
    void ISystem::FixedUpdate(double /*dt*/) {}
    void ISystem::RenderUpdate() {}
}
