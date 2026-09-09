#pragma once

namespace RT_PhysicsCore
{
    class Scene;

    class ISystem
    {
    public:
        explicit ISystem(Scene& scene);
        virtual ~ISystem();

        virtual void Update();
        virtual void FixedUpdate(double dt);
        virtual void RenderUpdate();

    protected:
        Scene& scene;
    };
}
