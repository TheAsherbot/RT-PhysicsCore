#pragma once

#include <functional>

// #include "RT-PhysicsCore/utils/FixedTimestep.h"

namespace RT_PhysicsCore
{
	class FixedTimestep;

    class Engine
    {
    public:
        using FixedUpdateCallback = std::function<void(double fixedDeltaTime)>;
        using UpdateCallback = std::function<void(double eltaTime)>;
        using RenderCallback = std::function<void(double alpha)>;

        Engine(double physicsHz = 60.0);
        ~Engine();

        void SetFixedUpdateCallback(FixedUpdateCallback cb);
        void SetUpdateCallback(UpdateCallback cb);
        void SetRenderCallback(RenderCallback cb);

        void Run();
        void RequestExit();

    private:
        bool isRunning = true;
        FixedTimestep* fixedTimestep;

        FixedUpdateCallback fixedUpdateCallback;
        UpdateCallback updateCallback;
        RenderCallback renderCallback;
    };
}
