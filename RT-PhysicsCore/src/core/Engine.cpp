#include "RT-PhysicsCore/core/Engine.h"
#include "utils/FixedTimestep.h"
#include "RT-PhysicsCore/utils/Log.h"

namespace RT_PhysicsCore
{
    Engine::Engine(double physicsHz)
        : fixedTimestep(new FixedTimestep(physicsHz))
    {
        RT_LOG_INFO("Engine constructed at " << physicsHz << " Hz");
    }

    Engine::~Engine() = default;

    void Engine::SetFixedUpdateCallback(FixedUpdateCallback fixedUpdateCallBack)
    {
        this->fixedUpdateCallback = std::move(fixedUpdateCallBack);
    }

	void Engine::SetUpdateCallback(UpdateCallback updateCallBack)
	{
		this->updateCallback = std::move(updateCallBack);
	}

    void Engine::SetRenderCallback(RenderCallback renderCallBack)
    {
        this->renderCallback = std::move(renderCallBack);
    }

    void Engine::RequestExit()
    {
        RT_LOG_INFO("Exit requested");
        isRunning = false;
    }


    void Engine::Run()
    {
        RT_LOG_INFO("Engine loop starting");

        while (isRunning)
        {
            std::uint32_t steps = fixedTimestep->Step();
            double fixedDeltaTime = fixedTimestep->FixedDeltaSeconds();
            double deltaTime = fixedTimestep->DeltaSeconds();

            for (std::uint32_t i = 0; i < steps; i++)
            {
                if (fixedUpdateCallback)
                    fixedUpdateCallback(fixedDeltaTime);
            }

            // Normal update must run before render: RenderSystem reads
            // WorldTransformComponent, which TransformPropagationSystem
            // recomputes during the normal update callback.
            if (updateCallback)
                updateCallback(deltaTime);

            double alpha = fixedTimestep->Alpha();
            if (renderCallback)
                renderCallback(alpha);
        }

        RT_LOG_INFO("Engine loop stopped");
    }
}
