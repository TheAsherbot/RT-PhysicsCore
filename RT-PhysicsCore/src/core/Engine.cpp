#include "RT-PhysicsCore/core/Engine.h"
#include "utils/FixedTimestep.h"

namespace RT_PhysicsCore
{
    Engine::Engine(double physicsHz)
        : fixedTimestep(new FixedTimestep(physicsHz))
    {}

    Engine::~Engine()
    {
        delete fixedTimestep;
    }

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
        isRunning = false;
    }


    void Engine::Run()
    {
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

            double alpha = fixedTimestep->Alpha();

            if (renderCallback)
                renderCallback(alpha);
			if (updateCallback)
				updateCallback(deltaTime);
        }
    }
}
