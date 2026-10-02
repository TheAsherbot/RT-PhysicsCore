/**
 * @file Engine.cpp
 * @brief Implementation of the main Engine coordinator and execution loop.
 */


#include "RT-PhysicsCore/core/Engine.h"
#include "utils/FixedTimestep.h"

#include "RT-PhysicsCore/utils/Log.h"

#include "RT-PhysicsCore/telemetry/TelemetryManager.h"

#include <chrono>

namespace RT_PhysicsCore
{
    Engine::Engine(double physicsHz)
        : fixedTimestep(new FixedTimestep(physicsHz))
    {
        RT_LOG_INFO("Engine constructed at " << physicsHz << " Hz");
    }

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
        RT_LOG_INFO("Exit requested");
        isRunning = false;
    }

    void Engine::Run()
    {
        RT_LOG_INFO("Engine loop starting");

        while (isRunning)
        {
            RT_PROFILE_SCOPE("Frame");

            auto frameStart = std::chrono::high_resolution_clock::now();

            std::uint32_t steps = fixedTimestep->Step();
            double fixedDeltaTime = fixedTimestep->FixedDeltaSeconds();
            double deltaTime = fixedTimestep->DeltaSeconds();

            double physicsMs = 0.0;
            {
                RT_PROFILE_SCOPE("FixedUpdate");
                auto physicsStart = std::chrono::high_resolution_clock::now();

                for (std::uint32_t i = 0; i < steps; i++)
                {
                    if (fixedUpdateCallback)
                    {
                        fixedUpdateCallback(fixedDeltaTime);
                    }
                }

                auto physicsEnd = std::chrono::high_resolution_clock::now();
                physicsMs = std::chrono::duration<double, std::milli>(physicsEnd - physicsStart).count();
            }

            double updateMs = 0.0;
            {
                RT_PROFILE_SCOPE("Update");
                auto updateStart = std::chrono::high_resolution_clock::now();

                if (updateCallback)
                {
                    updateCallback(deltaTime);
                }

                auto updateEnd = std::chrono::high_resolution_clock::now();
                updateMs = std::chrono::duration<double, std::milli>(updateEnd - updateStart).count();
            }

            double alpha = fixedTimestep->Alpha();
            double renderMs = 0.0;
            {
                RT_PROFILE_SCOPE("Render");
                auto renderStart = std::chrono::high_resolution_clock::now();

                if (renderCallback)
                {
                    renderCallback(alpha);
                }

                auto renderEnd = std::chrono::high_resolution_clock::now();
                renderMs = std::chrono::duration<double, std::milli>(renderEnd - renderStart).count();
            }

            auto frameEnd = std::chrono::high_resolution_clock::now();
            double totalFrameMs = std::chrono::duration<double, std::milli>(frameEnd - frameStart).count();

            TelemetryManager::Get().PushFrameSummary({ totalFrameMs, physicsMs, updateMs, renderMs });
        }

        RT_LOG_INFO("Engine loop stopped");
    }
}