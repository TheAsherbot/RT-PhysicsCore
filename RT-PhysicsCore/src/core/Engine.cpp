/**
 * @file Engine.cpp
 * @brief Implementation of the main Engine coordinator and execution loop.
 */

#include "RT-PhysicsCore/core/Engine.h"
#include "utils/FixedTimestep.h"

#include "RT-PhysicsCore/utils/Log.h"

#include "RT-PhysicsCore/telemetry/TelemetryManager.h"

#include <chrono>
#include <thread>

#if defined(_WIN32)
#define NOMINMAX
#include <windows.h>
#include <timeapi.h>
#pragma comment(lib, "winmm.lib")
#endif

namespace RT_PhysicsCore
{
    Engine::Engine(double physicsHz, double maxFrameRate)
        : isRunning(true),
        maxFrameRate(maxFrameRate),
        fixedTimestep(new FixedTimestep(physicsHz))
    {
        if (maxFrameRate > 0.0)
        {
            RT_LOG_INFO("Engine constructed at " << physicsHz << " Hz (max frame rate: " << maxFrameRate << " FPS)");
        }
        else
        {
            RT_LOG_INFO("Engine constructed at " << physicsHz << " Hz (max frame rate: unlimited)");
        }
    }

    Engine::~Engine()
    {
        delete fixedTimestep;
    }

    void Engine::SetMaxFrameRate(double maxFrameRate)
    {
        this->maxFrameRate = maxFrameRate;
        if (maxFrameRate > 0.0)
        {
            RT_LOG_INFO("Max frame rate set to " << maxFrameRate << " FPS");
        }
        else
        {
            RT_LOG_INFO("Max frame rate set to unlimited");
        }
    }

    double Engine::GetMaxFrameRate() const
    {
        return maxFrameRate;
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

#if defined(_WIN32)
        // Request 1 ms OS timer granularity on Windows so thread sleeping is accurate
        timeBeginPeriod(1);
#endif

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

            if (maxFrameRate > 0.0)
            {
                RT_PROFILE_SCOPE("FrameLimiter");

                const std::chrono::duration<double> targetFrameDuration(1.0 / maxFrameRate);
                const auto targetEndTime = frameStart + std::chrono::duration_cast<std::chrono::high_resolution_clock::duration>(targetFrameDuration);

                auto now = std::chrono::high_resolution_clock::now();
                while (now < targetEndTime)
                {
                    auto remaining = targetEndTime - now;

                    // Only sleep if remaining time is large (> 5 ms) to prevent OS scheduler oversleep
                    if (remaining > std::chrono::milliseconds(5))
                    {
                        std::this_thread::sleep_for(std::chrono::milliseconds(1));
                    }
                    else if (remaining > std::chrono::microseconds(500))
                    {
                        std::this_thread::yield();
                    }
                    else
                    {
                        // Sub-millisecond busy-wait for exact frame edge alignment
                    }

                    now = std::chrono::high_resolution_clock::now();
                }
            }

            auto frameEnd = std::chrono::high_resolution_clock::now();
            double totalFrameMs = std::chrono::duration<double, std::milli>(frameEnd - frameStart).count();

            TelemetryManager::Get().PushFrameSummary({ totalFrameMs, physicsMs, updateMs, renderMs });
        }

#if defined(_WIN32)
        timeEndPeriod(1);
#endif

        RT_LOG_INFO("Engine loop stopped");
    }
}