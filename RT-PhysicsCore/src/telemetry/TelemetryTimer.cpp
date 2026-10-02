/**
 * @file TelemetryTimer.cpp
 * @brief Implementation of the RAII-based execution scope timer.
 */

#include "RT-PhysicsCore/telemetry/TelemetryTimer.h"
#include "RT-PhysicsCore/telemetry/TelemetryManager.h"
#include "RT-PhysicsCore/telemetry/MemoryTracker.h"

#include <chrono>
#include <string>
#include <thread>

namespace RT_PhysicsCore
{
    TelemetryTimer::TelemetryTimer(const char* name)
        : name(name)
        , isStopped(false)
    {
        startTimepoint = std::chrono::high_resolution_clock::now();
    }

    TelemetryTimer::~TelemetryTimer()
    {
        if (!isStopped)
        {
            Stop();
        }
    }

    void TelemetryTimer::Stop()
    {
        auto endTimepoint = std::chrono::high_resolution_clock::now();

        long long start = std::chrono::time_point_cast<std::chrono::microseconds>(startTimepoint).time_since_epoch().count();
        long long end = std::chrono::time_point_cast<std::chrono::microseconds>(endTimepoint).time_since_epoch().count();
        
        uint32_t threadId = static_cast<uint32_t>(std::hash<std::thread::id>{}(std::this_thread::get_id()));
        
        ProfileResult result;
        result.name = name;
        result.startTimestamp = start;
        result.endTimestamp = end;
        result.threadId = threadId;

        // Take a memory snapshot precisely at the end of the frame
        if (name && std::string(name) == "Frame")
        {
            result.memoryCurrentBytes = MemoryTracker::GetCurrentBytes();
            result.memoryPeakBytes = MemoryTracker::GetPeakBytes();
            result.memoryAllocations = MemoryTracker::GetActiveAllocationCount();
        }

        TelemetryManager::Get().WriteProfile(result);
        
        isStopped = true;
    }
}