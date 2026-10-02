/**
 * @file MemoryTracker.cpp
 * @brief Implementation of memory usage tracking and global heap operator overrides.
 * @author Asher Jordan
 * @date 2026-10-01
 */

#include "RT-PhysicsCore/telemetry/MemoryTracker.h"

#include <atomic>
#include <cstdlib>
#include <new>

namespace
{
    std::atomic<std::size_t> currentBytes{ 0 };
    std::atomic<std::size_t> peakBytes{ 0 };
    std::atomic<std::size_t> allocationCount{ 0 };
}

namespace RT_PhysicsCore
{
    std::size_t MemoryTracker::GetCurrentBytes()
    {
        return currentBytes.load(std::memory_order_relaxed);
    }

    std::size_t MemoryTracker::GetPeakBytes()
    {
        return peakBytes.load(std::memory_order_relaxed);
    }

    std::size_t MemoryTracker::GetActiveAllocationCount()
    {
        return allocationCount.load(std::memory_order_relaxed);
    }

    void MemoryTracker::RecordAllocation(std::size_t bytes)
    {
        std::size_t current = currentBytes.fetch_add(bytes, std::memory_order_relaxed) + bytes;
        allocationCount.fetch_add(1, std::memory_order_relaxed);

        std::size_t peak = peakBytes.load(std::memory_order_relaxed);
        while (current > peak && !peakBytes.compare_exchange_weak(peak, current, std::memory_order_relaxed))
        {
        }
    }

    void MemoryTracker::RecordDeallocation(std::size_t bytes)
    {
        currentBytes.fetch_sub(bytes, std::memory_order_relaxed);
        allocationCount.fetch_sub(1, std::memory_order_relaxed);
    }
}

#if defined(_DEBUG) || defined(RT_FORCE_PROFILING)

void* operator new(std::size_t size)
{
    if (size == 0)
    {
        size = 1;
    }

    void* ptr = std::malloc(size);
    if (!ptr)
    {
        throw std::bad_alloc();
    }

    RT_PhysicsCore::MemoryTracker::RecordAllocation(size);
    return ptr;
}

void operator delete(void* ptr, std::size_t size) noexcept
{
    if (ptr)
    {
        RT_PhysicsCore::MemoryTracker::RecordDeallocation(size);
        std::free(ptr);
    }
}

void operator delete(void* ptr) noexcept
{
    if (ptr)
    {
        std::free(ptr);
    }
}

void* operator new[](std::size_t size)
{
    if (size == 0)
    {
        size = 1;
    }

    void* ptr = std::malloc(size);
    if (!ptr)
    {
        throw std::bad_alloc();
    }

    RT_PhysicsCore::MemoryTracker::RecordAllocation(size);
    return ptr;
}

void operator delete[](void* ptr, std::size_t size) noexcept
{
    if (ptr)
    {
        RT_PhysicsCore::MemoryTracker::RecordDeallocation(size);
        std::free(ptr);
    }
}

void operator delete[](void* ptr) noexcept
{
    if (ptr)
    {
        std::free(ptr);
    }
}

#endif