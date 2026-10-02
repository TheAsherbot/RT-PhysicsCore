/**
 * @file MemoryTracker.h
 * @brief Thread-safe memory allocation monitor tracking heap usage statistics.
 */

#pragma once

#include <cstddef>

namespace RT_PhysicsCore
{
    /**
     * @class MemoryTracker
     * @brief Exposes atomic counters and bookkeeping interfaces for heap allocations and deallocations.
     */
    class MemoryTracker
    {
    public:
        /**
         * @brief Queries the current active heap memory consumed in bytes.
         * @return Active bytes currently allocated.
         */
        static std::size_t GetCurrentBytes();

        /**
         * @brief Queries the highest recorded heap memory consumed during the process lifecycle.
         * @return Peak bytes recorded.
         */
        static std::size_t GetPeakBytes();

        /**
         * @brief Queries total active allocation count.
         * @return Number of outstanding heap allocations.
         */
        static std::size_t GetActiveAllocationCount();

        /**
         * @brief Records a heap allocation of a specific size.
         * @param bytes Number of bytes allocated.
         */
        static void RecordAllocation(std::size_t bytes);

        /**
         * @brief Records a heap deallocation of a specific size.
         * @param bytes Number of bytes deallocated.
         */
        static void RecordDeallocation(std::size_t bytes);
    };
}