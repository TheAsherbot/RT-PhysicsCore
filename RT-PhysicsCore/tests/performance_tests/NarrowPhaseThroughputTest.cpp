/**
 * @file NarrowPhaseThroughputTest.cpp
 * @brief Measures raw narrow-phase collision throughput across all primitive pair combinations.
 */

#include <chrono>
#include <iomanip>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include "RT-PhysicsCore/utils/Log.h"
#include "RT-PhysicsCore/physics/collision/NarrowPhase.h"
#include <glm/gtc/quaternion.hpp>

namespace
{
    using namespace RT_PhysicsCore;

    struct PairBenchmarkResult
    {
        std::string name;
        int totalQueries;
        int hitCount;
        int missCount;
        double totalMs;
        double perQueryUs;
    };

    PairBenchmarkResult BenchmarkPairType(const std::string& name,
        ColliderShape shapeA, glm::vec3 sizeA,
        ColliderShape shapeB, glm::vec3 sizeB)
    {
        constexpr int kTotalPairs = 10000;
        constexpr int kHalfPairs = kTotalPairs / 2;

        std::vector<ColliderPose> posesA(kTotalPairs);
        std::vector<ColliderPose> posesB(kTotalPairs);

        std::mt19937 rng(1337);
        std::uniform_real_distribution<float> angleDist(0.0f, 6.283185f);
        std::uniform_real_distribution<float> overlapOffset(-0.2f, 0.2f);
        std::uniform_real_distribution<float> separationOffset(3.0f, 6.0f);

        for (int i = 0; i < kTotalPairs; ++i)
        {
            posesA[i].shape = shapeA;
            posesA[i].size = sizeA;
            posesA[i].position = { 0.0f, 0.0f, 0.0f };
            glm::quat rotA = glm::angleAxis(angleDist(rng), glm::vec3(0.0f, 1.0f, 0.0f));
            posesA[i].rotation = glm::mat3_cast(rotA);

            posesB[i].shape = shapeB;
            posesB[i].size = sizeB;
            glm::quat rotB = glm::angleAxis(angleDist(rng), glm::vec3(1.0f, 0.0f, 0.0f));
            posesB[i].rotation = glm::mat3_cast(rotB);

            if (i < kHalfPairs)
            {
                // Overlapping configuration: positions interpenetrate slightly
                posesB[i].position = { overlapOffset(rng), overlapOffset(rng), overlapOffset(rng) };
            }
            else
            {
                // Separated configuration: tests SAT / distance early-out rejection
                posesB[i].position = { separationOffset(rng), 0.0f, 0.0f };
            }
        }

        Contact contact;
        int hits = 0;
        int misses = 0;

        auto start = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < kTotalPairs; ++i)
        {
            if (TestCollision(posesA[i], posesB[i], contact))
            {
                ++hits;
            }
            else
            {
                ++misses;
            }
        }
        auto end = std::chrono::high_resolution_clock::now();

        double totalMs = std::chrono::duration<double, std::milli>(end - start).count();
        double perQueryUs = (totalMs * 1000.0) / kTotalPairs;

        return { name, kTotalPairs, hits, misses, totalMs, perQueryUs };
    }
}

int main()
{
    RT_LOG_INFO("Narrowphase Throughput Benchmark (10,000 queries per pair type: 5,000 overlapping, 5,000 separated)");

    std::vector<PairBenchmarkResult> results;

    // 1. Sphere-Sphere
    results.push_back(BenchmarkPairType("Sphere - Sphere",
        ColliderShape::Sphere, { 0.5f, 0.0f, 0.0f },
        ColliderShape::Sphere, { 0.5f, 0.0f, 0.0f }));

    // 2. Sphere-Box
    results.push_back(BenchmarkPairType("Sphere - Box",
        ColliderShape::Sphere, { 0.5f, 0.0f, 0.0f },
        ColliderShape::Box, { 0.5f, 0.5f, 0.5f }));

    // 3. Sphere-Capsule
    results.push_back(BenchmarkPairType("Sphere - Capsule",
        ColliderShape::Sphere, { 0.4f, 0.0f, 0.0f },
        ColliderShape::Capsule, { 0.4f, 0.8f, 0.0f }));

    // 4. Capsule-Capsule
    results.push_back(BenchmarkPairType("Capsule - Capsule",
        ColliderShape::Capsule, { 0.4f, 0.8f, 0.0f },
        ColliderShape::Capsule, { 0.4f, 0.8f, 0.0f }));

    // 5. Box-Capsule
    results.push_back(BenchmarkPairType("Box - Capsule",
        ColliderShape::Box, { 0.5f, 0.5f, 0.5f },
        ColliderShape::Capsule, { 0.4f, 0.8f, 0.0f }));

    // 6. Box-Box (15-axis SAT + Sutherland-Hodgman clipping)
    results.push_back(BenchmarkPairType("Box - Box (SAT)",
        ColliderShape::Box, { 0.5f, 0.5f, 0.5f },
        ColliderShape::Box, { 0.5f, 0.5f, 0.5f }));

    RT_LOG_INFO("=========================================================================================");
    for (const auto& r : results)
    {
        std::ostringstream oss;
        oss << std::left << std::setw(22) << r.name << " | "
            << "Hits: " << std::setw(5) << r.hitCount << " | "
            << "Misses: " << std::setw(5) << r.missCount << " | "
            << std::right << std::fixed << std::setprecision(2) << std::setw(6) << r.totalMs << " ms | "
            << std::setprecision(3) << std::setw(6) << r.perQueryUs << " us/query";
        RT_LOG_INFO(oss.str());
    }
    RT_LOG_INFO("=========================================================================================");

    return 0;
}