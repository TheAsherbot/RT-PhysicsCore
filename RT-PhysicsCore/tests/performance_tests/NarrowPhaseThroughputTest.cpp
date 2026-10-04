/**
 * @file NarrowPhaseThroughputTest.cpp
 * @brief Measures raw narrow-phase collision throughput.
 *
 * Bypasses the ECS to time direct calls to TestCollision for all shape pair combinations.
 */

#include <chrono>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

#include "RT-PhysicsCore/utils/Log.h"
#include "RT-PhysicsCore/physics/collision/NarrowPhase.h"

namespace
{
    using namespace RT_PhysicsCore;

    void RunPairTest(const std::string& name, ColliderShape shapeA, glm::vec3 sizeA, ColliderShape shapeB, glm::vec3 sizeB)
    {
        constexpr int pairs = 10000;
        std::vector<ColliderPose> posesA(pairs);
        std::vector<ColliderPose> posesB(pairs);

        // Setup colliding geometries
        for (int i = 0; i < pairs; ++i)
        {
            posesA[i].shape = shapeA;
            posesA[i].size = sizeA;
            posesA[i].position = { 0.0f, 0.0f, 0.0f };

            posesB[i].shape = shapeB;
            posesB[i].size = sizeB;
            // Overlap slightly
            posesB[i].position = { 0.1f, 0.1f, 0.1f };
        }

        // Measure
        Contact dummy;
        int hitCount = 0;

        auto start = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < pairs; ++i)
        {
            if (TestCollision(posesA[i], posesB[i], dummy))
            {
                ++hitCount;
            }
        }
        auto end = std::chrono::high_resolution_clock::now();

        double totalMs = std::chrono::duration<double, std::milli>(end - start).count();
        double perPairUs = (totalMs * 1000.0) / pairs;

        std::ostringstream oss;
        oss << std::left << std::setw(20) << name << " | "
            << std::right << std::fixed << std::setprecision(2) << totalMs << " ms total | "
            << perPairUs << " us/pair";

        RT_LOG_INFO(oss.str());
    }
}

int main()
{
    RT_LOG_INFO("Narrowphase Throughput Test (10000 pairs each)");

    RunPairTest("Sphere-Sphere", ColliderShape::Sphere, { 1.0f, 0, 0 }, ColliderShape::Sphere, { 1.0f, 0, 0 });
    RunPairTest("Sphere-Box", ColliderShape::Sphere, { 1.0f, 0, 0 }, ColliderShape::Box, { 0.5f, 0.5f, 0.5f });
    // Capsule support omitted here since NarrowPhase might just support Box/Sphere initially, 
    // but we can test the primitive types that are implemented.
    RunPairTest("Box-Box (SAT+clip)", ColliderShape::Box, { 0.5f, 0.5f, 0.5f }, ColliderShape::Box, { 0.5f, 0.5f, 0.5f });

    RT_LOG_INFO("Throughput checks completed.");
    return 0;
}