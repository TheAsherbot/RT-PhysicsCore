/**
 * @file ECSChurnTest.cpp
 * @brief Benchmarks ECS architecture overhead.
 *
 * Measures creation, component addition, contiguous querying, and destruction speeds.
 */

#include <chrono>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

#include "RT-PhysicsCore/utils/Log.h"
#include "RT-PhysicsCore/core/ecs/core/Scene.h"
#include "RT-PhysicsCore/core/ecs/components/TransformComponent.h"
#include "RT-PhysicsCore/physics/components/RigidBodyComponent.h"
#include "RT-PhysicsCore/physics/components/ColliderComponent.h"

namespace
{
    using namespace RT_PhysicsCore;

    void LogTime(const std::string& desc, int count, double totalMs)
    {
        double perOpUs = (totalMs * 1000.0) / count;
        std::ostringstream oss;
        oss << std::left << std::setw(30) << desc << " | "
            << std::right << std::fixed << std::setprecision(2) << totalMs << " ms total | "
            << perOpUs << " us/op";
        RT_LOG_INFO(oss.str());
    }
}

int main()
{
    RT_LOG_INFO("ECS Churn Test (100,000 entities)");

    Scene scene;
    constexpr int count = 100000;
    std::vector<Entity> entities;
    entities.reserve(count);

    // 1. Create
    auto t1 = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < count; ++i)
    {
        entities.push_back(scene.CreateEntity());
    }
    auto t2 = std::chrono::high_resolution_clock::now();
    LogTime("Create entities", count, std::chrono::duration<double, std::milli>(t2 - t1).count());

    // 2. Add components
    auto t3 = std::chrono::high_resolution_clock::now();
    for (Entity e : entities)
    {
        scene.AddComponent(e, TransformComponent{});
        scene.AddComponent(e, RigidBodyComponent{});
        scene.AddComponent(e, ColliderComponent{});
    }
    auto t4 = std::chrono::high_resolution_clock::now();
    LogTime("Add 3 components", count, std::chrono::duration<double, std::milli>(t4 - t3).count());

    // 3. Query
    auto t5 = std::chrono::high_resolution_clock::now();
    int hitCount = 0;
    for (Entity e : scene.Query<TransformComponent, RigidBodyComponent>())
    {
        ++hitCount;
    }
    auto t6 = std::chrono::high_resolution_clock::now();
    LogTime("Query<Transform, RigidBody>", count, std::chrono::duration<double, std::milli>(t6 - t5).count());

    // 4. Destroy
    auto t7 = std::chrono::high_resolution_clock::now();
    std::vector<Entity> toDestroy = scene.GetEntities();
    for (Entity e : toDestroy)
    {
        scene.DestroyEntity(e);
    }
    auto t8 = std::chrono::high_resolution_clock::now();
    LogTime("Destroy entities", count, std::chrono::duration<double, std::milli>(t8 - t7).count());

    return 0;
}