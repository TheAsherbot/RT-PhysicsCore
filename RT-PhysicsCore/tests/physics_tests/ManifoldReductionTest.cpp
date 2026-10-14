/**
 * @file ManifoldReductionTest.cpp
 * @brief Validates 4-point contact manifold reduction, area maximization, and degenerate filtering.
 */

#include <chrono>
#include <cmath>
#include <sstream>
#include <string>
#include <vector>

#include "RT-PhysicsCore/utils/Log.h"
#include "RT-PhysicsCore/utils/DebugDraw.h"
#include "RT-PhysicsCore/rendering/Renderer.h"
#include "RT-PhysicsCore/physics/collision/ManifoldReduction.h"
#include "RT-PhysicsCore/physics/collision/Contact.h"

namespace
{
    using namespace RT_PhysicsCore;

    /**
     * @brief Computes 2D polygon area of reduced points projected along normal.
     */
    float ComputePolygonArea(const ContactPoint* points, int count, const Vec3& normal)
    {
        if (count < 3)
        {
            return 0.0f;
        }

        Vec3 areaVec{ 0.0, 0.0, 0.0 };
        for (int i = 0; i < count; ++i)
        {
            int next = (i + 1) % count;
            areaVec += glm::cross(points[i].position, points[next].position);
        }
        return 0.5f * std::abs(glm::dot(areaVec, normal));
    }
}

int main(int argc, char** argv)
{
    RT_LOG_INFO("Manifold Reduction Test harness starting.");

    bool enableGui = false;
    for (int i = 1; i < argc; ++i)
    {
        std::string arg = argv[i];
        if (arg == "--gui" || arg == "-g" || arg == "--visual" || arg == "-v")
        {
            enableGui = true;
            break;
        }
    }

    int passCount = 0;
    int totalCount = 0;
    Vec3 normal{ 0.0, 1.0, 0.0 };

    // ── Check 1: Pass-Through for <= 4 Points ──
    {
        ++totalCount;
        ContactPoint inPoints[3];
        inPoints[0].position = { -1.0, 0.0, -1.0 };
        inPoints[1].position = { 1.0, 0.0, -1.0 };
        inPoints[2].position = { 0.0, 0.0, 1.0 };

        ContactPoint outPoints[4];
        int count = ManifoldReduction::Reduce(inPoints, 3, normal, outPoints);

        bool pass = (count == 3) && (outPoints[0].position == inPoints[0].position);
        if (pass)
        {
            ++passCount;
        }

        std::ostringstream oss;
        oss << "count=" << count << " (expected 3)";
        RT_LOG_INFO((pass ? "[PASS] " : "[FAIL] ") << "Pass-Through Small Sets (<= 4 points) - " << oss.str());
    }

    // ── Check 2: Deepest Penetration Point Guaranteed Retention ──
    {
        ++totalCount;
        // 6 points on a circle, but point index 4 has the deepest penetration (-0.05m vs -0.01m)
        ContactPoint inPoints[6];
        for (int i = 0; i < 6; ++i)
        {
            float theta = static_cast<float>(i) * (2.0f * 3.14159265f / 6.0f);
            inPoints[i].position = { std::cos(theta), 0.0, std::sin(theta) };
            inPoints[i].signedDistance = -0.01;
        }
        inPoints[4].signedDistance = -0.05; // deepest

        ContactPoint outPoints[4];
        int count = ManifoldReduction::Reduce(inPoints, 6, normal, outPoints);

        // outPoints[0] must strictly be the deepest point (index 4)
        bool pass = (count == 4) && (outPoints[0].signedDistance == -0.05);
        if (pass)
        {
            ++passCount;
        }

        std::ostringstream oss;
        oss << "count=" << count << " out[0].dist=" << outPoints[0].signedDistance << " (expected -0.05)";
        RT_LOG_INFO((pass ? "[PASS] " : "[FAIL] ") << "Deepest Penetration Point Retention - " << oss.str());
    }

    // ── Check 3: 8-Point Octagon Reduced to Max-Area Quad ──
    {
        ++totalCount;
        // 8 points forming an octagon of radius 2.0m on the XZ plane
        ContactPoint inPoints[8];
        for (int i = 0; i < 8; ++i)
        {
            float theta = static_cast<float>(i) * (2.0f * 3.14159265f / 8.0f);
            inPoints[i].position = { 2.0 * std::cos(theta), 0.0, 2.0 * std::sin(theta) };
            inPoints[i].signedDistance = -0.002;
        }

        ContactPoint outPoints[4];
        int count = ManifoldReduction::Reduce(inPoints, 8, normal, outPoints);

        float area = ComputePolygonArea(outPoints, count, normal);
        // An inscribed square of radius 2 has area = 2 * r^2 = 8.0
        bool pass = (count == 4) && (area >= 7.0f);
        if (pass)
        {
            ++passCount;
        }

        std::ostringstream oss;
        oss << "count=" << count << " quadArea=" << area << " (expected >= 7.0)";
        RT_LOG_INFO((pass ? "[PASS] " : "[FAIL] ") << "8-Point Octagon Area Maximization - " << oss.str());
    }

    // ── Check 4: Collinear Degeneration Handling ──
    {
        ++totalCount;
        // 5 points strictly lying on the line along the X axis
        ContactPoint inPoints[5];
        for (int i = 0; i < 5; ++i)
        {
            inPoints[i].position = { static_cast<double>(i) - 2.0, 0.0, 0.0 };
            inPoints[i].signedDistance = -0.001;
        }

        ContactPoint outPoints[4];
        int count = ManifoldReduction::Reduce(inPoints, 5, normal, outPoints);

        // A collinear set has zero perpendicular triangle area, so it must gracefully return the 2 extreme segment endpoints
        bool pass = (count == 2);
        if (pass)
        {
            ++passCount;
        }

        std::ostringstream oss;
        oss << "count=" << count << " (expected 2 collinear endpoints)";
        RT_LOG_INFO((pass ? "[PASS] " : "[FAIL] ") << "Collinear Degeneration Filtering - " << oss.str());
    }

    RT_LOG_INFO(passCount << "/" << totalCount << " automated checks passed.");

    if (!enableGui)
    {
        RT_LOG_INFO("Headless execution complete (pass --gui to open interactive visualizer).");
        return (passCount == totalCount) ? 0 : 1;
    }

    // ── Phase 2: Interactive visualization ──

    Renderer renderer(1280, 720, "RT-PhysicsCore Manifold Reduction Test");
    if (!renderer.IsValid())
    {
        return 1;
    }

    // Build visual set of 8 points
    ContactPoint rawPoints[8];
    for (int i = 0; i < 8; ++i)
    {
        float theta = static_cast<float>(i) * (2.0f * 3.14159265f / 8.0f);
        rawPoints[i].position = { 3.0 * std::cos(theta), 0.0, 3.0 * std::sin(theta) };
        rawPoints[i].signedDistance = (i == 2) ? -0.05 : -0.01;
    }

    ContactPoint reducedPoints[4];
    int reducedCount = ManifoldReduction::Reduce(rawPoints, 8, normal, reducedPoints);

    while (!renderer.ShouldClose())
    {
        renderer.BeginFrame();

        // Draw ground grid
        DebugDraw::Line({ -5.0f, 0.0f, 0.0f }, { 5.0f, 0.0f, 0.0f }, { 0.2f, 0.2f, 0.2f });
        DebugDraw::Line({ 0.0f, 0.0f, -5.0f }, { 0.0f, 0.0f, 5.0f }, { 0.2f, 0.2f, 0.2f });

        // Draw all 8 original points in gray/red
        for (int i = 0; i < 8; ++i)
        {
            glm::vec3 pos = rawPoints[i].position;
            glm::vec3 col = (rawPoints[i].signedDistance < -0.02) ? glm::vec3(1.0f, 0.2f, 0.2f) : glm::vec3(0.5f, 0.5f, 0.5f);
            DebugDraw::Sphere(pos, 0.1f, col, 8);
        }

        // Draw reduced polygon in bright cyan
        for (int i = 0; i < reducedCount; ++i)
        {
            glm::vec3 p1 = reducedPoints[i].position;
            glm::vec3 p2 = reducedPoints[(i + 1) % reducedCount].position;
            DebugDraw::Line(p1, p2, { 0.0f, 1.0f, 1.0f });
            DebugDraw::Sphere(p1, 0.18f, { 0.0f, 1.0f, 1.0f }, 12);
            DebugDraw::Line(p1, p1 + glm::vec3(0.0f, 0.8f, 0.0f), { 1.0f, 1.0f, 0.0f });
        }

        renderer.FlushDebugDraw();
        renderer.EndFrame();
    }

    return 0;
}