/**
 * @file MassPropertiesTest.cpp
 * @brief Validates analytical inertia tensor computations and rigid body factory functions.
 *
 * Compares computed inertia diagonals against hand-calculated values for all supported
 * primitives, verifies I * I^-1 = Identity, and checks factory behavior.
 */

#include <chrono>
#include <cmath>
#include <sstream>
#include <string>
#include <vector>

#include "RT-PhysicsCore/utils/Log.h"
#include "RT-PhysicsCore/utils/DebugDraw.h"
#include "RT-PhysicsCore/rendering/Renderer.h"
#include "RT-PhysicsCore/physics/MassProperties.h"
#include "RT-PhysicsCore/physics/components/RigidBodyComponent.h"

#include <glm/glm.hpp>

namespace
{
    using namespace RT_PhysicsCore;

    struct InertiaTestCase
    {
        std::string name;
        glm::mat3 computed;
        glm::vec3 expectedDiagonal;
        float tolerance;
    };

    struct FactoryTestCase
    {
        std::string name;
        RigidBodyComponent body;
        float expectedInvMass;
        bool expectZeroInertia;
    };

    bool DiagonalApprox(const glm::mat3& m, const glm::vec3& diag, float tol)
    {
        return std::abs(m[0][0] - diag.x) < tol
            && std::abs(m[1][1] - diag.y) < tol
            && std::abs(m[2][2] - diag.z) < tol;
    }

    bool IsIdentityApprox(const glm::mat3& m, float tol)
    {
        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                float expected = (i == j) ? 1.0f : 0.0f;
                if (std::abs(m[i][j] - expected) > tol)
                {
                    return false;
                }
            }
        }
        return true;
    }

    float MaxOffDiagonal(const glm::mat3& m)
    {
        float maxVal = 0.0f;
        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                if (i != j)
                {
                    maxVal = std::max(maxVal, std::abs(m[i][j]));
                }
            }
        }
        return maxVal;
    }

    struct VisualShape
    {
        std::string name;
        glm::vec3 inertiaDiag;
        glm::vec3 shapeColor;
        // For drawing: 0=sphere, 1=box, 2=capsule
        int shapeType;
        glm::vec3 shapeSize;
    };
}

int main()
{
    RT_LOG_INFO("Mass Properties Test harness starting.");

    // ── Phase 1: Automated assertions ──

    int passCount = 0;
    int totalCount = 0;

    // Inertia tensor tests
    std::vector<InertiaTestCase> inertiaCases;

    inertiaCases.push_back({ "Unit sphere (m=1, r=1)",
        ComputeSphereInertia(1.0f, 1.0f),
        {0.4f, 0.4f, 0.4f}, 0.001f });

    inertiaCases.push_back({ "Heavy sphere (m=10, r=0.5)",
        ComputeSphereInertia(10.0f, 0.5f),
        {1.0f, 1.0f, 1.0f}, 0.001f });

    inertiaCases.push_back({ "Unit cube (m=1, half=[0.5,0.5,0.5])",
        ComputeBoxInertia(1.0f, {0.5f, 0.5f, 0.5f}),
        {0.1667f, 0.1667f, 0.1667f}, 0.001f });

    inertiaCases.push_back({ "Thin rod box (m=1, half=[0.05,0.05,2.0])",
        ComputeBoxInertia(1.0f, {0.05f, 0.05f, 2.0f}),
        {1.334f, 1.334f, 0.00167f}, 0.01f });

    inertiaCases.push_back({ "Flat pancake (m=1, half=[2,0.05,2])",
        ComputeBoxInertia(1.0f, {2.0f, 0.05f, 2.0f}),
        {1.334f, 2.667f, 1.334f}, 0.01f });

    inertiaCases.push_back({ "Cylinder (m=1, r=0.5, h=2)",
        ComputeCylinderInertia(1.0f, 0.5f, 2.0f),
        {0.3958f, 0.125f, 0.3958f}, 0.01f });

    inertiaCases.push_back({ "Capsule (m=1, r=0.3, hl=1.0)",
        ComputeCapsuleInertia(1.0f, 0.3f, 1.0f),
        // Approximate: verify symmetry Ixx == Izz and Iyy < Ixx
        {-1.0f, -1.0f, -1.0f}, 0.0f }); // special case: check symmetry instead

    for (auto& tc : inertiaCases)
    {
        ++totalCount;
        bool pass;
        std::ostringstream oss;

        if (tc.expectedDiagonal.x < 0.0f)
        {
            // Special case: check symmetry only (capsule)
            float ix = tc.computed[0][0];
            float iy = tc.computed[1][1];
            float iz = tc.computed[2][2];
            bool symmetric = std::abs(ix - iz) < 0.001f;
            bool axialSmaller = iy < ix;
            pass = symmetric && axialSmaller;
            oss << "I_diag=[" << ix << "," << iy << "," << iz << "]"
                << " Ixx==Izz:" << symmetric << " Iyy<Ixx:" << axialSmaller;
        }
        else
        {
            pass = DiagonalApprox(tc.computed, tc.expectedDiagonal, tc.tolerance);
            oss << "I_diag=[" << tc.computed[0][0] << "," << tc.computed[1][1] << "," << tc.computed[2][2] << "]"
                << " expected=[" << tc.expectedDiagonal.x << "," << tc.expectedDiagonal.y << "," << tc.expectedDiagonal.z << "]";
        }

        if (pass)
        {
            ++passCount;
        }
        RT_LOG_INFO((pass ? "[PASS] " : "[FAIL] ") << tc.name << " - " << oss.str());
    }

    // All inertia matrices should be diagonal (off-diagonal == 0)
    {
        ++totalCount;
        float maxOff = 0.0f;
        for (const auto& tc : inertiaCases)
        {
            maxOff = std::max(maxOff, MaxOffDiagonal(tc.computed));
        }
        bool pass = maxOff < 0.0001f;
        if (pass) ++passCount;
        std::ostringstream oss;
        oss << "max_off_diagonal=" << maxOff;
        RT_LOG_INFO((pass ? "[PASS] " : "[FAIL] ") << "All matrices are diagonal - " << oss.str());
    }

    // I * I^-1 = Identity for each shape
    {
        ++totalCount;
        float maxErr = 0.0f;
        for (const auto& tc : inertiaCases)
        {
            glm::mat3 invI = glm::inverse(tc.computed);
            glm::mat3 product = tc.computed * invI;
            for (int i = 0; i < 3; ++i)
                for (int j = 0; j < 3; ++j)
                    maxErr = std::max(maxErr, std::abs(product[i][j] - (i == j ? 1.0f : 0.0f)));
        }
        bool pass = maxErr < 0.001f;
        if (pass) ++passCount;
        std::ostringstream oss;
        oss << "max_err=" << maxErr;
        RT_LOG_INFO((pass ? "[PASS] " : "[FAIL] ") << "I * I^-1 = Identity - " << oss.str());
    }

    // Sphere is isotropic
    {
        ++totalCount;
        glm::mat3 I = ComputeSphereInertia(1.0f, 1.0f);
        bool pass = std::abs(I[0][0] - I[1][1]) < 0.0001f && std::abs(I[1][1] - I[2][2]) < 0.0001f;
        if (pass) ++passCount;
        RT_LOG_INFO((pass ? "[PASS] " : "[FAIL] ") << "Sphere is isotropic - Ixx=Iyy=Izz=" << I[0][0]);
    }

    // MakeDynamicBody factory
    {
        ++totalCount;
        glm::mat3 I = ComputeSphereInertia(5.0f, 1.0f);
        RigidBodyComponent rb = MakeDynamicBody(5.0f, I);
        bool pass = std::abs(rb.invMass - 0.2f) < 0.001f;
        if (pass) ++passCount;
        std::ostringstream oss;
        oss << "invMass=" << rb.invMass << " expected=0.2";
        RT_LOG_INFO((pass ? "[PASS] " : "[FAIL] ") << "MakeDynamicBody(5.0) - " << oss.str());
    }

    // MakeStaticBody factory
    {
        ++totalCount;
        RigidBodyComponent rb = MakeStaticBody();
        bool pass = rb.invMass == 0.0f;
        if (pass) ++passCount;
        RT_LOG_INFO((pass ? "[PASS] " : "[FAIL] ") << "MakeStaticBody - invMass=" << rb.invMass);
    }

    RT_LOG_INFO(passCount << "/" << totalCount << " automated checks passed.");

    // ── Phase 2: Visual - principal axis lengths ──

    Renderer renderer(1280, 720, "RT-PhysicsCore Mass Properties Test");
    if (!renderer.IsValid())
    {
        RT_LOG_FATAL("Renderer failed to initialize.");
        return 1;
    }

    struct VisualCase
    {
        std::string name;
        glm::vec3 inertiaDiag;
        // For drawing shape: 0=sphere, 1=box
        int shapeType;
        glm::vec3 shapeParam;
    };

    std::vector<VisualCase> visuals = {
        {"Unit sphere",  {0.4f, 0.4f, 0.4f},       0, {1.0f, 0, 0}},
        {"Unit cube",    {0.1667f, 0.1667f, 0.1667f}, 1, {0.5f, 0.5f, 0.5f}},
        {"Thin rod",     {1.334f, 1.334f, 0.00167f},  1, {0.05f, 0.05f, 2.0f}},
        {"Flat pancake", {1.334f, 2.667f, 1.334f},    1, {2.0f, 0.05f, 2.0f}},
        {"Cylinder",     {0.3958f, 0.125f, 0.3958f},  2, {0.5f, 1.0f, 0.0f}},
    };

    size_t current = 0;
    double elapsed = 0.0;
    constexpr double kSecondsPerCase = 4.0;
    auto lastTime = std::chrono::steady_clock::now();

    RT_LOG_INFO("Showing: " << visuals[0].name);

    while (!renderer.ShouldClose())
    {
        auto now = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(now - lastTime).count();
        lastTime = now;
        elapsed += dt;

        if (elapsed >= kSecondsPerCase)
        {
            elapsed = 0.0;
            current = (current + 1) % visuals.size();
            RT_LOG_INFO("Showing: " << visuals[current].name);
        }

        const auto& vis = visuals[current];

        // Draw shape wireframe
        glm::vec3 center(0.0f);
        if (vis.shapeType == 0)
        {
            DebugDraw::Sphere(center, vis.shapeParam.x, { 0.4f, 0.4f, 0.6f });
        }
        else if (vis.shapeType == 1)
        {
            DebugDraw::Box(center, vis.shapeParam, { 0.4f, 0.4f, 0.6f });
        }
        else if (vis.shapeType == 2)
        {
            DebugDraw::Cylinder(center, vis.shapeParam.x, vis.shapeParam.y, { 0.4f, 0.4f, 0.6f });
        } 

        // Draw principal inertia axes as colored lines from center
        // Length proportional to inertia value, scaled so max axis = 3 units
        float maxI = std::max({ vis.inertiaDiag.x, vis.inertiaDiag.y, vis.inertiaDiag.z });
        float scale = (maxI > 0.001f) ? 3.0f / maxI : 1.0f;

        float lx = vis.inertiaDiag.x * scale;
        float ly = vis.inertiaDiag.y * scale;
        float lz = vis.inertiaDiag.z * scale;

        // X axis (red)
        DebugDraw::Line(center, center + glm::vec3(lx, 0, 0), { 1.0f, 0.2f, 0.2f }, false);
        DebugDraw::Line(center, center - glm::vec3(lx, 0, 0), { 1.0f, 0.2f, 0.2f }, false);
        // Y axis (green)
        DebugDraw::Line(center, center + glm::vec3(0, ly, 0), { 0.2f, 1.0f, 0.2f }, false);
        DebugDraw::Line(center, center - glm::vec3(0, ly, 0), { 0.2f, 1.0f, 0.2f }, false);
        // Z axis (blue)
        DebugDraw::Line(center, center + glm::vec3(0, 0, lz), { 0.2f, 0.2f, 1.0f }, false);
        DebugDraw::Line(center, center - glm::vec3(0, 0, lz), { 0.2f, 0.2f, 1.0f }, false);

        // Axis endpoint markers
        DebugDraw::Sphere(center + glm::vec3(lx, 0, 0), 0.08f, { 1, 0, 0 }, 6, false);
        DebugDraw::Sphere(center + glm::vec3(0, ly, 0), 0.08f, { 0, 1, 0 }, 6, false);
        DebugDraw::Sphere(center + glm::vec3(0, 0, lz), 0.08f, { 0, 0, 1 }, 6, false);

        renderer.BeginFrame();
        renderer.FlushDebugDraw();
        renderer.EndFrame();
    }

    return 0;
}