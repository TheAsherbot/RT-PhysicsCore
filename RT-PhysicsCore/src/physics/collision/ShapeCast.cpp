/**
 * @file ShapeCast.cpp
 * @brief Implementation of bilateral conservative advancement time-of-impact queries.
 */

#include "RT-PhysicsCore/physics/collision/ShapeCast.h"
#include <algorithm>
#include <cmath>

namespace RT_PhysicsCore
{
    namespace
    {
        constexpr Real kCastEpsilon = 1e-12;

        Mat3 IntegrateRotation(const Mat3& initialRot, const Vec3& omega, Real dt)
        {
            Real angle = glm::length(omega) * dt;
            if (angle < kCastEpsilon)
            {
                return initialRot;
            }

            Vec3 axis = glm::normalize(omega);
            Quat dq = glm::angleAxis(angle, axis);
            Quat q = glm::normalize(dq * glm::quat_cast(initialRot));
            return glm::mat3_cast(q);
        }
    }

    ShapeCastResult ShapeCast::Cast(
        const ConvexCore& coreA, const Vec3& linDispA, const Vec3& angVelA, Real boundRadiusA,
        const ConvexCore& coreB, const Vec3& linDispB, const Vec3& angVelB, Real boundRadiusB,
        double h, Real targetGap, int maxIterations)
    {
        ShapeCastResult result;
        Real t = 0.0;

        ConvexCore currentCoreA = coreA;
        ConvexCore currentCoreB = coreB;

        Real angSpeedA = glm::length(angVelA);
        Real angSpeedB = glm::length(angVelB);

        for (int iter = 0; iter < maxIterations; ++iter)
        {
            result.iterations++;

            // Interpolate poses to current advancement fraction t
            currentCoreA.position = coreA.position + linDispA * t;
            currentCoreA.rotation = IntegrateRotation(coreA.rotation, angVelA, t * h);

            currentCoreB.position = coreB.position + linDispB * t;
            currentCoreB.rotation = IntegrateRotation(coreB.rotation, angVelB, t * h);

            // Evaluate high-precision GJK distance at time t
            GjkResult gjk = GJK::ComputeDistance(currentCoreA, currentCoreB);

            // Relative linear velocity displacement vector: delta_disp = linDispB - linDispA
            // Closing velocity component along collision normal (normal directed A -> B):
            Vec3 relDisp = linDispB - linDispA;
            Real linearClosingBound = std::max(0.0, -glm::dot(relDisp, gjk.normal));

            // Angular closing bound from rotational sweep of extremal vertices: ||omega|| * r * h
            Real angularClosingBound = (angSpeedA * boundRadiusA + angSpeedB * boundRadiusB) * h;
            Real totalClosingBound = linearClosingBound + angularClosingBound;

            if (totalClosingBound < kCastEpsilon)
            {
                // Shapes are separating or stationary relative to normal; no collision along this trajectory
                result.hasHit = false;
                result.timeOfImpact = 1.0;
                return result;
            }

            if (gjk.distance <= targetGap)
            {
                // If distance is already <= targetGap at the start of the substep (t < kCastEpsilon),
                // the bodies started in contact. Discrete collision resolution handles resting/touching contacts.
                // Reporting a CCD hit at t = 0.0 would clamp motion fraction to 0, permanently freezing bodies.
                if (t < kCastEpsilon)
                {
                    result.hasHit = false;
                    result.timeOfImpact = 1.0;
                    return result;
                }

                // Dynamic impact detected during advancement within security skin
                result.hasHit = true;
                result.timeOfImpact = t;
                result.normal = gjk.normal;
                result.contactPoint = 0.5 * (gjk.closestPointA + gjk.closestPointB);
                return result;
            }

            // Conservative advancement time step: delta_t = (d - targetGap) / closingBound
            Real deltaT = (gjk.distance - targetGap) / totalClosingBound;
            if (deltaT < kCastEpsilon)
            {
                break;
            }

            t += deltaT;

            if (t >= 1.0)
            {
                // Trajectory completed the full substep interval without impact
                result.hasHit = false;
                result.timeOfImpact = 1.0;
                return result;
            }
        }

        // Reached iteration cap without impact; trajectory is clear up to 1.0
        result.hasHit = false;
        result.timeOfImpact = 1.0;
        return result;
    }
}