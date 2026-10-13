/**
 * @file IterativeSolver.cpp
 * @brief Implementation of Projected Gauss-Seidel solver passes with split impulses.
 */

#include "RT-PhysicsCore/physics/solver/IterativeSolver.h"
#include <algorithm>
#include <cmath>

namespace RT_PhysicsCore
{
    namespace
    {
        constexpr Real kSolverEpsilon = 1e-12;
    }

    IterativeSolver::IterativeSolver()
    {}

    IterativeSolver::~IterativeSolver()
    {}

    void IterativeSolver::SetVelocityIterations(int iterations)
    {
        velocityIterations = iterations;
    }

    void IterativeSolver::SetPositionIterations(int iterations)
    {
        positionIterations = iterations;
    }

    void IterativeSolver::SetSlop(Real slop)
    {
        slopThreshold = slop;
    }

    void IterativeSolver::SetBeta(Real beta)
    {
        positionBeta = beta;
    }

    void IterativeSolver::SetMaxPushSpeed(Real speed)
    {
        maxPushVelocity = speed;
    }

    void IterativeSolver::SetRestitutionVelocityThreshold(Real threshold)
    {
        restitutionVelocityThreshold = threshold;
    }

    Real IterativeSolver::AngularEffectiveMass(const Vec3& r, const Vec3& axis, const Mat3& invInertiaWorld)
    {
        return glm::dot(axis, glm::cross(invInertiaWorld * glm::cross(r, axis), r));
    }

    Real IterativeSolver::CrossEffectiveMass(const Vec3& r, const Vec3& a1, const Vec3& a2, const Mat3& invInertiaWorld)
    {
        return glm::dot(a1, glm::cross(invInertiaWorld * glm::cross(r, a2), r));
    }

    void IterativeSolver::BuildTangentBasis(const Vec3& normal, Vec3& outT1, Vec3& outT2)
    {
        Vec3 arbitrary = (std::abs(normal.x) < 0.9) ? Vec3(1.0, 0.0, 0.0) : Vec3(0.0, 1.0, 0.0);
        outT1 = glm::normalize(glm::cross(arbitrary, normal));
        outT2 = glm::cross(normal, outT1);
    }

    void IterativeSolver::PrepareConstraints(SimulationIsland& island, std::vector<ContactConstraint>& constraints, double h)
    {
        constraints.clear();
        constraints.reserve(island.contacts.size());

        for (Contact* contactPtr : island.contacts)
        {
            if (!contactPtr || contactPtr->pointCount == 0)
            {
                continue;
            }

            Contact& c = *contactPtr;
            ContactConstraint cc;
            cc.originalContact = &c;
            cc.normal = c.normal;
            BuildTangentBasis(cc.normal, cc.tangent1, cc.tangent2);
            c.tangent1 = cc.tangent1;
            c.tangent2 = cc.tangent2;

            cc.points.resize(c.pointCount);

            for (int i = 0; i < c.pointCount; ++i)
            {
                const ContactPoint& cp = c.points[i];
                ConstraintPoint& p = cc.points[i];

                p.rA = cp.position;
                p.rB = cp.position;
                p.restitution = c.restitution;
                p.staticFriction = c.staticFriction;
                p.kineticFriction = c.kineticFriction;
                p.isStuck = cp.isStuck;

                p.normalImpulse = cp.normalImpulse;
                p.tangentImpulse1 = cp.tangentImpulse1;
                p.tangentImpulse2 = cp.tangentImpulse2;
                p.pseudoNormalImpulse = 0.0;

                p.speculativeBias = (cp.signedDistance > 0.0) ? (cp.signedDistance / h) : 0.0;

                Real penetration = -cp.signedDistance;
                p.pseudoVelocityTarget = (penetration > slopThreshold) ?
                    std::min(positionBeta * (penetration - slopThreshold) / h, maxPushVelocity) : 0.0;
            }

            constraints.push_back(std::move(cc));
        }
    }

    void IterativeSolver::SolveVelocity(std::vector<ContactConstraint>& constraints, double /*h*/)
    {
        // 1. Warm start: Apply cached impulses
        for (auto& cc : constraints)
        {
            RigidBodyComponent* bA = cc.bodyA;
            RigidBodyComponent* bB = cc.bodyB;

            for (auto& p : cc.points)
            {
                Vec3 totalImpulse = p.normalImpulse * cc.normal + p.tangentImpulse1 * cc.tangent1 + p.tangentImpulse2 * cc.tangent2;

                if (bA && bA->invMass > 0.0)
                {
                    bA->velocity -= bA->invMass * totalImpulse;
                    bA->angularMomentum -= glm::cross(p.rA, totalImpulse);
                    bA->angularVelocity = bA->invInertiaWorld * bA->angularMomentum;
                }
                if (bB && bB->invMass > 0.0)
                {
                    bB->velocity += bB->invMass * totalImpulse;
                    bB->angularMomentum += glm::cross(p.rB, totalImpulse);
                    bB->angularVelocity = bB->invInertiaWorld * bB->angularMomentum;
                }
            }
        }

        // 2. Iterative Projected Gauss-Seidel sweeps
        for (int iter = 0; iter < velocityIterations; ++iter)
        {
            Real maxImpulseDelta = 0.0;
            bool reverseOrder = (iter % 2 == 1);

            int startIdx = reverseOrder ? (static_cast<int>(constraints.size()) - 1) : 0;
            int endIdx = reverseOrder ? -1 : static_cast<int>(constraints.size());
            int step = reverseOrder ? -1 : 1;

            for (int cIdx = startIdx; cIdx != endIdx; cIdx += step)
            {
                ContactConstraint& cc = constraints[cIdx];
                RigidBodyComponent* bA = cc.bodyA;
                RigidBodyComponent* bB = cc.bodyB;

                Real invMassA = bA ? bA->invMass : 0.0;
                Real invMassB = bB ? bB->invMass : 0.0;
                Mat3 invInertiaA = bA ? bA->invInertiaWorld : Mat3(0.0);
                Mat3 invInertiaB = bB ? bB->invInertiaWorld : Mat3(0.0);

                for (auto& p : cc.points)
                {
                    // Compute current relative contact velocity
                    Vec3 vA = bA ? (bA->velocity + glm::cross(bA->angularVelocity, p.rA)) : Vec3(0.0);
                    Vec3 vB = bB ? (bB->velocity + glm::cross(bB->angularVelocity, p.rB)) : Vec3(0.0);
                    Vec3 relVel = vB - vA;

                    // A. Friction-first solve (coupled 2D circular Coulomb cone)
                    Real vt1 = glm::dot(relVel, cc.tangent1);
                    Real vt2 = glm::dot(relVel, cc.tangent2);

                    Real Kt1t1 = invMassA + invMassB + AngularEffectiveMass(p.rA, cc.tangent1, invInertiaA) + AngularEffectiveMass(p.rB, cc.tangent1, invInertiaB);
                    Real Kt2t2 = invMassA + invMassB + AngularEffectiveMass(p.rA, cc.tangent2, invInertiaA) + AngularEffectiveMass(p.rB, cc.tangent2, invInertiaB);
                    Real Kt1t2 = CrossEffectiveMass(p.rA, cc.tangent1, cc.tangent2, invInertiaA) + CrossEffectiveMass(p.rB, cc.tangent1, cc.tangent2, invInertiaB);

                    Real det = Kt1t1 * Kt2t2 - Kt1t2 * Kt1t2;
                    Real dLambdaT1 = 0.0;
                    Real dLambdaT2 = 0.0;

                    if (std::abs(det) > kSolverEpsilon)
                    {
                        dLambdaT1 = (-vt1 * Kt2t2 + Kt1t2 * vt2) / det;
                        dLambdaT2 = (-Kt1t1 * vt2 + vt1 * Kt1t2) / det;
                    }
                    else
                    {
                        dLambdaT1 = (Kt1t1 > kSolverEpsilon) ? (-vt1 / Kt1t1) : 0.0;
                        dLambdaT2 = (Kt2t2 > kSolverEpsilon) ? (-vt2 / Kt2t2) : 0.0;
                    }

                    Real newT1 = p.tangentImpulse1 + dLambdaT1;
                    Real newT2 = p.tangentImpulse2 + dLambdaT2;

                    Real frictionCoeff = p.isStuck ? p.staticFriction : p.kineticFriction;
                    Real maxFriction = frictionCoeff * p.normalImpulse;
                    Real frictionMag = std::sqrt(newT1 * newT1 + newT2 * newT2);

                    if (frictionMag > maxFriction)
                    {
                        Real scale = (frictionMag > kSolverEpsilon) ? (maxFriction / frictionMag) : 0.0;
                        newT1 *= scale;
                        newT2 *= scale;
                        p.isStuck = false;
                    }
                    else
                    {
                        p.isStuck = true;
                    }

                    Real deltaT1 = newT1 - p.tangentImpulse1;
                    Real deltaT2 = newT2 - p.tangentImpulse2;
                    p.tangentImpulse1 = newT1;
                    p.tangentImpulse2 = newT2;

                    Vec3 tangentImpulse = deltaT1 * cc.tangent1 + deltaT2 * cc.tangent2;
                    if (bA && bA->invMass > 0.0)
                    {
                        bA->velocity -= bA->invMass * tangentImpulse;
                        bA->angularMomentum -= glm::cross(p.rA, tangentImpulse);
                        bA->angularVelocity = bA->invInertiaWorld * bA->angularMomentum;
                    }
                    if (bB && bB->invMass > 0.0)
                    {
                        bB->velocity += bB->invMass * tangentImpulse;
                        bB->angularMomentum += glm::cross(p.rB, tangentImpulse);
                        bB->angularVelocity = bB->invInertiaWorld * bB->angularMomentum;
                    }

                    // Refresh velocities after friction
                    vA = bA ? (bA->velocity + glm::cross(bA->angularVelocity, p.rA)) : Vec3(0.0);
                    vB = bB ? (bB->velocity + glm::cross(bB->angularVelocity, p.rB)) : Vec3(0.0);
                    relVel = vB - vA;

                    // B. Normal non-penetration solve
                    Real vn = glm::dot(relVel, cc.normal);
                    Real Kn = invMassA + invMassB + AngularEffectiveMass(p.rA, cc.normal, invInertiaA) + AngularEffectiveMass(p.rB, cc.normal, invInertiaB);

                    if (Kn > kSolverEpsilon)
                    {
                        Real lambdaN = -(vn + p.speculativeBias) / Kn;
                        Real newNormal = std::max(0.0, p.normalImpulse + lambdaN);
                        Real deltaN = newNormal - p.normalImpulse;
                        p.normalImpulse = newNormal;

                        Vec3 normalImpulse = deltaN * cc.normal;
                        if (bA && bA->invMass > 0.0)
                        {
                            bA->velocity -= bA->invMass * normalImpulse;
                            bA->angularMomentum -= glm::cross(p.rA, normalImpulse);
                            bA->angularVelocity = bA->invInertiaWorld * bA->angularMomentum;
                        }
                        if (bB && bB->invMass > 0.0)
                        {
                            bB->velocity += bB->invMass * normalImpulse;
                            bB->angularMomentum += glm::cross(p.rB, normalImpulse);
                            bB->angularVelocity = bB->invInertiaWorld * bB->angularMomentum;
                        }

                        maxImpulseDelta = std::max(maxImpulseDelta, std::abs(deltaN));
                    }
                }
            }

            if (maxImpulseDelta < convergenceTolerance)
            {
                break;
            }
        }

        // Write converged impulses back to source contact manifold
        for (auto& cc : constraints)
        {
            if (cc.originalContact)
            {
                for (size_t i = 0; i < cc.points.size(); ++i)
                {
                    cc.originalContact->points[i].normalImpulse = cc.points[i].normalImpulse;
                    cc.originalContact->points[i].tangentImpulse1 = cc.points[i].tangentImpulse1;
                    cc.originalContact->points[i].tangentImpulse2 = cc.points[i].tangentImpulse2;
                    cc.originalContact->points[i].isStuck = cc.points[i].isStuck;
                }
            }
        }
    }

    void IterativeSolver::SolveRestitution(std::vector<ContactConstraint>& constraints, double /*h*/)
    {
        // Multi-pass shockwave propagation for chains of elastic touching bodies (e.g. Newton's Cradle)
        int maxPasses = std::min(10, std::max(4, static_cast<int>(constraints.size())));

        for (int pass = 0; pass < maxPasses; ++pass)
        {
            bool anyImpact = false;

            for (auto& cc : constraints)
            {
                RigidBodyComponent* bA = cc.bodyA;
                RigidBodyComponent* bB = cc.bodyB;

                Real invMassA = bA ? bA->invMass : 0.0;
                Real invMassB = bB ? bB->invMass : 0.0;
                Mat3 invInertiaA = bA ? bA->invInertiaWorld : Mat3(0.0);
                Mat3 invInertiaB = bB ? bB->invInertiaWorld : Mat3(0.0);

                for (auto& p : cc.points)
                {
                    if (p.restitution <= 0.0)
                    {
                        continue;
                    }

                    // Dynamically evaluate incoming normal velocity at point
                    Vec3 vA = bA ? (bA->velocity + glm::cross(bA->angularVelocity, p.rA)) : Vec3(0.0);
                    Vec3 vB = bB ? (bB->velocity + glm::cross(bB->angularVelocity, p.rB)) : Vec3(0.0);
                    Real vn = glm::dot(vB - vA, cc.normal);

                    // Only trigger if bodies are closing faster than the restitution threshold
                    if (vn >= -restitutionVelocityThreshold)
                    {
                        continue;
                    }

                    Real Kn = invMassA + invMassB + AngularEffectiveMass(p.rA, cc.normal, invInertiaA) + AngularEffectiveMass(p.rB, cc.normal, invInertiaB);
                    if (Kn > kSolverEpsilon)
                    {
                        Real targetVn = -p.restitution * vn;
                        Real delta = (targetVn - vn) / Kn;
                        Vec3 impulse = delta * cc.normal;

                        if (bA && bA->invMass > 0.0)
                        {
                            bA->velocity -= bA->invMass * impulse;
                            bA->angularMomentum -= glm::cross(p.rA, impulse);
                            bA->angularVelocity = bA->invInertiaWorld * bA->angularMomentum;
                        }
                        if (bB && bB->invMass > 0.0)
                        {
                            bB->velocity += bB->invMass * impulse;
                            bB->angularMomentum += glm::cross(p.rB, impulse);
                            bB->angularVelocity = bB->invInertiaWorld * bB->angularMomentum;
                        }

                        anyImpact = true;
                    }
                }
            }

            if (!anyImpact)
            {
                break;
            }
        }
    }

    void IterativeSolver::SolvePenetration(std::vector<ContactConstraint>& constraints, double /*h*/)
    {
        for (int iter = 0; iter < positionIterations; ++iter)
        {
            for (auto& cc : constraints)
            {
                RigidBodyComponent* bA = cc.bodyA;
                RigidBodyComponent* bB = cc.bodyB;

                Real invMassA = bA ? bA->invMass : 0.0;
                Real invMassB = bB ? bB->invMass : 0.0;
                Mat3 invInertiaA = bA ? bA->invInertiaWorld : Mat3(0.0);
                Mat3 invInertiaB = bB ? bB->invInertiaWorld : Mat3(0.0);

                for (auto& p : cc.points)
                {
                    if (p.pseudoVelocityTarget <= 0.0)
                    {
                        continue;
                    }

                    Vec3 vA = bA ? (bA->pseudoVelocity + glm::cross(bA->pseudoAngularVelocity, p.rA)) : Vec3(0.0);
                    Vec3 vB = bB ? (bB->pseudoVelocity + glm::cross(bB->pseudoAngularVelocity, p.rB)) : Vec3(0.0);
                    Real vpn = glm::dot(vB - vA, cc.normal);

                    Real Kn = invMassA + invMassB + AngularEffectiveMass(p.rA, cc.normal, invInertiaA) + AngularEffectiveMass(p.rB, cc.normal, invInertiaB);
                    if (Kn > kSolverEpsilon)
                    {
                        Real lambda = (p.pseudoVelocityTarget - vpn) / Kn;
                        Real newImpulse = std::max(0.0, p.pseudoNormalImpulse + lambda);
                        Real delta = newImpulse - p.pseudoNormalImpulse;
                        p.pseudoNormalImpulse = newImpulse;

                        Vec3 impulse = delta * cc.normal;
                        if (bA && bA->invMass > 0.0)
                        {
                            bA->pseudoVelocity -= bA->invMass * impulse;
                            bA->pseudoAngularVelocity -= invInertiaA * glm::cross(p.rA, impulse);
                        }
                        if (bB && bB->invMass > 0.0)
                        {
                            bB->pseudoVelocity += bB->invMass * impulse;
                            bB->pseudoAngularVelocity += invInertiaB * glm::cross(p.rB, impulse);
                        }
                    }
                }
            }
        }
    }
}