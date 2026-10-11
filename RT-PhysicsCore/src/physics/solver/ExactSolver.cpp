/**
 * @file ExactSolver.cpp
 * @brief Implementation of exact Delassus LCP Lemke solver with outer-loop Coulomb friction.
 */

#include "RT-PhysicsCore/physics/solver/ExactSolver.h"
#include "RT-PhysicsCore/physics/LCPSolver.h"
#include <algorithm>
#include <cmath>
#include <unordered_map>

namespace RT_PhysicsCore
{
    ExactSolver::ExactSolver()
    {}

    ExactSolver::~ExactSolver()
    {}

    void ExactSolver::SetExactMaxPoints(int maxPoints)
    {
        exactMaxPoints = maxPoints;
    }

    void ExactSolver::SetMaxOuterFrictionPasses(int passes)
    {
        maxOuterFrictionPasses = passes;
    }

    void ExactSolver::SetFrictionTolerance(Real tol)
    {
        frictionTolerance = tol;
    }

    Vec3 ExactSolver::ImpulseEffect(Real invMass, const Mat3& invInertiaWorld,
        const Vec3& rImpulse, const Vec3& rQuery, const Vec3& impulse)
    {
        Vec3 dv = invMass * impulse;
        Vec3 dOmega = invInertiaWorld * glm::cross(rImpulse, impulse);
        return dv + glm::cross(dOmega, rQuery);
    }

    void ExactSolver::PrepareConstraints(SimulationIsland& island, std::vector<ContactConstraint>& constraints, double h)
    {
        fallbackSolver.PrepareConstraints(island, constraints, h);
    }

    void ExactSolver::SolveVelocity(std::vector<ContactConstraint>& constraints, double h)
    {
        // Flatten active contact points
        std::vector<PointRef> pointRefs;
        for (auto& cc : constraints)
        {
            for (size_t i = 0; i < cc.points.size(); ++i)
            {
                pointRefs.push_back({ &cc, static_cast<int>(i) });
            }
        }

        int n = static_cast<int>(pointRefs.size());
        if (n == 0)
        {
            return;
        }

        // Delegate large islands to iterative solver to guarantee bounded execution
        if (n > exactMaxPoints)
        {
            fallbackSolver.SolveVelocity(constraints, h);
            return;
        }

        // 1. Snapshot initial island body momentum states
        std::unordered_map<RigidBodyComponent*, BodyState> initialStates;
        for (auto& cc : constraints)
        {
            if (cc.bodyA && initialStates.find(cc.bodyA) == initialStates.end())
            {
                initialStates[cc.bodyA] = { cc.bodyA->velocity, cc.bodyA->angularMomentum, cc.bodyA->angularVelocity };
            }
            if (cc.bodyB && initialStates.find(cc.bodyB) == initialStates.end())
            {
                initialStates[cc.bodyB] = { cc.bodyB->velocity, cc.bodyB->angularMomentum, cc.bodyB->angularVelocity };
            }
        }

        // 2. Assemble Delassus matrix M (size n * n)
        std::vector<Real> M(n * n, 0.0);
        for (int i = 0; i < n; ++i)
        {
            const PointRef& pi = pointRefs[i];
            const ContactConstraint* cci = pi.constraint;
            const ConstraintPoint& cpi = cci->points[pi.pointIndex];

            for (int j = 0; j < n; ++j)
            {
                const PointRef& pj = pointRefs[j];
                const ContactConstraint* ccj = pj.constraint;
                const ConstraintPoint& cpj = ccj->points[pj.pointIndex];

                Vec3 impulseOnB = ccj->normal;
                Vec3 impulseOnA = -ccj->normal;
                Real total = 0.0;

                if (cci->bodyB && cci->bodyB == ccj->bodyB)
                {
                    total += glm::dot(ImpulseEffect(cci->bodyB->invMass, cci->bodyB->invInertiaWorld, cpj.rB, cpi.rB, impulseOnB), cci->normal);
                }
                if (cci->bodyB && cci->bodyB == ccj->bodyA)
                {
                    total += glm::dot(ImpulseEffect(cci->bodyB->invMass, cci->bodyB->invInertiaWorld, cpj.rA, cpi.rB, impulseOnA), cci->normal);
                }
                if (cci->bodyA && cci->bodyA == ccj->bodyB)
                {
                    total -= glm::dot(ImpulseEffect(cci->bodyA->invMass, cci->bodyA->invInertiaWorld, cpj.rB, cpi.rA, impulseOnB), cci->normal);
                }
                if (cci->bodyA && cci->bodyA == ccj->bodyA)
                {
                    total -= glm::dot(ImpulseEffect(cci->bodyA->invMass, cci->bodyA->invInertiaWorld, cpj.rA, cpi.rA, impulseOnA), cci->normal);
                }

                M[i * n + j] = total;
            }
        }

        // 3. Outer loop: Alternate normal Lemke solve and circular friction passes
        std::vector<Real> z(n, 0.0);
        for (int i = 0; i < n; ++i)
        {
            z[i] = pointRefs[i].constraint->points[pointRefs[i].pointIndex].normalImpulse;
        }

        bool converged = false;
        for (int pass = 0; pass < maxOuterFrictionPasses; ++pass)
        {
            // Reset bodies to snapshot
            for (auto& entry : initialStates)
            {
                entry.first->velocity = entry.second.velocity;
                entry.first->angularMomentum = entry.second.angularMomentum;
                entry.first->angularVelocity = entry.second.angularVelocity;
            }

            // Apply active friction impulses to bodies
            for (int i = 0; i < n; ++i)
            {
                const PointRef& p = pointRefs[i];
                ContactConstraint* cc = p.constraint;
                ConstraintPoint& cp = cc->points[p.pointIndex];
                Vec3 fImpulse = cp.tangentImpulse1 * cc->tangent1 + cp.tangentImpulse2 * cc->tangent2;

                if (cc->bodyA && cc->bodyA->invMass > 0.0)
                {
                    cc->bodyA->velocity -= cc->bodyA->invMass * fImpulse;
                    cc->bodyA->angularMomentum -= glm::cross(cp.rA, fImpulse);
                }
                if (cc->bodyB && cc->bodyB->invMass > 0.0)
                {
                    cc->bodyB->velocity += cc->bodyB->invMass * fImpulse;
                    cc->bodyB->angularMomentum += glm::cross(cp.rB, fImpulse);
                }
            }

            for (auto& entry : initialStates)
            {
                entry.first->angularVelocity = entry.first->invInertiaWorld * entry.first->angularMomentum;
            }

            // Assemble free vector q
            std::vector<Real> q(n, 0.0);
            for (int i = 0; i < n; ++i)
            {
                const PointRef& p = pointRefs[i];
                ContactConstraint* cc = p.constraint;
                ConstraintPoint& cp = cc->points[p.pointIndex];

                Vec3 vA = cc->bodyA ? (cc->bodyA->velocity + glm::cross(cc->bodyA->angularVelocity, cp.rA)) : Vec3(0.0);
                Vec3 vB = cc->bodyB ? (cc->bodyB->velocity + glm::cross(cc->bodyB->angularVelocity, cp.rB)) : Vec3(0.0);
                q[i] = glm::dot(vB - vA, cc->normal) + cp.speculativeBias;
            }

            // Solve normal LCP
            std::vector<Real> newZ(n, 0.0);
            LcpResult lcpRes = SolveLCPLemke(M, q, newZ, n);
            if (lcpRes.status != LcpStatus::Solved)
            {
                // Fallback to iterative solver if Lemke encounters numerical failure
                for (auto& entry : initialStates)
                {
                    entry.first->velocity = entry.second.velocity;
                    entry.first->angularMomentum = entry.second.angularMomentum;
                    entry.first->angularVelocity = entry.second.angularVelocity;
                }
                fallbackSolver.SolveVelocity(constraints, h);
                return;
            }

            Real maxDelta = 0.0;
            for (int i = 0; i < n; ++i)
            {
                maxDelta = std::max(maxDelta, std::abs(newZ[i] - z[i]));
                z[i] = newZ[i];
                pointRefs[i].constraint->points[pointRefs[i].pointIndex].normalImpulse = z[i];
            }

            // Apply new normal impulses to snapshot bodies
            for (auto& entry : initialStates)
            {
                entry.first->velocity = entry.second.velocity;
                entry.first->angularMomentum = entry.second.angularMomentum;
            }

            for (int i = 0; i < n; ++i)
            {
                const PointRef& p = pointRefs[i];
                ContactConstraint* cc = p.constraint;
                ConstraintPoint& cp = cc->points[p.pointIndex];
                Vec3 nImpulse = z[i] * cc->normal;

                if (cc->bodyA && cc->bodyA->invMass > 0.0)
                {
                    cc->bodyA->velocity -= cc->bodyA->invMass * nImpulse;
                    cc->bodyA->angularMomentum -= glm::cross(cp.rA, nImpulse);
                }
                if (cc->bodyB && cc->bodyB->invMass > 0.0)
                {
                    cc->bodyB->velocity += cc->bodyB->invMass * nImpulse;
                    cc->bodyB->angularMomentum += glm::cross(cp.rB, nImpulse);
                }
            }

            for (auto& entry : initialStates)
            {
                entry.first->angularVelocity = entry.first->invInertiaWorld * entry.first->angularMomentum;
            }

            // Solve friction disc constraints against the fixed normal impulses z[i]
            for (int i = 0; i < n; ++i)
            {
                const PointRef& p = pointRefs[i];
                ContactConstraint* cc = p.constraint;
                ConstraintPoint& cp = cc->points[p.pointIndex];

                Vec3 vA = cc->bodyA ? (cc->bodyA->velocity + glm::cross(cc->bodyA->angularVelocity, cp.rA)) : Vec3(0.0);
                Vec3 vB = cc->bodyB ? (cc->bodyB->velocity + glm::cross(cc->bodyB->angularVelocity, cp.rB)) : Vec3(0.0);
                Vec3 relVel = vB - vA;

                Real vt1 = glm::dot(relVel, cc->tangent1);
                Real vt2 = glm::dot(relVel, cc->tangent2);

                Real maxFriction = cp.kineticFriction * z[i];
                Real mag = std::sqrt(vt1 * vt1 + vt2 * vt2);
                if (mag > 1e-12)
                {
                    cp.tangentImpulse1 = -std::min(maxFriction, mag) * (vt1 / mag);
                    cp.tangentImpulse2 = -std::min(maxFriction, mag) * (vt2 / mag);
                }
                else
                {
                    cp.tangentImpulse1 = 0.0;
                    cp.tangentImpulse2 = 0.0;
                }
            }

            if (maxDelta < frictionTolerance)
            {
                converged = true;
                break;
            }
        }

        // Apply final converged normal and friction impulses from initial snapshot
        for (auto& entry : initialStates)
        {
            entry.first->velocity = entry.second.velocity;
            entry.first->angularMomentum = entry.second.angularMomentum;
        }

        for (int i = 0; i < n; ++i)
        {
            const PointRef& p = pointRefs[i];
            ContactConstraint* cc = p.constraint;
            ConstraintPoint& cp = cc->points[p.pointIndex];
            Vec3 totalImpulse = z[i] * cc->normal + cp.tangentImpulse1 * cc->tangent1 + cp.tangentImpulse2 * cc->tangent2;

            if (cc->bodyA && cc->bodyA->invMass > 0.0)
            {
                cc->bodyA->velocity -= cc->bodyA->invMass * totalImpulse;
                cc->bodyA->angularMomentum -= glm::cross(cp.rA, totalImpulse);
            }
            if (cc->bodyB && cc->bodyB->invMass > 0.0)
            {
                cc->bodyB->velocity += cc->bodyB->invMass * totalImpulse;
                cc->bodyB->angularMomentum += glm::cross(cp.rB, totalImpulse);
            }
        }

        for (auto& entry : initialStates)
        {
            entry.first->angularVelocity = entry.first->invInertiaWorld * entry.first->angularMomentum;
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

    void ExactSolver::SolveRestitution(std::vector<ContactConstraint>& constraints, double h)
    {
        fallbackSolver.SolveRestitution(constraints, h);
    }

    void ExactSolver::SolvePenetration(std::vector<ContactConstraint>& constraints, double h)
    {
        fallbackSolver.SolvePenetration(constraints, h);
    }
}