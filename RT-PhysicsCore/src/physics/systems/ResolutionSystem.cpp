#include "RT-PhysicsCore/physics/systems/ResolutionSystem.h"
#include "RT-PhysicsCore/physics/systems/CollisionSystem.h"
#include "RT-PhysicsCore/physics/components/RigidBodyComponent.h"
#include "RT-PhysicsCore/physics/LCPSolver.h"
#include "RT-PhysicsCore/core/ecs/core/Scene.h"
#include "RT-PhysicsCore/core/ecs/components/TransformComponent.h"
#include "RT-PhysicsCore/utils/Log.h"

#include <glm/gtc/quaternion.hpp>
#include <algorithm>
#include <array>
#include <chrono>

namespace RT_PhysicsCore
{
    namespace
    {
        constexpr float kEpsilon = 1e-8f;
        constexpr float kSlop = 0.005f;         // allowed penetration before correcting - avoids jitter
        constexpr float kBeta = 0.2f;           // fraction of remaining penetration corrected per position iteration
        constexpr float kMaxCorrection = 0.2f;  // cap on one iteration's positional push - avoids a deep-penetration "pop"

        // How much a body resists an impulse along n applied at r, given
        // its inverse mass/inertia - the rotational half of the effective
        // mass K. Larger = stiffer (a given impulse changes v_rel_n less).
        float AngularEffectiveMassTerm(const glm::vec3& r, const glm::vec3& n, const glm::mat3& invInertiaWorld)
        {
            return glm::dot(n, glm::cross(invInertiaWorld * glm::cross(r, n), r));
        }

        // Change in velocity at (this same body's) offset rQuery, from an
        // impulse applied at offset rImpulse on that body. Building block
        // for the exact solver's cross-contact coupling matrix.
        glm::vec3 ImpulseEffect(float invMass, const glm::mat3& invInertiaWorld,
            const glm::vec3& rImpulse, const glm::vec3& rQuery, const glm::vec3& impulse)
        {
            glm::vec3 dv = invMass * impulse;
            glm::vec3 dOmega = invInertiaWorld * glm::cross(rImpulse, impulse);
            return dv + glm::cross(dOmega, rQuery);
        }

        struct BodyRefs
        {
            RigidBodyComponent* bodyA;
            RigidBodyComponent* bodyB;
            TransformComponent* transformA;
            TransformComponent* transformB;
        };

        bool FetchBodies(Scene& scene, const Contact& contact, BodyRefs& out)
        {
            out.bodyA = scene.GetComponent<RigidBodyComponent>(contact.a);
            out.bodyB = scene.GetComponent<RigidBodyComponent>(contact.b);
            out.transformA = scene.GetComponent<TransformComponent>(contact.a);
            out.transformB = scene.GetComponent<TransformComponent>(contact.b);
            return out.bodyA && out.bodyB && out.transformA && out.transformB;
        }

        struct FlatPoint
        {
            RigidBodyComponent* bodyA;
            RigidBodyComponent* bodyB;
            glm::vec3 rA, rB, normal;
        };

        // M[i][j]: change in point i's relative normal velocity per unit
        // impulse applied at point j. Nonzero only where the two points
        // share a body. The i == j case reduces exactly to the single-
        // contact K used by the sequential-impulses path (both are the
        // same underlying physics, just assembled for one point at a time
        // there vs. the whole system at once here).
        float BuildMEntry(const FlatPoint& pi, const FlatPoint& pj)
        {
            glm::vec3 impulseOnB = pj.normal;
            glm::vec3 impulseOnA = -pj.normal;

            float total = 0.0f;
            if (pi.bodyB == pj.bodyB)
                total += glm::dot(ImpulseEffect(pi.bodyB->invMass, pi.bodyB->invInertiaWorld, pj.rB, pi.rB, impulseOnB), pi.normal);
            if (pi.bodyB == pj.bodyA)
                total += glm::dot(ImpulseEffect(pi.bodyB->invMass, pi.bodyB->invInertiaWorld, pj.rA, pi.rB, impulseOnA), pi.normal);
            if (pi.bodyA == pj.bodyB)
                total -= glm::dot(ImpulseEffect(pi.bodyA->invMass, pi.bodyA->invInertiaWorld, pj.rB, pi.rA, impulseOnB), pi.normal);
            if (pi.bodyA == pj.bodyA)
                total -= glm::dot(ImpulseEffect(pi.bodyA->invMass, pi.bodyA->invInertiaWorld, pj.rA, pi.rA, impulseOnA), pi.normal);
            return total;
        }
    }

    ResolutionSystem::ResolutionSystem(Scene& scene, CollisionSystem& collisionSystem, SolverMode initialMode)
        : ISystem(scene), collisionSystem(collisionSystem), solverMode(initialMode)
    {}

    void ResolutionSystem::SetSolverMode(SolverMode mode)
    {
        solverMode = mode;
    }

    ResolutionSystem::SolverMode ResolutionSystem::GetSolverMode() const
    {
        return solverMode;
    }

    void ResolutionSystem::SetIterationMode(IterationMode mode)
    {
        iterationMode = mode;
    }

    ResolutionSystem::IterationMode ResolutionSystem::GetIterationMode() const
    {
        return iterationMode;
    }

    void ResolutionSystem::SetVelocityIterations(int iterations)
    {
        velocityIterations = iterations;
    }

    void ResolutionSystem::SetPositionIterations(int iterations)
    {
        positionIterations = iterations;
    }

    void ResolutionSystem::SetVelocityIterationBounds(int minIterations, int maxIterations)
    {
        minVelocityIterations = minIterations;
        maxVelocityIterations = maxIterations;
    }

    void ResolutionSystem::SetPositionIterationBounds(int minIterations, int maxIterations)
    {
        minPositionIterations = minIterations;
        maxPositionIterations = maxIterations;
    }

    void ResolutionSystem::SetVelocityTimeBudgetMs(float milliseconds)
    {
        velocityTimeBudgetMs = milliseconds;
    }

    void ResolutionSystem::SetPositionTimeBudgetMs(float milliseconds)
    {
        positionTimeBudgetMs = milliseconds;
    }

    void ResolutionSystem::SetMaxLcpPivots(int pivots)
    {
        maxLcpPivots = pivots;
    }

    void ResolutionSystem::FixedUpdate(double /*dt*/)
    {
        const std::vector<Contact>& contacts = collisionSystem.GetContacts();
        if (contacts.empty())
            return;

        if (solverMode == SolverMode::Exact)
        {
            if (!ResolveExact(contacts))
            {
                RT_LOG_WARN("Exact contact solve did not converge this step (pivot cap or ray "
                    "termination) - falling back to sequential impulses for this step.");
                ResolveSequentialImpulses(contacts);
            }
        }
        else
        {
            ResolveSequentialImpulses(contacts);
        }

        CorrectPositions(contacts);
    }

    // ---------------- Sequential impulses ----------------
    void ResolutionSystem::ResolveSequentialImpulses(const std::vector<Contact>& contacts)
    {
        std::vector<std::array<float, kMaxContactPoints>> accumulated(contacts.size());
        for (auto& row : accumulated)
            row.fill(0.0f);

        int maxIter = (iterationMode == IterationMode::Adaptive) ? maxVelocityIterations : velocityIterations;
        auto startTime = std::chrono::steady_clock::now();

        for (int iter = 0; iter < maxIter; ++iter)
        {
            for (size_t c = 0; c < contacts.size(); ++c)
            {
                const Contact& contact = contacts[c];
                BodyRefs refs;
                if (!FetchBodies(scene, contact, refs))
                    continue;

                for (int p = 0; p < contact.pointCount; ++p)
                {
                    glm::vec3 rA = contact.points[p] - refs.transformA->position;
                    glm::vec3 rB = contact.points[p] - refs.transformB->position;

                    glm::vec3 velAtA = refs.bodyA->velocity + glm::cross(refs.bodyA->angularVelocity, rA);
                    glm::vec3 velAtB = refs.bodyB->velocity + glm::cross(refs.bodyB->angularVelocity, rB);
                    float relVelN = glm::dot(velAtB - velAtA, contact.normal);

                    float K = refs.bodyA->invMass + refs.bodyB->invMass
                        + AngularEffectiveMassTerm(rA, contact.normal, refs.bodyA->invInertiaWorld)
                        + AngularEffectiveMassTerm(rB, contact.normal, refs.bodyB->invInertiaWorld);
                    if (K < kEpsilon)
                        continue;

                    float lambda = -relVelN / K;

                    // Clamp the RUNNING TOTAL, not this iteration's raw
                    // delta - a contact can only push (never pull), but an
                    // earlier iteration's overshoot still needs to be
                    // undoable by a later one.
                    float& accum = accumulated[c][p];
                    float newAccum = std::max(0.0f, accum + lambda);
                    float delta = newAccum - accum;
                    accum = newAccum;

                    glm::vec3 impulse = delta * contact.normal;

                    refs.bodyA->velocity -= refs.bodyA->invMass * impulse;
                    refs.bodyB->velocity += refs.bodyB->invMass * impulse;
                    refs.bodyA->angularMomentum -= glm::cross(rA, impulse);
                    refs.bodyB->angularMomentum += glm::cross(rB, impulse);

                    // Re-derive angular velocity immediately - later points
                    // and iterations this same step read it.
                    refs.bodyA->angularVelocity = refs.bodyA->invInertiaWorld * refs.bodyA->angularMomentum;
                    refs.bodyB->angularVelocity = refs.bodyB->invInertiaWorld * refs.bodyB->angularMomentum;
                }
            }

            if (iterationMode == IterationMode::Adaptive && (iter + 1) >= minVelocityIterations)
            {
                double elapsedMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - startTime).count();
                if (elapsedMs >= velocityTimeBudgetMs)
                    break;
            }
        }
    }

    // ---------------- Exact (Lemke's algorithm) ----------------
    bool ResolutionSystem::ResolveExact(const std::vector<Contact>& contacts)
    {
        std::vector<FlatPoint> points;

        for (const Contact& contact : contacts)
        {
            BodyRefs refs;
            if (!FetchBodies(scene, contact, refs))
                continue;

            for (int p = 0; p < contact.pointCount; ++p)
            {
                glm::vec3 rA = contact.points[p] - refs.transformA->position;
                glm::vec3 rB = contact.points[p] - refs.transformB->position;

                float K = refs.bodyA->invMass + refs.bodyB->invMass
                    + AngularEffectiveMassTerm(rA, contact.normal, refs.bodyA->invInertiaWorld)
                    + AngularEffectiveMassTerm(rB, contact.normal, refs.bodyB->invInertiaWorld);
                if (K < kEpsilon)
                    continue; // both sides immovable at this point - nothing to solve

                points.push_back({ refs.bodyA, refs.bodyB, rA, rB, contact.normal });
            }
        }

        int n = static_cast<int>(points.size());
        if (n == 0)
            return true;

        std::vector<std::vector<float>> M(n, std::vector<float>(n));
        std::vector<float> q(n);

        for (int i = 0; i < n; ++i)
        {
            glm::vec3 velAtA = points[i].bodyA->velocity + glm::cross(points[i].bodyA->angularVelocity, points[i].rA);
            glm::vec3 velAtB = points[i].bodyB->velocity + glm::cross(points[i].bodyB->angularVelocity, points[i].rB);
            q[i] = glm::dot(velAtB - velAtA, points[i].normal);

            for (int j = 0; j < n; ++j)
                M[i][j] = BuildMEntry(points[i], points[j]);
        }

        std::vector<float> z;
        if (!SolveLCPLemke(M, q, z, maxLcpPivots))
            return false;

        // All impulses are meant to apply simultaneously - accumulate
        // every velocity/momentum change first, and only refresh angular
        // velocity afterward, so the order points happen to be listed in
        // doesn't leak into the result the way it deliberately does in
        // the sequential-impulses pass.
        for (int i = 0; i < n; ++i)
        {
            if (z[i] <= kEpsilon)
                continue;
            glm::vec3 impulse = z[i] * points[i].normal;
            points[i].bodyA->velocity -= points[i].bodyA->invMass * impulse;
            points[i].bodyB->velocity += points[i].bodyB->invMass * impulse;
            points[i].bodyA->angularMomentum -= glm::cross(points[i].rA, impulse);
            points[i].bodyB->angularMomentum += glm::cross(points[i].rB, impulse);
        }
        for (int i = 0; i < n; ++i)
        {
            points[i].bodyA->angularVelocity = points[i].bodyA->invInertiaWorld * points[i].bodyA->angularMomentum;
            points[i].bodyB->angularVelocity = points[i].bodyB->invInertiaWorld * points[i].bodyB->angularMomentum;
        }

        return true;
    }

    // ---------------- Position correction (shared by both solver modes) ----------------
    void ResolutionSystem::CorrectPositions(const std::vector<Contact>& contacts)
    {
        // Deliberately separate from the velocity solve and never touches
        // velocity/angularMomentum - a bias baked into the velocity solve
        // (Baumgarte) would inject real kinetic energy into the system;
        // this instead nudges position/orientation directly, which can't.
        // Two standard, small-error simplifications, not attempts at being
        // exact: reuses each body's cached invInertiaWorld rather than
        // rebuilding it from the very slightly shifting orientation every
        // iteration, and tracks remaining separation incrementally instead
        // of re-running narrow phase.
        std::vector<std::array<float, kMaxContactPoints>> separation(contacts.size());
        for (size_t c = 0; c < contacts.size(); ++c)
            for (int p = 0; p < contacts[c].pointCount; ++p)
                separation[c][p] = -contacts[c].penetrations[p];

        int maxIter = (iterationMode == IterationMode::Adaptive) ? maxPositionIterations : positionIterations;
        auto startTime = std::chrono::steady_clock::now();

        for (int iter = 0; iter < maxIter; ++iter)
        {
            for (size_t c = 0; c < contacts.size(); ++c)
            {
                const Contact& contact = contacts[c];
                BodyRefs refs;
                if (!FetchBodies(scene, contact, refs))
                    continue;

                for (int p = 0; p < contact.pointCount; ++p)
                {
                    float& sep = separation[c][p];
                    float correction = std::min(std::max(kBeta * (-sep - kSlop), 0.0f), kMaxCorrection);
                    if (correction <= 0.0f)
                        continue;

                    glm::vec3 rA = contact.points[p] - refs.transformA->position;
                    glm::vec3 rB = contact.points[p] - refs.transformB->position;

                    float K = refs.bodyA->invMass + refs.bodyB->invMass
                        + AngularEffectiveMassTerm(rA, contact.normal, refs.bodyA->invInertiaWorld)
                        + AngularEffectiveMassTerm(rB, contact.normal, refs.bodyB->invInertiaWorld);
                    if (K < kEpsilon)
                        continue;

                    glm::vec3 push = (correction / K) * contact.normal;

                    refs.transformA->position -= refs.bodyA->invMass * push;
                    refs.transformB->position += refs.bodyB->invMass * push;

                    glm::vec3 rotA = -(refs.bodyA->invInertiaWorld * glm::cross(rA, push));
                    glm::vec3 rotB = refs.bodyB->invInertiaWorld * glm::cross(rB, push);

                    glm::quat dqA(0.0f, rotA.x, rotA.y, rotA.z);
                    glm::quat dqB(0.0f, rotB.x, rotB.y, rotB.z);
                    refs.transformA->rotation = glm::normalize(refs.transformA->rotation + 0.5f * (dqA * refs.transformA->rotation));
                    refs.transformB->rotation = glm::normalize(refs.transformB->rotation + 0.5f * (dqB * refs.transformB->rotation));

                    sep += correction;
                }
            }

            if (iterationMode == IterationMode::Adaptive && (iter + 1) >= minPositionIterations)
            {
                double elapsedMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - startTime).count();
                if (elapsedMs >= positionTimeBudgetMs)
                    break;
            }
        }
    }
}
