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
#include <cmath>

namespace RT_PhysicsCore
{
    namespace
    {
        constexpr float kEpsilon = 1e-8f;
        constexpr float kSlop = 0.005f;         // allowed penetration before correcting - avoids jitter
        constexpr float kBeta = 0.2f;           // fraction of remaining penetration corrected per position iteration
        constexpr float kMaxCorrection = 0.2f;  // cap on one iteration's positional push - avoids a deep-penetration "pop"

        // How much a body resists an impulse along axis, applied at r,
        // given its inverse mass/inertia - the rotational half of the
        // effective mass K along that axis. Larger = stiffer.
        float AngularEffectiveMassTerm(const glm::vec3& r, const glm::vec3& axis, const glm::mat3& invInertiaWorld)
        {
            return glm::dot(axis, glm::cross(invInertiaWorld * glm::cross(r, axis), r));
        }

        // General form: effect on relative velocity along dQuery from a
        // unit impulse along dImpulse (both through this body's rotation
        // at offset r). AngularEffectiveMassTerm above is the special
        // case dQuery == dImpulse; this is the off-diagonal term the
        // joint t1/t2 friction solve needs - symmetric in (dQuery,
        // dImpulse) since invInertiaWorld is symmetric.
        float CrossEffectiveMassTerm(const glm::vec3& r, const glm::vec3& dQuery, const glm::vec3& dImpulse, const glm::mat3& invInertiaWorld)
        {
            return glm::dot(dQuery, glm::cross(invInertiaWorld * glm::cross(r, dImpulse), r));
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

        // Per-point solver state: set up once per step from the state at
        // the start of the step (relative velocity, which regime -
        // stuck/sliding - friction starts in), then carried across
        // iterations as the running accumulated impulses.
        struct PointState
        {
            glm::vec3 rA, rB;
            glm::vec3 normal;
            glm::vec3 t1, t2;
            float restitutionBias = 0.0f;
            float frictionCoeff = 0.0f;
            float accumNormal = 0.0f;
            float accumT1 = 0.0f;
            float accumT2 = 0.0f;
        };

        // Fixed, normal-derived tangent basis (not velocity-aligned) -
        // well-defined even when nothing is sliding, which matters since
        // static friction needs a basis precisely in that case.
        void BuildTangentBasis(const glm::vec3& n, glm::vec3& t1, glm::vec3& t2)
        {
            glm::vec3 arbitrary = (std::abs(n.x) < 0.9f) ? glm::vec3(1.0f, 0.0f, 0.0f) : glm::vec3(0.0f, 1.0f, 0.0f);
            t1 = glm::normalize(glm::cross(arbitrary, n));
            t2 = glm::cross(n, t1);
        }

        PointState SetupPointState(const glm::vec3& normal, float restitution, float staticFriction, float kineticFriction,
            const glm::vec3& rA, const glm::vec3& rB,
            RigidBodyComponent* bodyA, RigidBodyComponent* bodyB,
            float restitutionVelThreshold, float frictionVelThreshold)
        {
            PointState state;
            state.rA = rA;
            state.rB = rB;
            state.normal = normal;
            BuildTangentBasis(normal, state.t1, state.t2);

            glm::vec3 velAtA = bodyA->velocity + glm::cross(bodyA->angularVelocity, rA);
            glm::vec3 velAtB = bodyB->velocity + glm::cross(bodyB->angularVelocity, rB);
            glm::vec3 relVel = velAtB - velAtA;
            float relVelN = glm::dot(relVel, normal);

            // Below the threshold, this reads as a resting contact (a
            // body sitting under gravity has a tiny closing velocity every
            // step from that step's integration alone) rather than a real
            // impact - forcing e=0 there is what keeps a resting body from
            // micro-bouncing forever instead of actually settling.
            state.restitutionBias = (relVelN < -restitutionVelThreshold) ? (restitution * relVelN) : 0.0f;

            glm::vec3 relVelT = relVel - relVelN * normal;
            state.frictionCoeff = (glm::length(relVelT) < frictionVelThreshold) ? staticFriction : kineticFriction;

            return state;
        }

        void SolveNormalAtPoint(RigidBodyComponent* bodyA, RigidBodyComponent* bodyB, PointState& state)
        {
            glm::vec3 velAtA = bodyA->velocity + glm::cross(bodyA->angularVelocity, state.rA);
            glm::vec3 velAtB = bodyB->velocity + glm::cross(bodyB->angularVelocity, state.rB);
            float relVelN = glm::dot(velAtB - velAtA, state.normal);

            float K = bodyA->invMass + bodyB->invMass
                + AngularEffectiveMassTerm(state.rA, state.normal, bodyA->invInertiaWorld)
                + AngularEffectiveMassTerm(state.rB, state.normal, bodyB->invInertiaWorld);
            if (K < kEpsilon)
                return;

            float lambda = -(relVelN + state.restitutionBias) / K;

            float newAccum = std::max(0.0f, state.accumNormal + lambda);
            float delta = newAccum - state.accumNormal;
            state.accumNormal = newAccum;

            glm::vec3 impulse = delta * state.normal;
            bodyA->velocity -= bodyA->invMass * impulse;
            bodyB->velocity += bodyB->invMass * impulse;
            bodyA->angularMomentum -= glm::cross(state.rA, impulse);
            bodyB->angularMomentum += glm::cross(state.rB, impulse);
            bodyA->angularVelocity = bodyA->invInertiaWorld * bodyA->angularMomentum;
            bodyB->angularVelocity = bodyB->invInertiaWorld * bodyB->angularMomentum;
        }

        // Solves for BOTH tangential impulses jointly (the 2x2 system
        // below accounts for inertial coupling between t1 and t2, not two
        // independent 1D solves), then clamps the combined 2D result to a
        // disc of radius state.frictionCoeff * state.accumNormal -
        // state.accumNormal being whatever it currently is (live-updating
        // alongside SolveNormalAtPoint in the sequential-impulses pass, or
        // fixed from the exact solve in its friction-only follow-up pass).
        // This is the actual Coulomb cone, not the square-pyramid
        // approximation (independent per-axis clamps).
        void SolveFrictionAtPoint(RigidBodyComponent* bodyA, RigidBodyComponent* bodyB, PointState& state)
        {
            glm::vec3 velAtA = bodyA->velocity + glm::cross(bodyA->angularVelocity, state.rA);
            glm::vec3 velAtB = bodyB->velocity + glm::cross(bodyB->angularVelocity, state.rB);
            glm::vec3 relVel = velAtB - velAtA;

            float vt1 = glm::dot(relVel, state.t1);
            float vt2 = glm::dot(relVel, state.t2);

            float Kt1t1 = bodyA->invMass + bodyB->invMass
                + AngularEffectiveMassTerm(state.rA, state.t1, bodyA->invInertiaWorld)
                + AngularEffectiveMassTerm(state.rB, state.t1, bodyB->invInertiaWorld);
            float Kt2t2 = bodyA->invMass + bodyB->invMass
                + AngularEffectiveMassTerm(state.rA, state.t2, bodyA->invInertiaWorld)
                + AngularEffectiveMassTerm(state.rB, state.t2, bodyB->invInertiaWorld);
            float Kt1t2 = CrossEffectiveMassTerm(state.rA, state.t1, state.t2, bodyA->invInertiaWorld)
                + CrossEffectiveMassTerm(state.rB, state.t1, state.t2, bodyB->invInertiaWorld);

            // Joint 2x2 solve (Cramer's rule) for the tangential impulse
            // that zeroes vt1 AND vt2 together, rather than treating the
            // two tangent axes as independent - the inertia tensor can
            // genuinely couple them (an impulse along t1 inducing a
            // velocity change with a component along t2), and capturing
            // that coupling faithfully is the whole point of doing the
            // cone properly instead of the cheaper pyramid approximation.
            float det = Kt1t1 * Kt2t2 - Kt1t2 * Kt1t2;

            float deltaLambdaT1, deltaLambdaT2;
            if (std::abs(det) > kEpsilon)
            {
                deltaLambdaT1 = (-vt1 * Kt2t2 + Kt1t2 * vt2) / det;
                deltaLambdaT2 = (-Kt1t1 * vt2 + vt1 * Kt1t2) / det;
            }
            else
            {
                // Near-singular 2x2 system - fall back to the two axes
                // independently rather than dividing by ~0.
                deltaLambdaT1 = (Kt1t1 > kEpsilon) ? (-vt1 / Kt1t1) : 0.0f;
                deltaLambdaT2 = (Kt2t2 > kEpsilon) ? (-vt2 / Kt2t2) : 0.0f;
            }

            float newAccumT1 = state.accumT1 + deltaLambdaT1;
            float newAccumT2 = state.accumT2 + deltaLambdaT2;

            float maxFriction = state.frictionCoeff * state.accumNormal;
            float mag = std::sqrt(newAccumT1 * newAccumT1 + newAccumT2 * newAccumT2);
            if (mag > maxFriction)
            {
                float scale = (mag > kEpsilon) ? (maxFriction / mag) : 0.0f;
                newAccumT1 *= scale;
                newAccumT2 *= scale;
            }

            float deltaT1 = newAccumT1 - state.accumT1;
            float deltaT2 = newAccumT2 - state.accumT2;
            state.accumT1 = newAccumT1;
            state.accumT2 = newAccumT2;

            glm::vec3 impulse = deltaT1 * state.t1 + deltaT2 * state.t2;
            bodyA->velocity -= bodyA->invMass * impulse;
            bodyB->velocity += bodyB->invMass * impulse;
            bodyA->angularMomentum -= glm::cross(state.rA, impulse);
            bodyB->angularMomentum += glm::cross(state.rB, impulse);
            bodyA->angularVelocity = bodyA->invInertiaWorld * bodyA->angularMomentum;
            bodyB->angularVelocity = bodyB->invInertiaWorld * bodyB->angularMomentum;
        }

        struct ExactPoint
        {
            RigidBodyComponent* bodyA;
            RigidBodyComponent* bodyB;
            glm::vec3 rA, rB, normal;
            float restitution, staticFriction, kineticFriction;
        };

        // M[i][j]: change in point i's relative normal velocity per unit
        // impulse applied at point j. Nonzero only where the two points
        // share a body. The i == j case reduces exactly to the single-
        // contact K used by the sequential-impulses path.
        float BuildMEntry(const ExactPoint& pi, const ExactPoint& pj)
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

    void ResolutionSystem::SetSolverMode(SolverMode mode) { solverMode = mode; }
    ResolutionSystem::SolverMode ResolutionSystem::GetSolverMode() const { return solverMode; }

    void ResolutionSystem::SetIterationMode(IterationMode mode) { iterationMode = mode; }
    ResolutionSystem::IterationMode ResolutionSystem::GetIterationMode() const { return iterationMode; }

    void ResolutionSystem::SetVelocityIterations(int iterations) { velocityIterations = iterations; }
    void ResolutionSystem::SetPositionIterations(int iterations) { positionIterations = iterations; }

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
    void ResolutionSystem::SetVelocityTimeBudgetMs(float milliseconds) { velocityTimeBudgetMs = milliseconds; }
    void ResolutionSystem::SetPositionTimeBudgetMs(float milliseconds) { positionTimeBudgetMs = milliseconds; }

    void ResolutionSystem::SetMaxLcpPivots(int pivots) { maxLcpPivots = pivots; }

    void ResolutionSystem::SetRestitutionVelocityThreshold(float threshold) { restitutionVelocityThreshold = threshold; }
    void ResolutionSystem::SetFrictionVelocityThreshold(float threshold) { frictionVelocityThreshold = threshold; }

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

    // ---------------- Sequential impulses (normal + friction together) ----------------
    void ResolutionSystem::ResolveSequentialImpulses(const std::vector<Contact>& contacts)
    {
        std::vector<BodyRefs> refs(contacts.size());
        std::vector<bool> valid(contacts.size(), false);
        std::vector<std::array<PointState, kMaxContactPoints>> states(contacts.size());

        for (size_t c = 0; c < contacts.size(); ++c)
        {
            const Contact& contact = contacts[c];
            valid[c] = FetchBodies(scene, contact, refs[c]);
            if (!valid[c])
                continue;

            for (int p = 0; p < contact.pointCount; ++p)
            {
                glm::vec3 rA = contact.points[p] - refs[c].transformA->position;
                glm::vec3 rB = contact.points[p] - refs[c].transformB->position;
                states[c][p] = SetupPointState(contact.normal, contact.restitution, contact.staticFriction, contact.kineticFriction,
                    rA, rB, refs[c].bodyA, refs[c].bodyB,
                    restitutionVelocityThreshold, frictionVelocityThreshold);
            }
        }

        int maxIter = (iterationMode == IterationMode::Adaptive) ? maxVelocityIterations : velocityIterations;
        auto startTime = std::chrono::steady_clock::now();

        for (int iter = 0; iter < maxIter; ++iter)
        {
            for (size_t c = 0; c < contacts.size(); ++c)
            {
                if (!valid[c])
                    continue;
                for (int p = 0; p < contacts[c].pointCount; ++p)
                {
                    SolveNormalAtPoint(refs[c].bodyA, refs[c].bodyB, states[c][p]);
                    SolveFrictionAtPoint(refs[c].bodyA, refs[c].bodyB, states[c][p]);
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

    // ---------------- Exact normal impulses (Lemke's algorithm) + iterative friction ----------------
    bool ResolutionSystem::ResolveExact(const std::vector<Contact>& contacts)
    {
        std::vector<ExactPoint> points;

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

                points.push_back({ refs.bodyA, refs.bodyB, rA, rB, contact.normal,
                                    contact.restitution, contact.staticFriction, contact.kineticFriction });
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
            float relVelN = glm::dot(velAtB - velAtA, points[i].normal);

            // Restitution folded into q, same resting-contact guard as the
            // sequential path: e = (1+e_material)*relVelN only counts as a
            // real impact above the threshold, else treated as e=0.
            float e = (relVelN < -restitutionVelocityThreshold) ? points[i].restitution : 0.0f;
            q[i] = (1.0f + e) * relVelN;

            for (int j = 0; j < n; ++j)
                M[i][j] = BuildMEntry(points[i], points[j]);
        }

        std::vector<float> z;
        if (!SolveLCPLemke(M, q, z, maxLcpPivots))
            return false;

        // All normal impulses are meant to apply simultaneously - apply
        // every velocity/momentum change first, and only refresh angular
        // velocity afterward, so point order doesn't leak into the result.
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

        // Friction as a separate, iterative pass afterward: normal
        // impulses are already exact and fixed (z[i]) - only the
        // tangential impulses iterate, bounded by those now-known normal
        // magnitudes. Jointly solving normal-and-friction "exactly" is a
        // genuinely harder (nonlinear) problem than Lemke's algorithm
        // covers - this keeps exact mode's real benefit (fully-converged
        // normal impulses) without pretending to solve something it can't.
        std::vector<PointState> frictionStates(n);
        for (int i = 0; i < n; ++i)
        {
            frictionStates[i] = SetupPointState(points[i].normal, points[i].restitution,
                points[i].staticFriction, points[i].kineticFriction,
                points[i].rA, points[i].rB, points[i].bodyA, points[i].bodyB,
                restitutionVelocityThreshold, frictionVelocityThreshold);
            frictionStates[i].accumNormal = std::max(0.0f, z[i]); // fixed - never updated again this pass
        }

        int maxIter = (iterationMode == IterationMode::Adaptive) ? maxVelocityIterations : velocityIterations;
        for (int iter = 0; iter < maxIter; ++iter)
            for (int i = 0; i < n; ++i)
                SolveFrictionAtPoint(points[i].bodyA, points[i].bodyB, frictionStates[i]);

        return true;
    }

    // ---------------- Position correction (shared by both solver modes) ----------------
    void ResolutionSystem::CorrectPositions(const std::vector<Contact>& contacts)
    {
        // Deliberately separate from the velocity solve and never touches
        // velocity/angularMomentum - a bias baked into the velocity solve
        // (Baumgarte) would inject real kinetic energy into the system;
        // this instead nudges position/orientation directly, which can't.
        std::vector<BodyRefs> refs(contacts.size());
        std::vector<bool> valid(contacts.size(), false);
        std::vector<std::array<float, kMaxContactPoints>> separation(contacts.size());

        for (size_t c = 0; c < contacts.size(); ++c)
        {
            valid[c] = FetchBodies(scene, contacts[c], refs[c]);
            for (int p = 0; p < contacts[c].pointCount; ++p)
                separation[c][p] = -contacts[c].penetrations[p];
        }

        int maxIter = (iterationMode == IterationMode::Adaptive) ? maxPositionIterations : positionIterations;
        auto startTime = std::chrono::steady_clock::now();

        for (int iter = 0; iter < maxIter; ++iter)
        {
            for (size_t c = 0; c < contacts.size(); ++c)
            {
                if (!valid[c])
                    continue;
                const Contact& contact = contacts[c];

                for (int p = 0; p < contact.pointCount; ++p)
                {
                    float& sep = separation[c][p];
                    float correction = std::min(std::max(kBeta * (-sep - kSlop), 0.0f), kMaxCorrection);
                    if (correction <= 0.0f)
                        continue;

                    glm::vec3 rA = contact.points[p] - refs[c].transformA->position;
                    glm::vec3 rB = contact.points[p] - refs[c].transformB->position;

                    float K = refs[c].bodyA->invMass + refs[c].bodyB->invMass
                        + AngularEffectiveMassTerm(rA, contact.normal, refs[c].bodyA->invInertiaWorld)
                        + AngularEffectiveMassTerm(rB, contact.normal, refs[c].bodyB->invInertiaWorld);
                    if (K < kEpsilon)
                        continue;

                    glm::vec3 push = (correction / K) * contact.normal;

                    refs[c].transformA->position -= refs[c].bodyA->invMass * push;
                    refs[c].transformB->position += refs[c].bodyB->invMass * push;

                    glm::vec3 rotA = -(refs[c].bodyA->invInertiaWorld * glm::cross(rA, push));
                    glm::vec3 rotB = refs[c].bodyB->invInertiaWorld * glm::cross(rB, push);

                    glm::quat dqA(0.0f, rotA.x, rotA.y, rotA.z);
                    glm::quat dqB(0.0f, rotB.x, rotB.y, rotB.z);
                    refs[c].transformA->rotation = glm::normalize(refs[c].transformA->rotation + 0.5f * (dqA * refs[c].transformA->rotation));
                    refs[c].transformB->rotation = glm::normalize(refs[c].transformB->rotation + 0.5f * (dqB * refs[c].transformB->rotation));

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