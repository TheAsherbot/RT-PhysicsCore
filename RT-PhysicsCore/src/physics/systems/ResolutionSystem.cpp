#include "RT-PhysicsCore/physics/systems/ResolutionSystem.h"
#include "RT-PhysicsCore/physics/systems/CollisionSystem.h"
#include "RT-PhysicsCore/core/ecs/core/Scene.h"
#include "RT-PhysicsCore/core/ecs/components/TransformComponent.h"
#include "RT-PhysicsCore/physics/components/RigidBodyComponent.h"

#include <glm/gtc/quaternion.hpp>
#include <algorithm>
#include <array>

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
    }

    ResolutionSystem::ResolutionSystem(Scene& scene, CollisionSystem& collisionSystem)
        : ISystem(scene), collisionSystem(collisionSystem)
    {}

    void ResolutionSystem::FixedUpdate(double /*dt*/)
    {
        const std::vector<Contact>& contacts = collisionSystem.GetContacts();
        if (contacts.empty())
            return;

        // ---------------- Velocity pass: resolve non-penetration ----------------
        // Per-point running impulse total for THIS step's iterations only -
        // no cross-frame warm start yet. That would need a stable identity
        // for "this frame's contact point" vs. "last frame's", which the
        // narrow phase doesn't produce today (raw clipped positions, no
        // feature IDs) - a real follow-up, not implemented here.
        std::vector<std::array<float, kMaxContactPoints>> accumulated(contacts.size());
        for (auto& row : accumulated)
            row.fill(0.0f);

        for (int iter = 0; iter < velocityIterations; ++iter)
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
                    // undoable by a later one. See the design notes for a
                    // worked example of why this matters.
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
        }

        // ---------------- Position pass: correct leftover penetration ----------------
        // Deliberately separate from the velocity pass and never touches
        // velocity/angularMomentum - a bias baked into the velocity solve
        // (Baumgarte) would inject real kinetic energy into the system;
        // this instead nudges position/orientation directly, which can't.
        // Two standard, small-error simplifications here, not attempts at
        // being exact: (1) reuses each body's cached invInertiaWorld
        // rather than rebuilding it from the very slightly shifting
        // orientation every iteration, and (2) tracks remaining separation
        // incrementally instead of re-running narrow phase each iteration.
        std::vector<std::array<float, kMaxContactPoints>> separation(contacts.size());
        for (size_t c = 0; c < contacts.size(); ++c)
            for (int p = 0; p < contacts[c].pointCount; ++p)
                separation[c][p] = -contacts[c].penetrations[p];

        for (int iter = 0; iter < positionIterations; ++iter)
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
        }
    }
}
