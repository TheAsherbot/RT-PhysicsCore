/**
 * @file NarrowPhase.cpp
 * @brief Implementation of exact geometric narrow-phase collision tests with signed separation and SAT face bias.
 */

#include "RT-PhysicsCore/physics/collision/NarrowPhase.h"
#include "RT-PhysicsCore/physics/collision/ManifoldReduction.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace RT_PhysicsCore
{
    namespace
    {
        constexpr Real kEpsilon = 1e-12;
        constexpr Real kFacePreferenceBias = 0.002; // Edge axes must beat face axes by 2mm to avoid chattering

        Vec3 ClosestPointOnSegment(const Vec3& p, const Vec3& a, const Vec3& b)
        {
            Vec3 ab = b - a;
            Real abLenSq = glm::dot(ab, ab);
            if (abLenSq < kEpsilon)
            {
                return a;
            }

            Real t = glm::clamp(glm::dot(p - a, ab) / abLenSq, 0.0, 1.0);
            return a + t * ab;
        }

        // Closest points between two 3D segments (Ericson, "Real-Time
        // Collision Detection" 5.1.9), handling degenerate/parallel cases.
        void ClosestPointsSegmentSegment(const Vec3& a0, const Vec3& a1,
            const Vec3& b0, const Vec3& b1, Vec3& outA, Vec3& outB)
        {
            Vec3 d1 = a1 - a0;
            Vec3 d2 = b1 - b0;
            Vec3 r = a0 - b0;

            Real a = glm::dot(d1, d1);
            Real e = glm::dot(d2, d2);
            Real f = glm::dot(d2, r);

            Real s = 0.0;
            Real t = 0.0;

            if (a <= kEpsilon && e <= kEpsilon)
            {
                outA = a0;
                outB = b0;
                return;
            }

            if (a <= kEpsilon)
            {
                s = 0.0;
                t = glm::clamp(f / e, 0.0, 1.0);
            }
            else
            {
                Real c = glm::dot(d1, r);
                if (e <= kEpsilon)
                {
                    t = 0.0;
                    s = glm::clamp(-c / a, 0.0, 1.0);
                }
                else
                {
                    Real b = glm::dot(d1, d2);
                    Real denom = a * e - b * b;

                    s = (denom > kEpsilon) ? glm::clamp((b * f - c * e) / denom, 0.0, 1.0) : 0.0;
                    t = (b * s + f) / e;

                    if (t < 0.0)
                    {
                        t = 0.0;
                        s = glm::clamp(-c / a, 0.0, 1.0);
                    }
                    else if (t > 1.0)
                    {
                        t = 1.0;
                        s = glm::clamp((b - c) / a, 0.0, 1.0);
                    }
                }
            }

            outA = a0 + d1 * s;
            outB = b0 + d2 * t;
        }

        bool SphereSphere(const Vec3& posA, Real radiusA, const Vec3& posB, Real radiusB, Contact& out, Real margin)
        {
            Vec3 delta = posB - posA;
            Real distSq = glm::dot(delta, delta);
            Real radiusSum = radiusA + radiusB;
            Real dist = std::sqrt(distSq);
            Real signedDistance = dist - radiusSum;

            if (signedDistance > margin)
            {
                return false;
            }

            Vec3 normal = (dist > kEpsilon) ? (delta / dist) : Vec3(0.0, 1.0, 0.0);

            out.normal = normal;
            out.points[0].position = 0.5 * ((posA + normal * radiusA) + (posB - normal * radiusB));
            out.points[0].signedDistance = signedDistance;
            out.points[0].localAnchorA = out.points[0].position - posA;
            out.points[0].localAnchorB = out.points[0].position - posB;
            out.pointCount = 1;
            return true;
        }

        bool SphereBox(const Vec3& sphereCenter, Real sphereRadius,
            const Vec3& boxPos, const Mat3& boxRot, const Vec3& boxHalf,
            Contact& out, Real margin)
        {
            Vec3 d = sphereCenter - boxPos;
            Vec3 local(glm::dot(d, boxRot[0]), glm::dot(d, boxRot[1]), glm::dot(d, boxRot[2]));
            Vec3 clampedLocal = glm::clamp(local, -boxHalf, boxHalf);

            Vec3 normalLocal;
            Vec3 closestLocal;
            Real signedDistance;

            if (clampedLocal == local)
            {
                // Center is inside the box - push out toward the nearest face.
                Vec3 faceDist = boxHalf - glm::abs(local);
                int axis = 0;
                if (faceDist.y < faceDist.x) axis = 1;
                if (faceDist.z < faceDist[axis]) axis = 2;

                Real sign = (local[axis] >= 0.0) ? 1.0 : -1.0;
                normalLocal = Vec3(0.0);
                normalLocal[axis] = sign;
                closestLocal = local;
                closestLocal[axis] = sign * boxHalf[axis];
                signedDistance = -(faceDist[axis] + sphereRadius);
            }
            else
            {
                Vec3 diff = local - clampedLocal; // box surface point -> sphere
                Real dist = glm::length(diff);
                signedDistance = dist - sphereRadius;

                if (signedDistance > margin)
                {
                    return false;
                }

                normalLocal = (dist > kEpsilon) ? -(diff / dist) : Vec3(1.0, 0.0, 0.0);
                closestLocal = clampedLocal;
            }

            out.normal = boxRot[0] * normalLocal.x + boxRot[1] * normalLocal.y + boxRot[2] * normalLocal.z;
            out.points[0].position = boxPos + boxRot[0] * closestLocal.x + boxRot[1] * closestLocal.y + boxRot[2] * closestLocal.z;
            out.points[0].signedDistance = signedDistance;
            out.points[0].localAnchorA = out.points[0].position - sphereCenter;
            out.points[0].localAnchorB = glm::transpose(boxRot) * (out.points[0].position - boxPos);
            out.pointCount = 1;
            return true;
        }

        bool SphereCapsule(const Vec3& sphereCenter, Real sphereRadius,
            const Vec3& capA, const Vec3& capB, Real capRadius,
            Contact& out, Real margin)
        {
            Vec3 closest = ClosestPointOnSegment(sphereCenter, capA, capB);
            return SphereSphere(sphereCenter, sphereRadius, closest, capRadius, out, margin);
        }

        bool CapsuleCapsule(const Vec3& aP0, const Vec3& aP1, Real radiusA,
            const Vec3& bP0, const Vec3& bP1, Real radiusB,
            Contact& out, Real margin)
        {
            Vec3 dA = aP1 - aP0;
            Vec3 dB = bP1 - bP0;
            Real lenA = glm::length(dA);
            Real lenB = glm::length(dB);

            // Near-parallel segments: a single closest-point pair leaves
            // one contact for what's actually a resting-along-each-other
            // overlap, which lets the pair roll. Generate two points
            // spanning the overlapping interval instead.
            if (lenA > kEpsilon && lenB > kEpsilon)
            {
                Vec3 dirA = dA / lenA;
                Vec3 dirB = dB / lenB;

                if (std::abs(glm::dot(dirA, dirB)) > 0.999)
                {
                    Real tB0 = glm::dot(bP0 - aP0, dirA);
                    Real tB1 = glm::dot(bP1 - aP0, dirA);
                    Real tMin = glm::clamp(std::min(tB0, tB1), 0.0, lenA);
                    Real tMax = glm::clamp(std::max(tB0, tB1), 0.0, lenA);

                    if (tMax - tMin > kEpsilon)
                    {
                        Vec3 samples[2] = { aP0 + dirA * tMin, aP0 + dirA * tMax };
                        ContactPoint rawPoints[2];
                        int hits = 0;
                        Vec3 sharedNormal(0.0, 1.0, 0.0);

                        for (const Vec3& sample : samples)
                        {
                            Vec3 closestOnB = ClosestPointOnSegment(sample, bP0, bP1);
                            Contact temp;
                            if (SphereSphere(sample, radiusA, closestOnB, radiusB, temp, margin))
                            {
                                sharedNormal = temp.normal;
                                rawPoints[hits] = temp.points[0];
                                ++hits;
                            }
                        }

                        if (hits > 0)
                        {
                            out.normal = sharedNormal;
                            out.pointCount = ManifoldReduction::Reduce(rawPoints, hits, sharedNormal, out.points);
                            return true;
                        }
                    }
                }
            }

            Vec3 closestA, closestB;
            ClosestPointsSegmentSegment(aP0, aP1, bP0, bP1, closestA, closestB);
            return SphereSphere(closestA, radiusA, closestB, radiusB, out, margin);
        }

        // normal points box -> capsule. Closest point on the capsule's
        // segment to the box found by alternating projection (segment ->
        // box surface -> segment), which converges quickly for two convex
        // shapes.
        bool BoxCapsule(const Vec3& boxPos, const Mat3& boxRot, const Vec3& boxHalf,
            const Vec3& capA, const Vec3& capB, Real capRadius,
            Contact& out, Real margin)
        {
            Vec3 segPoint = ClosestPointOnSegment(boxPos, capA, capB);
            for (int iter = 0; iter < 2; ++iter)
            {
                Vec3 d = segPoint - boxPos;
                Vec3 local(glm::dot(d, boxRot[0]), glm::dot(d, boxRot[1]), glm::dot(d, boxRot[2]));
                Vec3 clampedLocal = glm::clamp(local, -boxHalf, boxHalf);
                Vec3 boxPoint = boxPos + boxRot[0] * clampedLocal.x + boxRot[1] * clampedLocal.y + boxRot[2] * clampedLocal.z;
                segPoint = ClosestPointOnSegment(boxPoint, capA, capB);
            }

            if (!SphereBox(segPoint, capRadius, boxPos, boxRot, boxHalf, out, margin))
            {
                return false;
            }
            out.normal = -out.normal; // SphereBox gave capsule-point -> box

            // If the capsule's axis lies roughly in the contact plane
            // (resting along the face, not just touching near one end),
            // replace the single point above - which converges to the
            // AXIS MIDPOINT when the box sits symmetrically under the
            // capsule, not either end - with two points at the ends of
            // wherever the capsule's axis actually overlaps the box,
            // projected along the axis direction (same idea as the
            // capsule-capsule interval above, using the box's SAT-style
            // projected half-width in place of a second capsule).
            Vec3 axisVec = capB - capA;
            Real axisLen = glm::length(axisVec);
            if (axisLen > kEpsilon)
            {
                Vec3 axisDir = axisVec / axisLen;
                if (std::abs(glm::dot(axisDir, out.normal)) < 0.2)
                {
                    Real boxProjHalfWidth = boxHalf.x * std::abs(glm::dot(axisDir, boxRot[0]))
                        + boxHalf.y * std::abs(glm::dot(axisDir, boxRot[1]))
                        + boxHalf.z * std::abs(glm::dot(axisDir, boxRot[2]));
                    Real boxCenterT = glm::dot(boxPos - capA, axisDir);

                    Real tMin = glm::clamp(boxCenterT - boxProjHalfWidth, 0.0, axisLen);
                    Real tMax = glm::clamp(boxCenterT + boxProjHalfWidth, 0.0, axisLen);

                    if (tMax - tMin > kEpsilon)
                    {
                        Vec3 samples[2] = { capA + axisDir * tMin, capA + axisDir * tMax };
                        ContactPoint rawPoints[2];
                        int hits = 0;

                        for (const Vec3& sample : samples)
                        {
                            Contact temp;
                            if (SphereBox(sample, capRadius, boxPos, boxRot, boxHalf, temp, margin))
                            {
                                rawPoints[hits] = temp.points[0];
                                ++hits;
                            }
                        }

                        if (hits > 0)
                        {
                            out.pointCount = ManifoldReduction::Reduce(rawPoints, hits, out.normal, out.points);
                        }
                    }
                }
            }

            return true;
        }

        bool TestAxis(Vec3 axis, const Vec3& centerDelta,
            const Mat3& rotA, const Vec3& halfA,
            const Mat3& rotB, const Vec3& halfB,
            Real& outSignedDistance, Vec3& outNormalizedAxis)
        {
            Real lenSq = glm::dot(axis, axis);
            if (lenSq < kEpsilon * kEpsilon)
            {
                outSignedDistance = -std::numeric_limits<Real>::max(); // degenerate (near-parallel edges) - never the min
                return true;
            }

            axis /= std::sqrt(lenSq);
            outNormalizedAxis = axis;

            Real rA = halfA.x * std::abs(glm::dot(axis, rotA[0]))
                + halfA.y * std::abs(glm::dot(axis, rotA[1]))
                + halfA.z * std::abs(glm::dot(axis, rotA[2]));
            Real rB = halfB.x * std::abs(glm::dot(axis, rotB[0]))
                + halfB.y * std::abs(glm::dot(axis, rotB[1]))
                + halfB.z * std::abs(glm::dot(axis, rotB[2]));

            Real overlap = (rA + rB) - std::abs(glm::dot(centerDelta, axis));
            outSignedDistance = -overlap; // positive = separated, negative = overlap
            return true;
        }

        int ClipPolygonAgainstPlane(const Vec3* inPoly, int inCount, Vec3* outPoly,
            const Vec3& planeNormal, const Vec3& planePoint)
        {
            int outCount = 0;
            for (int i = 0; i < inCount; ++i)
            {
                const Vec3& current = inPoly[i];
                const Vec3& next = inPoly[(i + 1) % inCount];

                Real dCurrent = glm::dot(current - planePoint, planeNormal);
                Real dNext = glm::dot(next - planePoint, planeNormal);

                bool currentInside = dCurrent <= 0.0;
                if (currentInside)
                {
                    outPoly[outCount++] = current;
                }

                if (currentInside != (dNext <= 0.0))
                {
                    Real t = dCurrent / (dCurrent - dNext);
                    outPoly[outCount++] = current + t * (next - current);
                }
            }
            return outCount;
        }

        // Reference face's outward normal is refNormal (already A->B, or its
        // flip if the reference box is B). Clips the incident box's nearest
        // face against the reference face's 4 side planes so a flat box-on-
        // box rest produces multiple contact points, not one.
        void GenerateFaceContact(const Vec3& posA, const Mat3& rotA, const Vec3& halfA,
            const Vec3& posB, const Mat3& rotB, const Vec3& halfB,
            int minAxisIndex, const Vec3& normalAtoB, Contact& out, Real margin)
        {
            bool refIsA = minAxisIndex < 3;

            const Vec3& posRef = refIsA ? posA : posB;
            const Mat3& rotRef = refIsA ? rotA : rotB;
            const Vec3& halfRef = refIsA ? halfA : halfB;
            const Vec3& posInc = refIsA ? posB : posA;
            const Mat3& rotInc = refIsA ? rotB : rotA;
            const Vec3& halfInc = refIsA ? halfB : halfA;

            int refAxis = refIsA ? minAxisIndex : (minAxisIndex - 3);
            Vec3 refFaceNormal = refIsA ? normalAtoB : -normalAtoB;
            Vec3 refFaceCenter = posRef + refFaceNormal * halfRef[refAxis];

            int uRef = (refAxis + 1) % 3;
            int vRef = (refAxis + 2) % 3;

            int incAxis = 0;
            Real incSign = 1.0;
            Real minDot = std::numeric_limits<Real>::max();
            for (int m = 0; m < 3; ++m)
            {
                for (Real sign : { 1.0, -1.0 })
                {
                    Real d = glm::dot(rotInc[m] * sign, refFaceNormal);
                    if (d < minDot) { minDot = d; incAxis = m; incSign = sign; }
                }
            }

            Vec3 incFaceNormal = rotInc[incAxis] * incSign;
            Vec3 incFaceCenter = posInc + incFaceNormal * halfInc[incAxis];
            int uInc = (incAxis + 1) % 3;
            int vInc = (incAxis + 2) % 3;

            Vec3 poly[8] = {
                incFaceCenter + rotInc[uInc] * halfInc[uInc] + rotInc[vInc] * halfInc[vInc],
                incFaceCenter - rotInc[uInc] * halfInc[uInc] + rotInc[vInc] * halfInc[vInc],
                incFaceCenter - rotInc[uInc] * halfInc[uInc] - rotInc[vInc] * halfInc[vInc],
                incFaceCenter + rotInc[uInc] * halfInc[uInc] - rotInc[vInc] * halfInc[vInc],
            };
            int polyCount = 4;

            Vec3 sideNormals[4] = { rotRef[uRef], -rotRef[uRef], rotRef[vRef], -rotRef[vRef] };
            Vec3 sidePoints[4] = {
                refFaceCenter + rotRef[uRef] * halfRef[uRef],
                refFaceCenter - rotRef[uRef] * halfRef[uRef],
                refFaceCenter + rotRef[vRef] * halfRef[vRef],
                refFaceCenter - rotRef[vRef] * halfRef[vRef],
            };

            for (int p = 0; p < 4 && polyCount > 0; ++p)
            {
                Vec3 clipped[8];
                polyCount = ClipPolygonAgainstPlane(poly, polyCount, clipped, sideNormals[p], sidePoints[p]);
                for (int i = 0; i < polyCount; ++i) poly[i] = clipped[i];
            }

            ContactPoint rawPoints[8];
            int rawCount = 0;

            for (int i = 0; i < polyCount && rawCount < 8; ++i)
            {
                Real depth = -glm::dot(poly[i] - refFaceCenter, refFaceNormal);
                Real signedDist = -depth;
                if (signedDist <= margin)
                {
                    rawPoints[rawCount].position = poly[i];
                    rawPoints[rawCount].signedDistance = signedDist;
                    rawPoints[rawCount].localAnchorA = glm::transpose(rotA) * (poly[i] - posA);
                    rawPoints[rawCount].localAnchorB = glm::transpose(rotB) * (poly[i] - posB);
                    rawCount++;
                }
            }

            out.pointCount = ManifoldReduction::Reduce(rawPoints, rawCount, out.normal, out.points);
            if (out.pointCount == 0) // float-precision fallback; SAT already confirmed overlap
            {
                out.points[0].position = refFaceCenter;
                out.points[0].signedDistance = 0.0;
                out.points[0].localAnchorA = glm::transpose(rotA) * (refFaceCenter - posA);
                out.points[0].localAnchorB = glm::transpose(rotB) * (refFaceCenter - posB);
                out.pointCount = 1;
            }
        }

        // Edge-edge contact: rebuild the two specific edges involved (the
        // ones facing each other along the other two local axes) and take
        // their closest points.
        void GenerateEdgeContact(const Vec3& posA, const Mat3& rotA, const Vec3& halfA,
            const Vec3& posB, const Mat3& rotB, const Vec3& halfB,
            int axisA, int axisB, Real signedDist, Contact& out)
        {
            Vec3 centerDelta = posB - posA;
            Vec3 dLocalA(glm::dot(centerDelta, rotA[0]), glm::dot(centerDelta, rotA[1]), glm::dot(centerDelta, rotA[2]));
            Vec3 dLocalB(glm::dot(-centerDelta, rotB[0]), glm::dot(-centerDelta, rotB[1]), glm::dot(-centerDelta, rotB[2]));

            Vec3 baseA = posA;
            for (int m = 0; m < 3; ++m)
            {
                if (m != axisA) baseA += rotA[m] * ((dLocalA[m] >= 0.0) ? halfA[m] : -halfA[m]);
            }

            Vec3 baseB = posB;
            for (int m = 0; m < 3; ++m)
            {
                if (m != axisB) baseB += rotB[m] * ((dLocalB[m] >= 0.0) ? halfB[m] : -halfB[m]);
            }

            Vec3 closestA, closestB;
            ClosestPointsSegmentSegment(baseA - rotA[axisA] * halfA[axisA], baseA + rotA[axisA] * halfA[axisA],
                baseB - rotB[axisB] * halfB[axisB], baseB + rotB[axisB] * halfB[axisB],
                closestA, closestB);

            out.points[0].position = 0.5 * (closestA + closestB);
            out.points[0].signedDistance = signedDist;
            out.points[0].localAnchorA = glm::transpose(rotA) * (out.points[0].position - posA);
            out.points[0].localAnchorB = glm::transpose(rotB) * (out.points[0].position - posB);
            out.pointCount = 1;
        }

        // Exact 3D SAT: 6 face axes + 9 edge-edge cross-product axes.
        bool BoxVsBox(const Vec3& posA, const Mat3& rotA, const Vec3& halfA,
            const Vec3& posB, const Mat3& rotB, const Vec3& halfB,
            Contact& out, Real margin)
        {
            Vec3 centerDelta = posB - posA;

            Vec3 axes[15];
            for (int i = 0; i < 3; ++i) axes[i] = rotA[i];
            for (int i = 0; i < 3; ++i) axes[3 + i] = rotB[i];
            int k = 6;
            for (int i = 0; i < 3; ++i)
            {
                for (int j = 0; j < 3; ++j)
                {
                    axes[k++] = glm::cross(rotA[i], rotB[j]);
                }
            }

            Real maxSignedDist = -std::numeric_limits<Real>::max();
            int bestAxisIndex = -1;
            Vec3 bestAxis(0.0);

            Real bestFaceSignedDist = -std::numeric_limits<Real>::max();
            int bestFaceAxisIndex = -1;
            Vec3 bestFaceAxis(0.0);

            for (int i = 0; i < 15; ++i)
            {
                Real signedDist;
                Vec3 normalizedAxis;
                TestAxis(axes[i], centerDelta, rotA, halfA, rotB, halfB, signedDist, normalizedAxis);

                if (signedDist > margin)
                {
                    return false; // Separating axis found
                }

                if (i < 6)
                {
                    if (signedDist > bestFaceSignedDist)
                    {
                        bestFaceSignedDist = signedDist;
                        bestFaceAxisIndex = i;
                        bestFaceAxis = normalizedAxis;
                    }
                }

                if (signedDist > maxSignedDist)
                {
                    maxSignedDist = signedDist;
                    bestAxisIndex = i;
                    bestAxis = normalizedAxis;
                }
            }

            // Apply Face Preference Bias: An edge-edge axis must strictly beat best face axis by bias
            if (bestAxisIndex >= 6 && bestFaceAxisIndex != -1)
            {
                if (maxSignedDist < bestFaceSignedDist + kFacePreferenceBias)
                {
                    bestAxisIndex = bestFaceAxisIndex;
                    bestAxis = bestFaceAxis;
                    maxSignedDist = bestFaceSignedDist;
                }
            }

            if (glm::dot(bestAxis, centerDelta) < 0.0)
            {
                bestAxis = -bestAxis;
            }

            out.normal = bestAxis;

            if (bestAxisIndex < 6)
            {
                GenerateFaceContact(posA, rotA, halfA, posB, rotB, halfB, bestAxisIndex, bestAxis, out, margin);
            }
            else
            {
                GenerateEdgeContact(posA, rotA, halfA, posB, rotB, halfB,
                    (bestAxisIndex - 6) / 3, (bestAxisIndex - 6) % 3, maxSignedDist, out);
            }

            return (out.pointCount > 0);
        }
    }

    bool TestCollision(const ColliderPose& a, const ColliderPose& b, Contact& outContact, Real speculativeMargin)
    {
        outContact.pointCount = 0;

        if (a.shape == ColliderShape::Sphere && b.shape == ColliderShape::Sphere)
        {
            return SphereSphere(a.position, a.size.x, b.position, b.size.x, outContact, speculativeMargin);
        }

        if (a.shape == ColliderShape::Sphere && b.shape == ColliderShape::Box)
        {
            return SphereBox(a.position, a.size.x, b.position, b.rotation, b.size, outContact, speculativeMargin);
        }

        if (a.shape == ColliderShape::Box && b.shape == ColliderShape::Sphere)
        {
            bool hit = SphereBox(b.position, b.size.x, a.position, a.rotation, a.size, outContact, speculativeMargin);
            if (hit)
            {
                outContact.normal = -outContact.normal;
            }
            return hit;
        }

        if (a.shape == ColliderShape::Sphere && b.shape == ColliderShape::Capsule)
        {
            Vec3 capA = b.position - b.rotation[1] * b.size.y;
            Vec3 capB = b.position + b.rotation[1] * b.size.y;
            return SphereCapsule(a.position, a.size.x, capA, capB, b.size.x, outContact, speculativeMargin);
        }

        if (a.shape == ColliderShape::Capsule && b.shape == ColliderShape::Sphere)
        {
            Vec3 capA = a.position - a.rotation[1] * a.size.y;
            Vec3 capB = a.position + a.rotation[1] * a.size.y;
            bool hit = SphereCapsule(b.position, b.size.x, capA, capB, a.size.x, outContact, speculativeMargin);
            if (hit)
            {
                outContact.normal = -outContact.normal;
            }
            return hit;
        }

        if (a.shape == ColliderShape::Capsule && b.shape == ColliderShape::Capsule)
        {
            Vec3 aP0 = a.position - a.rotation[1] * a.size.y;
            Vec3 aP1 = a.position + a.rotation[1] * a.size.y;
            Vec3 bP0 = b.position - b.rotation[1] * b.size.y;
            Vec3 bP1 = b.position + b.rotation[1] * b.size.y;
            return CapsuleCapsule(aP0, aP1, a.size.x, bP0, bP1, b.size.x, outContact, speculativeMargin);
        }

        if (a.shape == ColliderShape::Box && b.shape == ColliderShape::Capsule)
        {
            Vec3 capA = b.position - b.rotation[1] * b.size.y;
            Vec3 capB = b.position + b.rotation[1] * b.size.y;
            return BoxCapsule(a.position, a.rotation, a.size, capA, capB, b.size.x, outContact, speculativeMargin);
        }

        if (a.shape == ColliderShape::Capsule && b.shape == ColliderShape::Box)
        {
            Vec3 capA = a.position - a.rotation[1] * a.size.y;
            Vec3 capB = a.position + a.rotation[1] * a.size.y;
            bool hit = BoxCapsule(b.position, b.rotation, b.size, capA, capB, a.size.x, outContact, speculativeMargin);
            if (hit)
            {
                outContact.normal = -outContact.normal;
            }
            return hit;
        }

        return BoxVsBox(a.position, a.rotation, a.size, b.position, b.rotation, b.size, outContact, speculativeMargin);
    }
}