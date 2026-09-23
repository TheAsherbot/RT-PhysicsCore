#include "RT-PhysicsCore/physics/collision/NarrowPhase.h"

#include <cmath>
#include <limits>

namespace RT_PhysicsCore
{
    namespace
    {
        constexpr float kEpsilon = 1e-6f;

        glm::vec3 ClosestPointOnSegment(const glm::vec3& p, const glm::vec3& a, const glm::vec3& b)
        {
            glm::vec3 ab = b - a;
            float abLenSq = glm::dot(ab, ab);
            if (abLenSq < kEpsilon)
                return a;

            float t = glm::clamp(glm::dot(p - a, ab) / abLenSq, 0.0f, 1.0f);
            return a + t * ab;
        }

        // Closest points between two 3D segments (Ericson, "Real-Time
        // Collision Detection" 5.1.9), handling degenerate/parallel cases.
        void ClosestPointsSegmentSegment(const glm::vec3& a0, const glm::vec3& a1,
                                          const glm::vec3& b0, const glm::vec3& b1,
                                          glm::vec3& outA, glm::vec3& outB)
        {
            glm::vec3 d1 = a1 - a0;
            glm::vec3 d2 = b1 - b0;
            glm::vec3 r = a0 - b0;

            float a = glm::dot(d1, d1);
            float e = glm::dot(d2, d2);
            float f = glm::dot(d2, r);

            float s, t;

            if (a <= kEpsilon && e <= kEpsilon)
            {
                outA = a0;
                outB = b0;
                return;
            }

            if (a <= kEpsilon)
            {
                s = 0.0f;
                t = glm::clamp(f / e, 0.0f, 1.0f);
            }
            else
            {
                float c = glm::dot(d1, r);
                if (e <= kEpsilon)
                {
                    t = 0.0f;
                    s = glm::clamp(-c / a, 0.0f, 1.0f);
                }
                else
                {
                    float b = glm::dot(d1, d2);
                    float denom = a * e - b * b;

                    s = (denom > kEpsilon) ? glm::clamp((b * f - c * e) / denom, 0.0f, 1.0f) : 0.0f;
                    t = (b * s + f) / e;

                    if (t < 0.0f)
                    {
                        t = 0.0f;
                        s = glm::clamp(-c / a, 0.0f, 1.0f);
                    }
                    else if (t > 1.0f)
                    {
                        t = 1.0f;
                        s = glm::clamp((b - c) / a, 0.0f, 1.0f);
                    }
                }
            }

            outA = a0 + d1 * s;
            outB = b0 + d2 * t;
        }

        bool SphereSphere(const glm::vec3& posA, float radiusA, const glm::vec3& posB, float radiusB, Contact& out)
        {
            glm::vec3 delta = posB - posA;
            float distSq = glm::dot(delta, delta);
            float radiusSum = radiusA + radiusB;
            if (distSq >= radiusSum * radiusSum)
                return false;

            float dist = std::sqrt(distSq);
            glm::vec3 normal = (dist > kEpsilon) ? (delta / dist) : glm::vec3(0.0f, 1.0f, 0.0f);

            out.normal = normal;
            out.points[0] = 0.5f * ((posA + normal * radiusA) + (posB - normal * radiusB));
            out.penetrations[0] = radiusSum - dist;
            out.pointCount = 1;
            return true;
        }

        // normal points sphere -> box.
        bool SphereBox(const glm::vec3& sphereCenter, float sphereRadius,
                        const glm::vec3& boxPos, const glm::mat3& boxRot, const glm::vec3& boxHalf,
                        Contact& out)
        {
            glm::vec3 d = sphereCenter - boxPos;
            glm::vec3 local(glm::dot(d, boxRot[0]), glm::dot(d, boxRot[1]), glm::dot(d, boxRot[2]));
            glm::vec3 clampedLocal = glm::clamp(local, -boxHalf, boxHalf);

            glm::vec3 normalLocal;
            glm::vec3 closestLocal;
            float penetration;

            if (clampedLocal == local)
            {
                // Center is inside the box - push out toward the nearest face.
                glm::vec3 faceDist = boxHalf - glm::abs(local);
                int axis = 0;
                if (faceDist.y < faceDist.x) axis = 1;
                if (faceDist.z < faceDist[axis]) axis = 2;

                float sign = (local[axis] >= 0.0f) ? 1.0f : -1.0f;
                normalLocal = glm::vec3(0.0f);
                normalLocal[axis] = sign;
                closestLocal = local;
                closestLocal[axis] = sign * boxHalf[axis];
                penetration = faceDist[axis] + sphereRadius;
            }
            else
            {
                glm::vec3 diff = local - clampedLocal; // box surface point -> sphere
                float dist = glm::length(diff);
                if (dist >= sphereRadius)
                    return false;

                normalLocal = (dist > kEpsilon) ? -(diff / dist) : glm::vec3(1.0f, 0.0f, 0.0f);
                closestLocal = clampedLocal;
                penetration = sphereRadius - dist;
            }

            out.normal = boxRot[0] * normalLocal.x + boxRot[1] * normalLocal.y + boxRot[2] * normalLocal.z;
            out.points[0] = boxPos + boxRot[0] * closestLocal.x + boxRot[1] * closestLocal.y + boxRot[2] * closestLocal.z;
            out.penetrations[0] = penetration;
            out.pointCount = 1;
            return true;
        }

        bool SphereCapsule(const glm::vec3& sphereCenter, float sphereRadius,
                            const glm::vec3& capA, const glm::vec3& capB, float capRadius,
                            Contact& out)
        {
            glm::vec3 closest = ClosestPointOnSegment(sphereCenter, capA, capB);
            return SphereSphere(sphereCenter, sphereRadius, closest, capRadius, out);
        }

        bool CapsuleCapsule(const glm::vec3& aP0, const glm::vec3& aP1, float radiusA,
                             const glm::vec3& bP0, const glm::vec3& bP1, float radiusB,
                             Contact& out)
        {
            glm::vec3 dA = aP1 - aP0;
            glm::vec3 dB = bP1 - bP0;
            float lenA = glm::length(dA);
            float lenB = glm::length(dB);

            // Near-parallel segments: a single closest-point pair leaves
            // one contact for what's actually a resting-along-each-other
            // overlap, which lets the pair roll. Generate two points
            // spanning the overlapping interval instead.
            if (lenA > kEpsilon && lenB > kEpsilon)
            {
                glm::vec3 dirA = dA / lenA;
                glm::vec3 dirB = dB / lenB;

                if (std::abs(glm::dot(dirA, dirB)) > 0.999f)
                {
                    float tB0 = glm::dot(bP0 - aP0, dirA);
                    float tB1 = glm::dot(bP1 - aP0, dirA);
                    float tMin = glm::clamp(std::min(tB0, tB1), 0.0f, lenA);
                    float tMax = glm::clamp(std::max(tB0, tB1), 0.0f, lenA);

                    if (tMax - tMin > kEpsilon)
                    {
                        glm::vec3 samples[2] = { aP0 + dirA * tMin, aP0 + dirA * tMax };
                        Contact points[2];
                        int hits = 0;
                        for (glm::vec3& sample : samples)
                        {
                            glm::vec3 closestOnB = ClosestPointOnSegment(sample, bP0, bP1);
                            if (SphereSphere(sample, radiusA, closestOnB, radiusB, points[hits]))
                                ++hits;
                        }

                        if (hits > 0)
                        {
                            out.normal = points[0].normal;
                            out.pointCount = 0;
                            for (int i = 0; i < hits; ++i)
                            {
                                out.points[out.pointCount] = points[i].points[0];
                                out.penetrations[out.pointCount] = points[i].penetrations[0];
                                ++out.pointCount;
                            }
                            return true;
                        }
                    }
                }
            }

            glm::vec3 closestA, closestB;
            ClosestPointsSegmentSegment(aP0, aP1, bP0, bP1, closestA, closestB);
            return SphereSphere(closestA, radiusA, closestB, radiusB, out);
        }

        // normal points box -> capsule. Closest point on the capsule's
        // segment to the box found by alternating projection (segment ->
        // box surface -> segment), which converges quickly for two convex
        // shapes.
        bool BoxCapsule(const glm::vec3& boxPos, const glm::mat3& boxRot, const glm::vec3& boxHalf,
                         const glm::vec3& capA, const glm::vec3& capB, float capRadius,
                         Contact& out)
        {
            glm::vec3 segPoint = ClosestPointOnSegment(boxPos, capA, capB);
            for (int iter = 0; iter < 2; ++iter)
            {
                glm::vec3 d = segPoint - boxPos;
                glm::vec3 local(glm::dot(d, boxRot[0]), glm::dot(d, boxRot[1]), glm::dot(d, boxRot[2]));
                glm::vec3 clampedLocal = glm::clamp(local, -boxHalf, boxHalf);
                glm::vec3 boxPoint = boxPos + boxRot[0] * clampedLocal.x + boxRot[1] * clampedLocal.y + boxRot[2] * clampedLocal.z;
                segPoint = ClosestPointOnSegment(boxPoint, capA, capB);
            }

            if (!SphereBox(segPoint, capRadius, boxPos, boxRot, boxHalf, out))
                return false;
            out.normal = -out.normal; // SphereBox gave capsule-point -> box

            // If the capsule's axis lies roughly in the contact plane
            // (resting along the face, not just touching near one end),
            // add a second point at the far end of the capsule instead of
            // leaving a single point that lets it rock.
            glm::vec3 axisVec = capB - capA;
            float axisLen = glm::length(axisVec);
            if (axisLen > kEpsilon)
            {
                glm::vec3 axisDir = axisVec / axisLen;
                if (std::abs(glm::dot(axisDir, out.normal)) < 0.2f)
                {
                    float tSeg = glm::dot(segPoint - capA, axisDir);
                    glm::vec3 farPoint = capA + axisDir * ((tSeg < axisLen * 0.5f) ? axisLen : 0.0f);

                    glm::vec3 d = farPoint - boxPos;
                    glm::vec3 local(glm::dot(d, boxRot[0]), glm::dot(d, boxRot[1]), glm::dot(d, boxRot[2]));
                    glm::vec3 clampedLocal = glm::clamp(local, -boxHalf, boxHalf);
                    glm::vec3 boxPoint = boxPos + boxRot[0] * clampedLocal.x + boxRot[1] * clampedLocal.y + boxRot[2] * clampedLocal.z;

                    float dist = glm::length(farPoint - boxPoint);
                    if (dist < capRadius)
                    {
                        out.points[1] = boxPoint;
                        out.penetrations[1] = capRadius - dist;
                        out.pointCount = 2;
                    }
                }
            }

            return true;
        }

        bool TestAxis(glm::vec3 axis, const glm::vec3& centerDelta,
                      const glm::mat3& rotA, const glm::vec3& halfA,
                      const glm::mat3& rotB, const glm::vec3& halfB,
                      float& outOverlap, glm::vec3& outNormalizedAxis)
        {
            float lenSq = glm::dot(axis, axis);
            if (lenSq < kEpsilon * kEpsilon)
            {
                outOverlap = std::numeric_limits<float>::max(); // degenerate (near-parallel edges) - never the min
                return true;
            }

            axis /= std::sqrt(lenSq);
            outNormalizedAxis = axis;

            float rA = halfA.x * std::abs(glm::dot(axis, rotA[0]))
                     + halfA.y * std::abs(glm::dot(axis, rotA[1]))
                     + halfA.z * std::abs(glm::dot(axis, rotA[2]));
            float rB = halfB.x * std::abs(glm::dot(axis, rotB[0]))
                     + halfB.y * std::abs(glm::dot(axis, rotB[1]))
                     + halfB.z * std::abs(glm::dot(axis, rotB[2]));

            outOverlap = (rA + rB) - std::abs(glm::dot(centerDelta, axis));
            return outOverlap >= 0.0f;
        }

        int ClipPolygonAgainstPlane(const glm::vec3* inPoly, int inCount, glm::vec3* outPoly,
                                     const glm::vec3& planeNormal, const glm::vec3& planePoint)
        {
            int outCount = 0;
            for (int i = 0; i < inCount; ++i)
            {
                const glm::vec3& current = inPoly[i];
                const glm::vec3& next = inPoly[(i + 1) % inCount];

                float dCurrent = glm::dot(current - planePoint, planeNormal);
                float dNext = glm::dot(next - planePoint, planeNormal);

                bool currentInside = dCurrent <= 0.0f;
                if (currentInside)
                    outPoly[outCount++] = current;

                if (currentInside != (dNext <= 0.0f))
                {
                    float t = dCurrent / (dCurrent - dNext);
                    outPoly[outCount++] = current + t * (next - current);
                }
            }
            return outCount;
        }

        // Reference face's outward normal is refNormal (already A->B, or its
        // flip if the reference box is B). Clips the incident box's nearest
        // face against the reference face's 4 side planes so a flat box-on-
        // box rest produces multiple contact points, not one.
        void GenerateFaceContact(const glm::vec3& posA, const glm::mat3& rotA, const glm::vec3& halfA,
                                  const glm::vec3& posB, const glm::mat3& rotB, const glm::vec3& halfB,
                                  int minAxisIndex, const glm::vec3& normalAtoB, Contact& out)
        {
            bool refIsA = minAxisIndex < 3;

            const glm::vec3& posRef = refIsA ? posA : posB;
            const glm::mat3& rotRef = refIsA ? rotA : rotB;
            const glm::vec3& halfRef = refIsA ? halfA : halfB;
            const glm::vec3& posInc = refIsA ? posB : posA;
            const glm::mat3& rotInc = refIsA ? rotB : rotA;
            const glm::vec3& halfInc = refIsA ? halfB : halfA;

            int refAxis = refIsA ? minAxisIndex : (minAxisIndex - 3);
            glm::vec3 refFaceNormal = refIsA ? normalAtoB : -normalAtoB;
            glm::vec3 refFaceCenter = posRef + refFaceNormal * halfRef[refAxis];

            int uRef = (refAxis + 1) % 3;
            int vRef = (refAxis + 2) % 3;

            int incAxis = 0;
            float incSign = 1.0f;
            float minDot = std::numeric_limits<float>::max();
            for (int m = 0; m < 3; ++m)
            {
                for (float sign : { 1.0f, -1.0f })
                {
                    float d = glm::dot(rotInc[m] * sign, refFaceNormal);
                    if (d < minDot) { minDot = d; incAxis = m; incSign = sign; }
                }
            }
            glm::vec3 incFaceNormal = rotInc[incAxis] * incSign;
            glm::vec3 incFaceCenter = posInc + incFaceNormal * halfInc[incAxis];
            int uInc = (incAxis + 1) % 3;
            int vInc = (incAxis + 2) % 3;

            glm::vec3 poly[8] = {
                incFaceCenter + rotInc[uInc] * halfInc[uInc] + rotInc[vInc] * halfInc[vInc],
                incFaceCenter - rotInc[uInc] * halfInc[uInc] + rotInc[vInc] * halfInc[vInc],
                incFaceCenter - rotInc[uInc] * halfInc[uInc] - rotInc[vInc] * halfInc[vInc],
                incFaceCenter + rotInc[uInc] * halfInc[uInc] - rotInc[vInc] * halfInc[vInc],
            };
            int polyCount = 4;

            glm::vec3 sideNormals[4] = { rotRef[uRef], -rotRef[uRef], rotRef[vRef], -rotRef[vRef] };
            glm::vec3 sidePoints[4] = {
                refFaceCenter + rotRef[uRef] * halfRef[uRef],
                refFaceCenter - rotRef[uRef] * halfRef[uRef],
                refFaceCenter + rotRef[vRef] * halfRef[vRef],
                refFaceCenter - rotRef[vRef] * halfRef[vRef],
            };

            for (int p = 0; p < 4 && polyCount > 0; ++p)
            {
                glm::vec3 clipped[8];
                polyCount = ClipPolygonAgainstPlane(poly, polyCount, clipped, sideNormals[p], sidePoints[p]);
                for (int i = 0; i < polyCount; ++i) poly[i] = clipped[i];
            }

            out.pointCount = 0;
            for (int i = 0; i < polyCount && out.pointCount < kMaxContactPoints; ++i)
            {
                float depth = -glm::dot(poly[i] - refFaceCenter, refFaceNormal);
                if (depth > 0.0f)
                {
                    out.points[out.pointCount] = poly[i];
                    out.penetrations[out.pointCount] = depth;
                    out.pointCount++;
                }
            }

            if (out.pointCount == 0) // float-precision fallback; SAT already confirmed overlap
            {
                out.points[0] = refFaceCenter;
                out.penetrations[0] = 0.0f;
                out.pointCount = 1;
            }
        }

        // Edge-edge contact: rebuild the two specific edges involved (the
        // ones facing each other along the other two local axes) and take
        // their closest points.
        void GenerateEdgeContact(const glm::vec3& posA, const glm::mat3& rotA, const glm::vec3& halfA,
                                  const glm::vec3& posB, const glm::mat3& rotB, const glm::vec3& halfB,
                                  int axisA, int axisB, float penetration, Contact& out)
        {
            glm::vec3 centerDelta = posB - posA;
            glm::vec3 dLocalA(glm::dot(centerDelta, rotA[0]), glm::dot(centerDelta, rotA[1]), glm::dot(centerDelta, rotA[2]));
            glm::vec3 dLocalB(glm::dot(-centerDelta, rotB[0]), glm::dot(-centerDelta, rotB[1]), glm::dot(-centerDelta, rotB[2]));

            glm::vec3 baseA = posA;
            for (int m = 0; m < 3; ++m)
                if (m != axisA) baseA += rotA[m] * ((dLocalA[m] >= 0.0f) ? halfA[m] : -halfA[m]);

            glm::vec3 baseB = posB;
            for (int m = 0; m < 3; ++m)
                if (m != axisB) baseB += rotB[m] * ((dLocalB[m] >= 0.0f) ? halfB[m] : -halfB[m]);

            glm::vec3 closestA, closestB;
            ClosestPointsSegmentSegment(baseA - rotA[axisA] * halfA[axisA], baseA + rotA[axisA] * halfA[axisA],
                                         baseB - rotB[axisB] * halfB[axisB], baseB + rotB[axisB] * halfB[axisB],
                                         closestA, closestB);

            out.points[0] = 0.5f * (closestA + closestB);
            out.penetrations[0] = penetration;
            out.pointCount = 1;
        }

        // Exact 3D SAT: 6 face axes + 9 edge-edge cross-product axes.
        bool BoxVsBox(const glm::vec3& posA, const glm::mat3& rotA, const glm::vec3& halfA,
                      const glm::vec3& posB, const glm::mat3& rotB, const glm::vec3& halfB,
                      Contact& out)
        {
            glm::vec3 centerDelta = posB - posA;

            glm::vec3 axes[15];
            for (int i = 0; i < 3; ++i) axes[i] = rotA[i];
            for (int i = 0; i < 3; ++i) axes[3 + i] = rotB[i];
            int k = 6;
            for (int i = 0; i < 3; ++i)
                for (int j = 0; j < 3; ++j)
                    axes[k++] = glm::cross(rotA[i], rotB[j]);

            float minOverlap = std::numeric_limits<float>::max();
            int minAxisIndex = -1;
            glm::vec3 minAxis(0.0f);

            for (int i = 0; i < 15; ++i)
            {
                float overlap;
                glm::vec3 normalizedAxis;
                if (!TestAxis(axes[i], centerDelta, rotA, halfA, rotB, halfB, overlap, normalizedAxis))
                    return false;

                if (overlap < minOverlap)
                {
                    minOverlap = overlap;
                    minAxisIndex = i;
                    minAxis = normalizedAxis;
                }
            }

            if (glm::dot(minAxis, centerDelta) < 0.0f)
                minAxis = -minAxis;

            out.normal = minAxis;

            if (minAxisIndex < 6)
            {
                GenerateFaceContact(posA, rotA, halfA, posB, rotB, halfB, minAxisIndex, minAxis, out);
            }
            else
            {
                GenerateEdgeContact(posA, rotA, halfA, posB, rotB, halfB,
                                     (minAxisIndex - 6) / 3, (minAxisIndex - 6) % 3, minOverlap, out);
            }

            return true;
        }
    }

    bool TestCollision(const ColliderPose& a, const ColliderPose& b, Contact& out)
    {
        out.pointCount = 0;

        if (a.shape == ColliderShape::Sphere && b.shape == ColliderShape::Sphere)
            return SphereSphere(a.position, a.size.x, b.position, b.size.x, out);

        if (a.shape == ColliderShape::Sphere && b.shape == ColliderShape::Box)
            return SphereBox(a.position, a.size.x, b.position, b.rotation, b.size, out);

        if (a.shape == ColliderShape::Box && b.shape == ColliderShape::Sphere)
        {
            bool hit = SphereBox(b.position, b.size.x, a.position, a.rotation, a.size, out);
            if (hit) out.normal = -out.normal;
            return hit;
        }

        if (a.shape == ColliderShape::Sphere && b.shape == ColliderShape::Capsule)
        {
            glm::vec3 capA = b.position - b.rotation[1] * b.size.y;
            glm::vec3 capB = b.position + b.rotation[1] * b.size.y;
            return SphereCapsule(a.position, a.size.x, capA, capB, b.size.x, out);
        }

        if (a.shape == ColliderShape::Capsule && b.shape == ColliderShape::Sphere)
        {
            glm::vec3 capA = a.position - a.rotation[1] * a.size.y;
            glm::vec3 capB = a.position + a.rotation[1] * a.size.y;
            bool hit = SphereCapsule(b.position, b.size.x, capA, capB, a.size.x, out);
            if (hit) out.normal = -out.normal;
            return hit;
        }

        if (a.shape == ColliderShape::Capsule && b.shape == ColliderShape::Capsule)
        {
            glm::vec3 aP0 = a.position - a.rotation[1] * a.size.y;
            glm::vec3 aP1 = a.position + a.rotation[1] * a.size.y;
            glm::vec3 bP0 = b.position - b.rotation[1] * b.size.y;
            glm::vec3 bP1 = b.position + b.rotation[1] * b.size.y;
            return CapsuleCapsule(aP0, aP1, a.size.x, bP0, bP1, b.size.x, out);
        }

        if (a.shape == ColliderShape::Box && b.shape == ColliderShape::Capsule)
        {
            glm::vec3 capA = b.position - b.rotation[1] * b.size.y;
            glm::vec3 capB = b.position + b.rotation[1] * b.size.y;
            return BoxCapsule(a.position, a.rotation, a.size, capA, capB, b.size.x, out);
        }

        if (a.shape == ColliderShape::Capsule && b.shape == ColliderShape::Box)
        {
            glm::vec3 capA = a.position - a.rotation[1] * a.size.y;
            glm::vec3 capB = a.position + a.rotation[1] * a.size.y;
            bool hit = BoxCapsule(b.position, b.rotation, b.size, capA, capB, a.size.x, out);
            if (hit) out.normal = -out.normal;
            return hit;
        }

        return BoxVsBox(a.position, a.rotation, a.size, b.position, b.rotation, b.size, out);
    }
}
