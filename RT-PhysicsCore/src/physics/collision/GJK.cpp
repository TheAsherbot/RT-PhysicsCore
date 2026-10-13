/**
 * @file GJK.cpp
 * @brief Implementation of GJK distance algorithm using sub-simplex Voronoi region projection.
 */

#include "RT-PhysicsCore/physics/collision/GJK.h"
#include <algorithm>
#include <cmath>

namespace RT_PhysicsCore
{
    namespace
    {
        constexpr Real kGjkEpsilon = 1e-12;
        constexpr Real kGjkEpsilonSq = 1e-24;
    }

    bool GJK::SolvePoint(Simplex& simplex, Vec3& searchDirection, Vec3& outClosestPoint,
        Vec3& outClosestA, Vec3& outClosestB)
    {
        outClosestPoint = simplex.vertices[0].v;
        outClosestA = simplex.vertices[0].pA;
        outClosestB = simplex.vertices[0].pB;
        searchDirection = -outClosestPoint;
        return (glm::dot(outClosestPoint, outClosestPoint) < kGjkEpsilonSq);
    }

    bool GJK::SolveSegment(Simplex& simplex, Vec3& searchDirection, Vec3& outClosestPoint,
        Vec3& outClosestA, Vec3& outClosestB)
    {
        const SimplexVertex& A = simplex.vertices[1];
        const SimplexVertex& B = simplex.vertices[0];

        Vec3 ab = B.v - A.v;
        Vec3 ao = -A.v;

        Real abLenSq = glm::dot(ab, ab);
        if (abLenSq < kGjkEpsilonSq)
        {
            simplex.count = 1;
            return SolvePoint(simplex, searchDirection, outClosestPoint, outClosestA, outClosestB);
        }

        Real u = glm::dot(ao, ab);
        if (u <= 0.0)
        {
            // Region A: closest point is A
            simplex.vertices[0] = A;
            simplex.count = 1;
            outClosestPoint = A.v;
            outClosestA = A.pA;
            outClosestB = A.pB;
            searchDirection = -outClosestPoint;
            return false;
        }

        if (u >= abLenSq)
        {
            // Region B: closest point is B
            simplex.count = 1; // B is already at index 0
            outClosestPoint = B.v;
            outClosestA = B.pA;
            outClosestB = B.pB;
            searchDirection = -outClosestPoint;
            return false;
        }

        // Segment interior
        Real t = u / abLenSq;
        outClosestPoint = A.v + t * ab;
        outClosestA = A.pA + t * (B.pA - A.pA);
        outClosestB = A.pB + t * (B.pB - A.pB);

        // Search direction orthogonal to segment pointing toward origin: (AB x AO) x AB
        Vec3 normal = glm::cross(ab, ao);
        searchDirection = glm::cross(normal, ab);
        if (glm::dot(searchDirection, searchDirection) < kGjkEpsilonSq)
        {
            // Origin lies collinearly on the segment line
            searchDirection = -outClosestPoint;
        }

        return (glm::dot(outClosestPoint, outClosestPoint) < kGjkEpsilonSq);
    }

    bool GJK::SolveTriangle(Simplex& simplex, Vec3& searchDirection, Vec3& outClosestPoint,
        Vec3& outClosestA, Vec3& outClosestB)
    {
        const SimplexVertex& A = simplex.vertices[2];
        const SimplexVertex& B = simplex.vertices[1];
        const SimplexVertex& C = simplex.vertices[0];

        Vec3 ab = B.v - A.v;
        Vec3 ac = C.v - A.v;
        Vec3 ao = -A.v;

        Real d1 = glm::dot(ab, ao);
        Real d2 = glm::dot(ac, ao);
        if (d1 <= 0.0 && d2 <= 0.0)
        {
            // Vertex A region
            simplex.vertices[0] = A;
            simplex.count = 1;
            outClosestPoint = A.v;
            outClosestA = A.pA;
            outClosestB = A.pB;
            searchDirection = -outClosestPoint;
            return false;
        }

        Real d3 = glm::dot(ab, B.v);
        Real d4 = glm::dot(ac, B.v);
        if (d3 >= 0.0 && d4 <= d3)
        {
            // Vertex B region
            simplex.vertices[0] = B;
            simplex.count = 1;
            outClosestPoint = B.v;
            outClosestA = B.pA;
            outClosestB = B.pB;
            searchDirection = -outClosestPoint;
            return false;
        }

        // Edge AB region
        Real vc = d1 * (-d4) - (-d3) * d2;
        if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0)
        {
            Real v = d1 / (d1 - d3);
            simplex.vertices[0] = B;
            simplex.vertices[1] = A;
            simplex.count = 2;
            outClosestPoint = A.v + v * ab;
            outClosestA = A.pA + v * (B.pA - A.pA);
            outClosestB = A.pB + v * (B.pB - A.pB);
            searchDirection = glm::cross(glm::cross(ab, ao), ab);
            return false;
        }

        Real d5 = glm::dot(ab, C.v);
        Real d6 = glm::dot(ac, C.v);
        if (d6 >= 0.0 && d5 <= d6)
        {
            // Vertex C region
            simplex.count = 1;
            outClosestPoint = C.v;
            outClosestA = C.pA;
            outClosestB = C.pB;
            searchDirection = -outClosestPoint;
            return false;
        }

        // Edge AC region
        Real vb = d5 * d2 - d1 * d6;
        if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0)
        {
            Real w = d2 / (d2 - d6);
            simplex.vertices[1] = A; // C is already at 0
            simplex.count = 2;
            outClosestPoint = A.v + w * ac;
            outClosestA = A.pA + w * (C.pA - A.pA);
            outClosestB = A.pB + w * (C.pB - A.pB);
            searchDirection = glm::cross(glm::cross(ac, ao), ac);
            return false;
        }

        // Face ABC region
        Vec3 n = glm::cross(ab, ac);
        Real nLenSq = glm::dot(n, n);
        if (nLenSq < kGjkEpsilonSq)
        {
            // Degenerate coplanar triangle; reduce to segment AB
            simplex.count = 2;
            return SolveSegment(simplex, searchDirection, outClosestPoint, outClosestA, outClosestB);
        }

        Real va = d3 * d6 - d5 * d4;
        Real denom = 1.0 / (va + vb + vc);
        Real uA = va * denom;
        Real uB = vb * denom;
        Real uC = vc * denom;

        outClosestPoint = uA * A.v + uB * B.v + uC * C.v;
        outClosestA = uA * A.pA + uB * B.pA + uC * C.pA;
        outClosestB = uA * A.pB + uB * B.pB + uC * C.pB;

        // Search direction is triangle normal pointing toward origin
        searchDirection = (glm::dot(n, ao) >= 0.0) ? n : -n;
        return (glm::dot(outClosestPoint, outClosestPoint) < kGjkEpsilonSq);
    }

    bool GJK::SolveTetrahedron(Simplex& simplex, Vec3& searchDirection, Vec3& outClosestPoint,
        Vec3& outClosestA, Vec3& outClosestB)
    {
        // 4 faces of tetrahedron (winding ordered outward)
        // If origin lies on outward side of a face, reduce to that triangle face
        const SimplexVertex& A = simplex.vertices[3];
        const SimplexVertex& B = simplex.vertices[2];
        const SimplexVertex& C = simplex.vertices[1];
        const SimplexVertex& D = simplex.vertices[0];

        Vec3 ab = B.v - A.v;
        Vec3 ac = C.v - A.v;
        Vec3 ad = D.v - A.v;
        Vec3 ao = -A.v;

        Vec3 nABC = glm::cross(ab, ac);
        Vec3 nACD = glm::cross(ac, ad);
        Vec3 nADB = glm::cross(ad, ab);

        // Ensure outward pointing relative to opposite vertex
        if (glm::dot(nABC, ad) > 0.0) nABC = -nABC;
        if (glm::dot(nACD, ab) > 0.0) nACD = -nACD;
        if (glm::dot(nADB, ac) > 0.0) nADB = -nADB;

        if (glm::dot(nABC, ao) > 0.0)
        {
            simplex.vertices[0] = C;
            simplex.vertices[1] = B;
            simplex.vertices[2] = A;
            simplex.count = 3;
            return SolveTriangle(simplex, searchDirection, outClosestPoint, outClosestA, outClosestB);
        }

        if (glm::dot(nACD, ao) > 0.0)
        {
            simplex.vertices[0] = D;
            simplex.vertices[1] = C;
            simplex.vertices[2] = A;
            simplex.count = 3;
            return SolveTriangle(simplex, searchDirection, outClosestPoint, outClosestA, outClosestB);
        }

        if (glm::dot(nADB, ao) > 0.0)
        {
            simplex.vertices[0] = B;
            simplex.vertices[1] = D;
            simplex.vertices[2] = A;
            simplex.count = 3;
            return SolveTriangle(simplex, searchDirection, outClosestPoint, outClosestA, outClosestB);
        }

        // Origin is fully enclosed within the tetrahedron interior; cores intersect!
        outClosestPoint = Vec3(0.0);
        outClosestA = 0.25 * (A.pA + B.pA + C.pA + D.pA);
        outClosestB = 0.25 * (A.pB + B.pB + C.pB + D.pB);
        return true;
    }

    bool GJK::SolveSimplex(Simplex& simplex, Vec3& searchDirection, Vec3& outClosestPoint,
        Vec3& outClosestA, Vec3& outClosestB)
    {
        switch (simplex.count)
        {
        case 1:
            return SolvePoint(simplex, searchDirection, outClosestPoint, outClosestA, outClosestB);
        case 2:
            return SolveSegment(simplex, searchDirection, outClosestPoint, outClosestA, outClosestB);
        case 3:
            return SolveTriangle(simplex, searchDirection, outClosestPoint, outClosestA, outClosestB);
        case 4:
            return SolveTetrahedron(simplex, searchDirection, outClosestPoint, outClosestA, outClosestB);
        }
        return false;
    }

    GjkResult GJK::ComputeDistance(const ConvexCore& coreA, const ConvexCore& coreB, int maxIterations)
    {
        GjkResult result;
        Simplex simplex;

        // Initial search direction: delta between core centers
        Vec3 d = coreB.position - coreA.position;
        if (glm::dot(d, d) < kGjkEpsilonSq)
        {
            d = Vec3(0.0, 1.0, 0.0);
        }

        // Seed simplex with first extremal Minkowski difference point
        SimplexVertex initialVertex;
        initialVertex.pA = Support::ComputeSupport(coreA, -d);
        initialVertex.pB = Support::ComputeSupport(coreB, d);
        initialVertex.v = initialVertex.pA - initialVertex.pB;

        simplex.vertices[0] = initialVertex;
        simplex.count = 1;

        Vec3 closestPoint = initialVertex.v;
        Vec3 closestA = initialVertex.pA;
        Vec3 closestB = initialVertex.pB;
        Vec3 searchDirection = -closestPoint;

        Real prevDistSq = std::numeric_limits<Real>::max();

        for (int iter = 0; iter < maxIterations; ++iter)
        {
            result.iterationsExecuted++;

            Real distSq = glm::dot(closestPoint, closestPoint);
            if (distSq < kGjkEpsilonSq)
            {
                // Core shapes intersect
                result.areDisjoint = false;
                result.distance = -(coreA.coreRadius + coreB.coreRadius);
                result.closestPointA = closestA;
                result.closestPointB = closestB;
                result.normal = Vec3(0.0, 1.0, 0.0);
                return result;
            }

            // Normalization guard
            if (glm::dot(searchDirection, searchDirection) < kGjkEpsilonSq)
            {
                searchDirection = -closestPoint;
            }
            Vec3 dirNorm = glm::normalize(searchDirection);

            // Fetch new extremal vertex along search direction
            SimplexVertex newVertex;
            newVertex.pA = Support::ComputeSupport(coreA, dirNorm);
            newVertex.pB = Support::ComputeSupport(coreB, -dirNorm);
            newVertex.v = newVertex.pA - newVertex.pB;

            // Convergence test: new vertex must project further in search direction than closest point
            Real progress = glm::dot(newVertex.v - closestPoint, dirNorm);
            if (progress <= kGjkEpsilon || distSq >= prevDistSq)
            {
                break;
            }
            prevDistSq = distSq;

            // Shift vertices and insert new vertex at top
            simplex.vertices[simplex.count] = newVertex;
            simplex.count++;

            bool originEnclosed = SolveSimplex(simplex, searchDirection, closestPoint, closestA, closestB);
            if (originEnclosed)
            {
                result.areDisjoint = false;
                result.distance = -(coreA.coreRadius + coreB.coreRadius);
                result.closestPointA = closestA;
                result.closestPointB = closestB;
                result.normal = Vec3(0.0, 1.0, 0.0);
                return result;
            }
        }

        Real coreDistance = glm::length(closestPoint);
        Real netDistance = coreDistance - (coreA.coreRadius + coreB.coreRadius);

        // ClosestPoint on (A - B) points from B to A; negate so normal points from A to B
        Vec3 normal = (coreDistance > kGjkEpsilon) ? (-closestPoint / coreDistance) : Vec3(0.0, 1.0, 0.0);

        // Account for spherical expansion radii on witness points
        result.areDisjoint = (netDistance > 0.0);
        result.distance = netDistance;
        result.normal = normal;
        result.closestPointA = closestA + normal * coreA.coreRadius;
        result.closestPointB = closestB - normal * coreB.coreRadius;

        return result;
    }
}