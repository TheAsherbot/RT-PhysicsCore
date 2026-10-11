/**
 * @file MathTypes.cpp
 * @brief Implementation of vector/quaternion operations and rendering boundary conversion routines.
 */

#include "RT-PhysicsCore/core/math/MathTypes.h"
#include <cassert>
#include <algorithm>
#include <cmath>

namespace RT_PhysicsCore
{
    Vec3::Vec3()
        : x(0.0), y(0.0), z(0.0)
    {}

    Vec3::Vec3(Real scalar)
        : x(scalar), y(scalar), z(scalar)
    {}

    Vec3::Vec3(Real x, Real y, Real z)
        : x(x), y(y), z(z)
    {}

    Vec3::Vec3(const glm::vec3& v)
        : x(static_cast<Real>(v.x)), y(static_cast<Real>(v.y)), z(static_cast<Real>(v.z))
    {}

    Vec3::Vec3(const glm::dvec3& v)
        : x(v.x), y(v.y), z(v.z)
    {}

    Vec3::operator glm::vec3() const
    {
        return glm::vec3(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z));
    }

    Vec3::operator glm::dvec3() const
    {
        return glm::dvec3(x, y, z);
    }

    Real& Vec3::operator[](int index)
    {
        assert(index >= 0 && index < 3);
        return (&x)[index];
    }

    const Real& Vec3::operator[](int index) const
    {
        assert(index >= 0 && index < 3);
        return (&x)[index];
    }

    Vec3 Vec3::operator+(const Vec3& other) const
    {
        return Vec3(x + other.x, y + other.y, z + other.z);
    }

    Vec3 Vec3::operator-(const Vec3& other) const
    {
        return Vec3(x - other.x, y - other.y, z - other.z);
    }

    Vec3 Vec3::operator*(Real scalar) const
    {
        return Vec3(x * scalar, y * scalar, z * scalar);
    }

    Vec3 Vec3::operator/(Real scalar) const
    {
        Real inv = 1.0 / scalar;
        return Vec3(x * inv, y * inv, z * inv);
    }

    Vec3 Vec3::operator*(const Vec3& other) const
    {
        return Vec3(x * other.x, y * other.y, z * other.z);
    }

    Vec3 Vec3::operator/(const Vec3& other) const
    {
        return Vec3(x / other.x, y / other.y, z / other.z);
    }

    Vec3 Vec3::operator-() const
    {
        return Vec3(-x, -y, -z);
    }

    Vec3& Vec3::operator+=(const Vec3& other)
    {
        x += other.x;
        y += other.y;
        z += other.z;
        return *this;
    }

    Vec3& Vec3::operator-=(const Vec3& other)
    {
        x -= other.x;
        y -= other.y;
        z -= other.z;
        return *this;
    }

    Vec3& Vec3::operator*=(Real scalar)
    {
        x *= scalar;
        y *= scalar;
        z *= scalar;
        return *this;
    }

    Vec3& Vec3::operator/=(Real scalar)
    {
        Real inv = 1.0 / scalar;
        x *= inv;
        y *= inv;
        z *= inv;
        return *this;
    }

    Vec3& Vec3::operator*=(const Vec3& other)
    {
        x *= other.x;
        y *= other.y;
        z *= other.z;
        return *this;
    }

    Vec3& Vec3::operator/=(const Vec3& other)
    {
        x /= other.x;
        y /= other.y;
        z /= other.z;
        return *this;
    }

    bool Vec3::operator==(const Vec3& other) const
    {
        return x == other.x && y == other.y && z == other.z;
    }

    bool Vec3::operator!=(const Vec3& other) const
    {
        return !(*this == other);
    }

    Vec3 operator*(Real scalar, const Vec3& v)
    {
        return Vec3(scalar * v.x, scalar * v.y, scalar * v.z);
    }

    Vec3 operator*(const Mat3& m, const Vec3& v)
    {
        glm::dvec3 res = m * static_cast<glm::dvec3>(v);
        return Vec3(res.x, res.y, res.z);
    }

    Vec3 operator*(const Mat4& m, const Vec3& v)
    {
        glm::dvec4 res = m * glm::dvec4(v.x, v.y, v.z, 1.0);
        return Vec3(res.x, res.y, res.z);
    }

    Vec3 operator*(const glm::mat3& m, const Vec3& v)
    {
        glm::vec3 res = m * static_cast<glm::vec3>(v);
        return Vec3(res.x, res.y, res.z);
    }

    Vec3 operator*(const glm::mat4& m, const Vec3& v)
    {
        glm::vec4 res = m * glm::vec4(static_cast<float>(v.x), static_cast<float>(v.y), static_cast<float>(v.z), 1.0f);
        return Vec3(res.x, res.y, res.z);
    }

    Quat::Quat()
        : w(1.0), x(0.0), y(0.0), z(0.0)
    {}

    Quat::Quat(Real w, Real x, Real y, Real z)
        : w(w), x(x), y(y), z(z)
    {}

    Quat::Quat(const glm::quat& q)
        : w(static_cast<Real>(q.w)), x(static_cast<Real>(q.x)), y(static_cast<Real>(q.y)), z(static_cast<Real>(q.z))
    {}

    Quat::Quat(const glm::dquat& q)
        : w(q.w), x(q.x), y(q.y), z(q.z)
    {}

    Quat::operator glm::quat() const
    {
        return glm::quat(static_cast<float>(w), static_cast<float>(x), static_cast<float>(y), static_cast<float>(z));
    }

    Quat::operator glm::dquat() const
    {
        return glm::dquat(w, x, y, z);
    }

    Quat Quat::operator*(const Quat& other) const
    {
        glm::dquat dq1(w, x, y, z);
        glm::dquat dq2(other.w, other.x, other.y, other.z);
        glm::dquat res = dq1 * dq2;
        return Quat(res.w, res.x, res.y, res.z);
    }

    Vec3 Quat::operator*(const Vec3& v) const
    {
        glm::dquat dq(w, x, y, z);
        glm::dvec3 dv(v.x, v.y, v.z);
        glm::dvec3 res = dq * dv;
        return Vec3(res.x, res.y, res.z);
    }

    bool Quat::operator==(const Quat& other) const
    {
        return w == other.w && x == other.x && y == other.y && z == other.z;
    }

    bool Quat::operator!=(const Quat& other) const
    {
        return !(*this == other);
    }

    glm::vec3 ToFloatVec3(const Vec3& v)
    {
        return static_cast<glm::vec3>(v);
    }

    glm::quat ToFloatQuat(const Quat& q)
    {
        return static_cast<glm::quat>(q);
    }
}

namespace glm
{
    mat4 translate(const mat4& m, const RT_PhysicsCore::Vec3& v)
    {
        return translate(m, static_cast<vec3>(v));
    }

    dmat4 translate(const dmat4& m, const RT_PhysicsCore::Vec3& v)
    {
        return translate(m, static_cast<dvec3>(v));
    }

    mat4 scale(const mat4& m, const RT_PhysicsCore::Vec3& v)
    {
        return scale(m, static_cast<vec3>(v));
    }

    dmat4 scale(const dmat4& m, const RT_PhysicsCore::Vec3& v)
    {
        return scale(m, static_cast<dvec3>(v));
    }

    RT_PhysicsCore::Real dot(const RT_PhysicsCore::Vec3& a, const RT_PhysicsCore::Vec3& b)
    {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }

    RT_PhysicsCore::Vec3 cross(const RT_PhysicsCore::Vec3& a, const RT_PhysicsCore::Vec3& b)
    {
        return RT_PhysicsCore::Vec3(
            a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x
        );
    }

    RT_PhysicsCore::Real length(const RT_PhysicsCore::Vec3& v)
    {
        return std::sqrt(dot(v, v));
    }

    RT_PhysicsCore::Real length2(const RT_PhysicsCore::Vec3& v)
    {
        return dot(v, v);
    }

    RT_PhysicsCore::Vec3 normalize(const RT_PhysicsCore::Vec3& v)
    {
        RT_PhysicsCore::Real len = length(v);
        if (len > 1e-12)
        {
            return v / len;
        }
        return RT_PhysicsCore::Vec3(0.0);
    }

    RT_PhysicsCore::Real distance(const RT_PhysicsCore::Vec3& a, const RT_PhysicsCore::Vec3& b)
    {
        return length(a - b);
    }

    RT_PhysicsCore::Vec3 min(const RT_PhysicsCore::Vec3& a, const RT_PhysicsCore::Vec3& b)
    {
        return RT_PhysicsCore::Vec3(
            std::min(a.x, b.x),
            std::min(a.y, b.y),
            std::min(a.z, b.z)
        );
    }

    RT_PhysicsCore::Vec3 max(const RT_PhysicsCore::Vec3& a, const RT_PhysicsCore::Vec3& b)
    {
        return RT_PhysicsCore::Vec3(
            std::max(a.x, b.x),
            std::max(a.y, b.y),
            std::max(a.z, b.z)
        );
    }

    RT_PhysicsCore::Vec3 abs(const RT_PhysicsCore::Vec3& v)
    {
        return RT_PhysicsCore::Vec3(std::abs(v.x), std::abs(v.y), std::abs(v.z));
    }

    RT_PhysicsCore::Vec3 clamp(const RT_PhysicsCore::Vec3& v, const RT_PhysicsCore::Vec3& minVal, const RT_PhysicsCore::Vec3& maxVal)
    {
        return RT_PhysicsCore::Vec3(
            std::clamp(v.x, minVal.x, maxVal.x),
            std::clamp(v.y, minVal.y, maxVal.y),
            std::clamp(v.z, minVal.z, maxVal.z)
        );
    }

    mat4 mat4_cast(const RT_PhysicsCore::Quat& q)
    {
        return mat4_cast(static_cast<quat>(q));
    }

    mat3 mat3_cast(const RT_PhysicsCore::Quat& q)
    {
        return mat3_cast(static_cast<dquat>(q));
    }

    RT_PhysicsCore::Quat normalize(const RT_PhysicsCore::Quat& q)
    {
        glm::dquat dq = normalize(static_cast<dquat>(q));
        return RT_PhysicsCore::Quat(dq.w, dq.x, dq.y, dq.z);
    }

    RT_PhysicsCore::Quat conjugate(const RT_PhysicsCore::Quat& q)
    {
        glm::dquat dq = conjugate(static_cast<dquat>(q));
        return RT_PhysicsCore::Quat(dq.w, dq.x, dq.y, dq.z);
    }

    RT_PhysicsCore::Quat inverse(const RT_PhysicsCore::Quat& q)
    {
        glm::dquat dq = inverse(static_cast<dquat>(q));
        return RT_PhysicsCore::Quat(dq.w, dq.x, dq.y, dq.z);
    }

    RT_PhysicsCore::Real dot(const RT_PhysicsCore::Quat& a, const RT_PhysicsCore::Quat& b)
    {
        return dot(static_cast<dquat>(a), static_cast<dquat>(b));
    }

    RT_PhysicsCore::Quat angleAxis(RT_PhysicsCore::Real angle, const RT_PhysicsCore::Vec3& axis)
    {
        glm::dquat dq = glm::angleAxis(angle, static_cast<glm::dvec3>(axis));
        return RT_PhysicsCore::Quat(dq.w, dq.x, dq.y, dq.z);
    }
}