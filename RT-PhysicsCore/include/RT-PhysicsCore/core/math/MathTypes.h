/**
 * @file MathTypes.h
 * @brief High-precision numerical types and vector conversion utilities.
 */

#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/matrix_transform.hpp>

namespace RT_PhysicsCore
{
    /**
     * @brief High-precision scalar type for simulation mechanics.
     */
    using Real = double;

    /**
     * @brief 2-component high-precision vector.
     */
    using Vec2 = glm::dvec2;

    struct Quat;

    /**
     * @struct Vec3
     * @brief High-precision 3-component vector with seamless GLM interoperability.
     */
    struct Vec3
    {
        Real x{ 0.0 }; ///< X component coordinate.
        Real y{ 0.0 }; ///< Y component coordinate.
        Real z{ 0.0 }; ///< Z component coordinate.

        /**
         * @brief Default constructor initializing all components to zero.
         */
        Vec3();

        /**
         * @brief Uniform scalar constructor filling all components.
         * @param scalar Value assigned to X, Y, and Z.
         */
        Vec3(Real scalar);

        /**
         * @brief Component-wise constructor.
         * @param x X component value.
         * @param y Y component value.
         * @param z Z component value.
         */
        Vec3(Real x, Real y, Real z);

        /**
         * @brief Implicit converting constructor from single-precision GLM vector.
         * @param v Single-precision 3D vector.
         */
        Vec3(const glm::vec3& v);

        /**
         * @brief Implicit converting constructor from double-precision GLM vector.
         * @param v Double-precision 3D vector.
         */
        Vec3(const glm::dvec3& v);

        /**
         * @brief Implicit conversion operator to single-precision GLM vector.
         * @return Single-precision glm::vec3 representation.
         */
        operator glm::vec3() const;

        /**
         * @brief Implicit conversion operator to double-precision GLM vector.
         * @return Double-precision glm::dvec3 representation.
         */
        operator glm::dvec3() const;

        /**
         * @brief Subscript access operator by coordinate index.
         * @param index Coordinate index (0 = x, 1 = y, 2 = z).
         * @return Reference to component scalar.
         */
        Real& operator[](int index);

        /**
         * @brief Const subscript access operator by coordinate index.
         * @param index Coordinate index (0 = x, 1 = y, 2 = z).
         * @return Const reference to component scalar.
         */
        const Real& operator[](int index) const;

        /**
         * @brief Vector addition operator.
         * @param other Vector to add.
         * @return Component-wise sum.
         */
        Vec3 operator+(const Vec3& other) const;

        /**
         * @brief Vector subtraction operator.
         * @param other Vector to subtract.
         * @return Component-wise difference.
         */
        Vec3 operator-(const Vec3& other) const;

        /**
         * @brief Scalar multiplication operator.
         * @param scalar Multiplier factor.
         * @return Scaled vector.
         */
        Vec3 operator*(Real scalar) const;

        /**
         * @brief Scalar division operator.
         * @param scalar Divisor factor.
         * @return Scaled vector.
         */
        Vec3 operator/(Real scalar) const;

        /**
         * @brief Component-wise vector multiplication operator.
         * @param other Vector multiplier.
         * @return Component-wise product.
         */
        Vec3 operator*(const Vec3& other) const;

        /**
         * @brief Component-wise vector division operator.
         * @param other Vector divisor.
         * @return Component-wise quotient.
         */
        Vec3 operator/(const Vec3& other) const;

        /**
         * @brief Unary negation operator.
         * @return Negated vector.
         */
        Vec3 operator-() const;

        /**
         * @brief Compound addition assignment operator.
         * @param other Vector to add.
         * @return Reference to this vector.
         */
        Vec3& operator+=(const Vec3& other);

        /**
         * @brief Compound subtraction assignment operator.
         * @param other Vector to subtract.
         * @return Reference to this vector.
         */
        Vec3& operator-=(const Vec3& other);

        /**
         * @brief Compound scalar multiplication assignment operator.
         * @param scalar Multiplier factor.
         * @return Reference to this vector.
         */
        Vec3& operator*=(Real scalar);

        /**
         * @brief Compound scalar division assignment operator.
         * @param scalar Divisor factor.
         * @return Reference to this vector.
         */
        Vec3& operator/=(Real scalar);

        /**
         * @brief Compound component-wise vector multiplication assignment operator.
         * @param other Vector multiplier.
         * @return Reference to this vector.
         */
        Vec3& operator*=(const Vec3& other);

        /**
         * @brief Compound component-wise vector division assignment operator.
         * @param other Vector divisor.
         * @return Reference to this vector.
         */
        Vec3& operator/=(const Vec3& other);

        /**
         * @brief Equality comparison operator.
         * @param other Vector to compare.
         * @return True if components are identical.
         */
        bool operator==(const Vec3& other) const;

        /**
         * @brief Inequality comparison operator.
         * @param other Vector to compare.
         * @return True if any component differs.
         */
        bool operator!=(const Vec3& other) const;
    };

    /**
     * @brief Left scalar multiplication operator.
     * @param scalar Multiplier factor.
     * @param v Vector to scale.
     * @return Scaled vector.
     */
    Vec3 operator*(Real scalar, const Vec3& v);

    /**
     * @brief 4-component high-precision vector.
     */
    using Vec4 = glm::dvec4;

    /**
     * @brief 3x3 high-precision matrix.
     */
    using Mat3 = glm::dmat3;

    /**
     * @brief 4x4 high-precision matrix.
     */
    using Mat4 = glm::dmat4;

    /**
     * @brief Matrix-vector multiplication operator for 3x3 double-precision matrices.
     * @param m 3x3 matrix.
     * @param v 3D vector.
     * @return Transformed 3D vector.
     */
    Vec3 operator*(const Mat3& m, const Vec3& v);

    /**
     * @brief Matrix-vector multiplication operator for 4x4 double-precision matrices.
     * @param m 4x4 matrix.
     * @param v 3D vector (w assumed 1.0).
     * @return Transformed 3D vector.
     */
    Vec3 operator*(const Mat4& m, const Vec3& v);

    /**
     * @brief Matrix-vector multiplication operator for 3x3 single-precision matrices.
     * @param m 3x3 single-precision matrix.
     * @param v 3D vector.
     * @return Transformed 3D vector.
     */
    Vec3 operator*(const glm::mat3& m, const Vec3& v);

    /**
     * @brief Matrix-vector multiplication operator for 4x4 single-precision matrices.
     * @param m 4x4 single-precision matrix.
     * @param v 3D vector.
     * @return Transformed 3D vector.
     */
    Vec3 operator*(const glm::mat4& m, const Vec3& v);

    /**
     * @struct Quat
     * @brief High-precision quaternion with seamless GLM interoperability.
     */
    struct Quat
    {
        Real w{ 1.0 }; ///< Real scalar component.
        Real x{ 0.0 }; ///< Imaginary vector X component.
        Real y{ 0.0 }; ///< Imaginary vector Y component.
        Real z{ 0.0 }; ///< Imaginary vector Z component.

        /**
         * @brief Default constructor initializing identity orientation (1, 0, 0, 0).
         */
        Quat();

        /**
         * @brief Component-wise constructor.
         * @param w Real scalar component.
         * @param x Imaginary X component.
         * @param y Imaginary Y component.
         * @param z Imaginary Z component.
         */
        Quat(Real w, Real x, Real y, Real z);

        /**
         * @brief Implicit converting constructor from single-precision GLM quaternion.
         * @param q Single-precision quaternion.
         */
        Quat(const glm::quat& q);

        /**
         * @brief Implicit converting constructor from double-precision GLM quaternion.
         * @param q Double-precision quaternion.
         */
        Quat(const glm::dquat& q);

        /**
         * @brief Implicit conversion operator to single-precision GLM quaternion.
         * @return Single-precision glm::quat representation.
         */
        operator glm::quat() const;

        /**
         * @brief Implicit conversion operator to double-precision GLM quaternion.
         * @return Double-precision glm::dquat representation.
         */
        operator glm::dquat() const;

        /**
         * @brief Quaternion composition multiplication operator.
         * @param other Second rotation to compose.
         * @return Composed rotation quaternion.
         */
        Quat operator*(const Quat& other) const;

        /**
         * @brief Vector rotation operator applying this quaternion orientation to a 3D vector.
         * @param v Vector to rotate.
         * @return Rotated vector.
         */
        Vec3 operator*(const Vec3& v) const;

        /**
         * @brief Equality comparison operator.
         * @param other Quaternion to compare.
         * @return True if components are identical.
         */
        bool operator==(const Quat& other) const;

        /**
         * @brief Inequality comparison operator.
         * @param other Quaternion to compare.
         * @return True if any component differs.
         */
        bool operator!=(const Quat& other) const;
    };

    /**
     * @brief Converts a high-precision 3D vector to single precision for rendering pipelines.
     * @param v High-precision vector.
     * @return Single-precision 3D vector.
     */
    glm::vec3 ToFloatVec3(const Vec3& v);

    /**
     * @brief Converts a high-precision quaternion to single precision for rendering pipelines.
     * @param q High-precision quaternion.
     * @return Single-precision quaternion.
     */
    glm::quat ToFloatQuat(const Quat& q);
}

namespace glm
{
    /**
     * @brief Translation transformation overload for single-precision 4x4 matrix and high-precision Vec3.
     * @param m Base transformation matrix.
     * @param v Translation vector.
     * @return Translated single-precision matrix.
     */
    mat4 translate(const mat4& m, const RT_PhysicsCore::Vec3& v);

    /**
     * @brief Translation transformation overload for double-precision 4x4 matrix and high-precision Vec3.
     * @param m Base transformation matrix.
     * @param v Translation vector.
     * @return Translated double-precision matrix.
     */
    dmat4 translate(const dmat4& m, const RT_PhysicsCore::Vec3& v);

    /**
     * @brief Scale transformation overload for single-precision 4x4 matrix and high-precision Vec3.
     * @param m Base transformation matrix.
     * @param v Scale vector.
     * @return Scaled single-precision matrix.
     */
    mat4 scale(const mat4& m, const RT_PhysicsCore::Vec3& v);

    /**
     * @brief Scale transformation overload for double-precision 4x4 matrix and high-precision Vec3.
     * @param m Base transformation matrix.
     * @param v Scale vector.
     * @return Scaled double-precision matrix.
     */
    dmat4 scale(const dmat4& m, const RT_PhysicsCore::Vec3& v);

    /**
     * @brief Dot product of two high-precision 3D vectors.
     * @param a First vector.
     * @param b Second vector.
     * @return Scalar dot product.
     */
    RT_PhysicsCore::Real dot(const RT_PhysicsCore::Vec3& a, const RT_PhysicsCore::Vec3& b);

    /**
     * @brief Cross product of two high-precision 3D vectors.
     * @param a First vector.
     * @param b Second vector.
     * @return Vector cross product.
     */
    RT_PhysicsCore::Vec3 cross(const RT_PhysicsCore::Vec3& a, const RT_PhysicsCore::Vec3& b);

    /**
     * @brief Euclidean length of a high-precision 3D vector.
     * @param v Input vector.
     * @return Scalar length.
     */
    RT_PhysicsCore::Real length(const RT_PhysicsCore::Vec3& v);

    /**
     * @brief Squared Euclidean length of a high-precision 3D vector.
     * @param v Input vector.
     * @return Scalar squared length.
     */
    RT_PhysicsCore::Real length2(const RT_PhysicsCore::Vec3& v);

    /**
     * @brief Normalized unit vector.
     * @param v Input vector.
     * @return Unit-length vector.
     */
    RT_PhysicsCore::Vec3 normalize(const RT_PhysicsCore::Vec3& v);

    /**
     * @brief Euclidean distance between two high-precision 3D vectors.
     * @param a First point.
     * @param b Second point.
     * @return Distance scalar.
     */
    RT_PhysicsCore::Real distance(const RT_PhysicsCore::Vec3& a, const RT_PhysicsCore::Vec3& b);

    /**
     * @brief Component-wise minimum of two high-precision 3D vectors.
     * @param a First vector.
     * @param b Second vector.
     * @return Component-wise minimum vector.
     */
    RT_PhysicsCore::Vec3 min(const RT_PhysicsCore::Vec3& a, const RT_PhysicsCore::Vec3& b);

    /**
     * @brief Component-wise maximum of two high-precision 3D vectors.
     * @param a First vector.
     * @param b Second vector.
     * @return Component-wise maximum vector.
     */
    RT_PhysicsCore::Vec3 max(const RT_PhysicsCore::Vec3& a, const RT_PhysicsCore::Vec3& b);

    /**
     * @brief Component-wise absolute value of a high-precision 3D vector.
     * @param v Input vector.
     * @return Absolute value vector.
     */
    RT_PhysicsCore::Vec3 abs(const RT_PhysicsCore::Vec3& v);

    /**
     * @brief Component-wise clamp of a high-precision 3D vector within [minVal, maxVal].
     * @param v Input vector.
     * @param minVal Minimum boundary vector.
     * @param maxVal Maximum boundary vector.
     * @return Clamped vector.
     */
    RT_PhysicsCore::Vec3 clamp(const RT_PhysicsCore::Vec3& v, const RT_PhysicsCore::Vec3& minVal, const RT_PhysicsCore::Vec3& maxVal);

    /**
     * @brief Matrix conversion casting Quat to single-precision 4x4 matrix for graphics rendering.
     * @param q Orientation quaternion.
     * @return Single-precision 4x4 rotation matrix.
     */
    mat4 mat4_cast(const RT_PhysicsCore::Quat& q);

    /**
     * @brief Matrix conversion casting Quat to double-precision 3x3 matrix for physics mechanics.
     * @param q Orientation quaternion.
     * @return Double-precision 3x3 rotation matrix.
     */
    mat3 mat3_cast(const RT_PhysicsCore::Quat& q);

    /**
     * @brief Normalizes a high-precision orientation quaternion.
     * @param q Quaternion to normalize.
     * @return Unit-length quaternion.
     */
    RT_PhysicsCore::Quat normalize(const RT_PhysicsCore::Quat& q);

    /**
     * @brief Conjugate of an orientation quaternion.
     * @param q Input quaternion.
     * @return Conjugated quaternion (w, -x, -y, -z).
     */
    RT_PhysicsCore::Quat conjugate(const RT_PhysicsCore::Quat& q);

    /**
     * @brief Inverse of an orientation quaternion.
     * @param q Input quaternion.
     * @return Inverted quaternion.
     */
    RT_PhysicsCore::Quat inverse(const RT_PhysicsCore::Quat& q);

    /**
     * @brief Dot product of two quaternions.
     * @param a First quaternion.
     * @param b Second quaternion.
     * @return Scalar dot product.
     */
    RT_PhysicsCore::Real dot(const RT_PhysicsCore::Quat& a, const RT_PhysicsCore::Quat& b);

    /**
     * @brief Constructs an orientation quaternion from axis-angle representation.
     * @param angle Rotation angle in radians.
     * @param axis Unit rotation axis vector.
     * @return Orientation quaternion.
     */
    RT_PhysicsCore::Quat angleAxis(RT_PhysicsCore::Real angle, const RT_PhysicsCore::Vec3& axis);
}