/**
 * @file PhysicsMaterial.h
 * @brief Surface material properties and friction/restitution combination rules.
 *
 * Defines solo material coefficients and computes paired contact interaction parameters
 * via an explicit lookup table or geometric/arithmetic mean mixing models.
 */

#pragma once

namespace RT_PhysicsCore
{
    /**
     * @enum MaterialId
     * @brief Identifier for registered physics surface materials.
     */
    enum class MaterialId
    {
        Default, ///< Default generic material.
    };

    /**
     * @struct MaterialProperties
     * @brief Intrinsic physical coefficients of a single isolated material.
     */
    struct MaterialProperties
    {
        float restitution;      ///< Coefficient of restitution (bounciness) in range [0.0, 1.0].
        float staticFriction;   ///< Coefficient of static friction (grip before sliding).
        float kineticFriction;  ///< Coefficient of kinetic friction (resistance while sliding).
    };

    /**
     * @brief Retrieves the intrinsic physical properties defined for a single material.
     * @param id The material identifier.
     * @return MaterialProperties struct containing restitution and friction values.
     */
    MaterialProperties GetMaterialProperties(MaterialId id);

    /**
     * @struct MaterialPairProperties
     * @brief Effective combined interaction coefficients between two contacting surfaces.
     */
    struct MaterialPairProperties
    {
        float restitution;      ///< Combined restitution coefficient.
        float staticFriction;   ///< Combined static friction coefficient.
        float kineticFriction;  ///< Combined kinetic friction coefficient.
    };

    /**
     * @brief Computes interaction properties for a contact pair between two materials.
     *
     * Checks an explicit override table first. If no explicit entry exists, falls back
     * to heuristic mixing: arithmetic mean for restitution, geometric mean for friction.
     *
     * @param a First material identifier (order is symmetric).
     * @param b Second material identifier.
     * @return Effective combined MaterialPairProperties.
     */
    MaterialPairProperties GetPairProperties(MaterialId a, MaterialId b);
}