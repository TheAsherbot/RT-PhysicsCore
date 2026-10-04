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
        Default,     ///< Default generic material (e=0.3, mu=0.6).
        Clay,        ///< Heavy, no bounce (e=0.0).
        Wood,        ///< Slight bounce (e=0.25).
        Rubber,      ///< Bouncy (e=0.5).
        HardRubber,  ///< Very bouncy (e=0.75).
        SuperBall,   ///< Perfect bounce (e=1.0).
        BouncyIce,   ///< Perfect bounce, zero friction (e=1.0, mu=0.0) for Newton's Cradle.
        Domino       ///< High friction, low bounce for stable chaining.
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