#pragma once

namespace RT_PhysicsCore
{
    // Extend this as real materials are needed - each new entry needs a
    // solo-properties row in PhysicsMaterial.cpp, and optionally explicit
    // pair entries there for combinations that matter enough to hand-tune.
    enum class MaterialId
    {
        Default,
    };

    struct MaterialProperties
    {
        float restitution;
        float staticFriction;
        float kineticFriction;
    };

    // A single material's own properties (used as the basis for combining
    // two different materials that have no explicit pair entry).
    MaterialProperties GetMaterialProperties(MaterialId id);

    struct MaterialPairProperties
    {
        float restitution;
        float staticFriction;
        float kineticFriction;
    };

    // Combined properties for two materials in contact (order doesn't
    // matter). Checks an explicit pair table first; falls back to
    // combining each material's solo properties - geometric mean for
    // friction (the more standard choice), arithmetic mean for
    // restitution. No combination rule is truly physically exact for a
    // real material pair - these are reasonable, deliberately-chosen
    // defaults, not derived results. Add pairs to the explicit table in
    // PhysicsMaterial.cpp for any combination that needs real values
    // instead.
    MaterialPairProperties GetPairProperties(MaterialId a, MaterialId b);
}
