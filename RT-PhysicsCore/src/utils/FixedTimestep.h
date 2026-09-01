#pragma once

#include <cstdint>

namespace RT_PhysicsCore
{
    class FixedTimestep
    {
    public:
        explicit FixedTimestep(double hz = 60.0);

        void SetFrequency(double hz);
        double FixedDeltaSeconds() const;

        // Called once per frame; returns number of fixed updates to run.
        std::uint32_t Step();

        // Interpolation alpha (0..1)
        double Alpha() const;
        double DeltaSeconds() const;

    private:
        double fixedDelta;     // seconds per fixed step
        double accumulator;    // leftover time
        double lastTime;       // last timestamp in seconds
		double frameDelta;     // time since last frame in seconds
    };
}
