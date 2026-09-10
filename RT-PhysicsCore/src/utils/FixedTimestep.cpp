#include "FixedTimestep.h"
#include <chrono>

namespace RT_PhysicsCore
{
    static double NowSeconds()
    {
        using clock = std::chrono::steady_clock;
        using sec = std::chrono::duration<double>;
        return sec(clock::now().time_since_epoch()).count();
    }

    FixedTimestep::FixedTimestep(double hz)
        : fixedDelta(1.0 / hz),
        accumulator(0.0),
        lastTime(NowSeconds()),
        frameDelta(0.0f)
    {}

    void FixedTimestep::SetFrequency(double hz)
    {
        fixedDelta = 1.0 / hz;
    }

    double FixedTimestep::FixedDeltaSeconds() const
    {
        return fixedDelta;
    }

    std::uint32_t FixedTimestep::Step()
    {
        double now = NowSeconds();
        frameDelta = now - lastTime;
        lastTime = now;

        // Clamp to avoid spiral of death
        double maxFrameDelta = fixedDelta * 4.0;
        if (frameDelta > maxFrameDelta)
            frameDelta = maxFrameDelta;

        accumulator += frameDelta;

        std::uint32_t steps = 0;
        while (accumulator >= fixedDelta)
        {
            accumulator -= fixedDelta;
            ++steps;
        }

        return steps;
    }

    double FixedTimestep::Alpha() const
    {
        return accumulator / fixedDelta;
    }

	double FixedTimestep::DeltaSeconds() const
	{
		return frameDelta;
	}
}
