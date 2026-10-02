/**
 * @file TelemetryOverlay.h
 * @brief Lightweight on-screen heads-up display rendering runtime performance metrics.
 */

#pragma once

namespace RT_PhysicsCore
{
    /**
     * @class TelemetryOverlay
     * @brief Renders the Mode 2 performance HUD overlay using Dear ImGui.
     */
    class TelemetryOverlay
    {
    public:
        /**
         * @brief Default constructor.
         */
        TelemetryOverlay();

        /**
         * @brief Destructor.
         */
        ~TelemetryOverlay();

        TelemetryOverlay(const TelemetryOverlay&) = delete;
        TelemetryOverlay& operator=(const TelemetryOverlay&) = delete;

        /**
         * @brief Renders the overlay window in the viewport when Mode 2 is active.
         */
        void Render();
    };
}