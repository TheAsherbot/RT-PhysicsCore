/**
 * @file TelemetryMacros.h
 * @brief Preprocessor macros for zero-overhead performance instrumentation across build configurations.
 */

#pragma once

#include "RT-PhysicsCore/telemetry/TelemetryTimer.h"

#if defined(_DEBUG) || defined(RT_FORCE_PROFILING)
#define RT_ENABLE_PROFILING 1
#else
#define RT_ENABLE_PROFILING 0
#endif

#if RT_ENABLE_PROFILING
 /**
  * @def RT_PROFILE_SCOPE(name)
  * @brief Measures execution duration for the enclosing code block under a custom label.
  */
#define RT_PROFILE_SCOPE(name) ::RT_PhysicsCore::TelemetryTimer timer##__LINE__(name)

  /**
   * @def RT_PROFILE_FUNCTION()
   * @brief Measures execution duration for the enclosing function using compiler function metadata.
   */
#define RT_PROFILE_FUNCTION() RT_PROFILE_SCOPE(__FUNCTION__)
#else
#define RT_PROFILE_SCOPE(name)
#define RT_PROFILE_FUNCTION()
#endif