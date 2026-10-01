/**
 * @file ConsoleInput.h
 * @brief Non-blocking terminal keyboard polling utility.
 *
 * Provides a cross-platform equivalent to `<conio.h>` for non-blocking stdin
 * interrogation without waiting for newline submission.
 */

#pragma once

namespace RT_PhysicsCore
{
    /**
     * @brief Checks stdin for a pending keypress and consumes it if waiting.
     *
     * Cross-platform replacement for Windows `_kbhit()` and `_getch()`.
     * On POSIX systems, automatically configures the terminal to non-canonical,
     * non-echoing mode via an internal RAII guard on first call.
     *
     * @return True if a keypress was pending and consumed; false otherwise.
     */
    bool ConsumeKeyPress();
}