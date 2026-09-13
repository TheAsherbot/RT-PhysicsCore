#pragma once

namespace RT_PhysicsCore
{

    // Checks stdin for a pending keypress without blocking, and consumes it
    // (discards the character) if one is waiting. Returns true if a key was
    // consumed, false if nothing was pending.
    //
    // Cross-platform replacement for conio.h's _kbhit()+_getch() pair:
    //   if (_kbhit()) { _getch(); ... }   // Windows-only
    //   if (ConsumeKeyPress()) { ... }    // Windows, Linux, macOS
    //
    // On POSIX (Linux/macOS), the first call puts the terminal into
    // non-canonical, non-echoing mode so a keypress is seen immediately
    // without waiting for Enter - matching _getch()'s behavior on Windows.
    // The original terminal settings are restored automatically at program
    // exit. If stdin isn't an interactive terminal (e.g. input has been
    // redirected from a file), this is skipped and the function falls back
    // to plain non-blocking reads.
    bool ConsumeKeyPress();
}
