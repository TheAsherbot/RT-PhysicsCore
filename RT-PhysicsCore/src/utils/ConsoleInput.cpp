#include "RT-PhysicsCore/Utils/ConsoleInput.h"

#ifdef _WIN32

#include <conio.h>

namespace RT_PhysicsCore
{
    bool ConsumeKeyPress()
    {
        if (_kbhit())
        {
            _getch();
            return true;
        }
        return false;
    }
}

#else // POSIX (Linux, macOS)

#include <termios.h>
#include <unistd.h>
#include <sys/select.h>

namespace RT_PhysicsCore
{
    namespace
    {
        // RAII guard: puts stdin into non-canonical, non-echoing mode for
        // the lifetime of the program, restoring the original settings on
        // exit. Constructed once via a function-local static, so setup runs
        // on the first call to ConsumeKeyPress() and teardown runs at
        // normal static destruction (program exit) - no manual atexit()
        // bookkeeping needed.
        struct TerminalRawModeGuard
        {
            termios original{};
            bool valid = false;

            TerminalRawModeGuard()
            {
                if (tcgetattr(STDIN_FILENO, &original) != 0)
                    return; // stdin isn't a real terminal (e.g. redirected) - leave it alone

                termios raw = original;
                raw.c_lflag &= ~(ICANON | ECHO);
                raw.c_cc[VMIN] = 0;
                raw.c_cc[VTIME] = 0;

                if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) == 0)
                    valid = true;
            }

            ~TerminalRawModeGuard()
            {
                if (valid)
                    tcsetattr(STDIN_FILENO, TCSANOW, &original);
            }
        };
    }

    bool ConsumeKeyPress()
    {
        static TerminalRawModeGuard guard;
        (void)guard;

        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(STDIN_FILENO, &fds);
        timeval timeout{0, 0}; // zero timeout: poll, never block

        int ready = select(STDIN_FILENO + 1, &fds, nullptr, nullptr, &timeout);
        if (ready <= 0)
            return false;

        char c;
        return read(STDIN_FILENO, &c, 1) > 0;
    }
}

#endif
