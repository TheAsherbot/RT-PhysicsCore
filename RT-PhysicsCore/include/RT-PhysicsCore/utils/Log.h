#pragma once

#include <memory>
#include <sstream>
#include <string>

namespace RT_PhysicsCore
{
    enum class LogLevel : int
    {
        Trace = 0,
        Debug = 1,
        Info  = 2,
        Warn  = 3,
        Error = 4,
        Fatal = 5
    };

    const char* ToString(LogLevel level);

    // A destination for log output. Write() receives one already-formatted
    // line (timestamp, level, file:line, message all baked in) - sinks just
    // decide where that line goes.
    class ILogSink
    {
    public:
        virtual ~ILogSink();
        virtual void Write(LogLevel level, const std::string& formattedLine) = 0;
    };

    // Console sink. Colors by level and enables ANSI/VT100 processing on
    // Windows automatically; Trace/Debug/Info go to stdout, Warn/Error/Fatal
    // go to stderr so they can be redirected separately.
    class ConsoleLogSink : public ILogSink
    {
    public:
        explicit ConsoleLogSink(bool useColor = true);
        void Write(LogLevel level, const std::string& formattedLine) override;

    private:
        bool useColor;
    };

    // Appends (or truncates, by default) formatted lines to a file. Flushes
    // every line - costs some I/O throughput, but means a crash mid-run
    // doesn't lose the tail of the log, which matters more here.
    class FileLogSink : public ILogSink
    {
    public:
        explicit FileLogSink(const std::string& path, bool append = false);
        ~FileLogSink() override;
        void Write(LogLevel level, const std::string& formattedLine) override;

    private:
        struct Impl;
        std::unique_ptr<Impl> impl;
    };

    // Builds "<directory>/RT-PhysicsCore_YYYY-MM-DD_HH-MM-SS.log" and creates
    // <directory> if it doesn't exist yet. Convenience for FileLogSink's path.
    std::string DefaultLogFilePath(const std::string& directory = "logs");

    // Static log facade - no instance to fetch, just call Log::Write (or,
    // normally, go through the RT_LOG_* macros below instead).
    //
    // A ConsoleLogSink is registered automatically on first use, so basic
    // logging works with zero setup. Nothing else is automatic: library
    // code (Engine, Scene, systems, ...) should only ever call RT_LOG_*,
    // never touch sinks - only the application (main.cpp) decides where
    // logs actually go, e.g. by calling AddSink for a log file.
    class Log
    {
    public:
        static void SetMinLevel(LogLevel level);
        static LogLevel GetMinLevel();
        static bool IsLevelEnabled(LogLevel level);

        static void AddSink(std::unique_ptr<ILogSink> sink);
        static void ClearSinks(); // removes the default console sink too

        // Formats one line and dispatches it to every registered sink.
        // Prefer the RT_LOG_* macros: they capture file/line for you and,
        // when RT_LOG_ACTIVE_LEVEL strips a level at compile time, skip
        // building the message entirely rather than just filtering it here.
        static void Write(LogLevel level, const char* file, int line, const std::string& message);
    };
}

// --- Compile-time level gate ---
// Define RT_LOG_ACTIVE_LEVEL (0=Trace .. 5=Fatal) before including this
// header - e.g. as a CMake compile definition - to strip lower levels out
// of the build entirely. Below the active level, RT_LOG_* expands to
// nothing: the logged expression isn't even evaluated. Defaults to
// everything enabled in debug builds, Info-and-up in NDEBUG (release)
// builds, if not set explicitly.
#ifndef RT_LOG_ACTIVE_LEVEL
    #ifdef NDEBUG
        #define RT_LOG_ACTIVE_LEVEL 2
    #else
        #define RT_LOG_ACTIVE_LEVEL 0
    #endif
#endif

#define RT_LOG_IMPL(level, expr)                                             \
    do {                                                                     \
        if (::RT_PhysicsCore::Log::IsLevelEnabled(level)) {                  \
            std::ostringstream rt_log_stream;                                \
            rt_log_stream << expr;                                           \
            ::RT_PhysicsCore::Log::Write(level, __FILE__, __LINE__,          \
                                          rt_log_stream.str());              \
        }                                                                    \
    } while (0)

#if RT_LOG_ACTIVE_LEVEL <= 0
    #define RT_LOG_TRACE(expr) RT_LOG_IMPL(::RT_PhysicsCore::LogLevel::Trace, expr)
#else
    #define RT_LOG_TRACE(expr) do {} while (0)
#endif

#if RT_LOG_ACTIVE_LEVEL <= 1
    #define RT_LOG_DEBUG(expr) RT_LOG_IMPL(::RT_PhysicsCore::LogLevel::Debug, expr)
#else
    #define RT_LOG_DEBUG(expr) do {} while (0)
#endif

#if RT_LOG_ACTIVE_LEVEL <= 2
    #define RT_LOG_INFO(expr) RT_LOG_IMPL(::RT_PhysicsCore::LogLevel::Info, expr)
#else
    #define RT_LOG_INFO(expr) do {} while (0)
#endif

#if RT_LOG_ACTIVE_LEVEL <= 3
    #define RT_LOG_WARN(expr) RT_LOG_IMPL(::RT_PhysicsCore::LogLevel::Warn, expr)
#else
    #define RT_LOG_WARN(expr) do {} while (0)
#endif

#if RT_LOG_ACTIVE_LEVEL <= 4
    #define RT_LOG_ERROR(expr) RT_LOG_IMPL(::RT_PhysicsCore::LogLevel::Error, expr)
#else
    #define RT_LOG_ERROR(expr) do {} while (0)
#endif

// Fatal always compiles in, even in a fully-stripped release build - if
// something's fatal you want to know about it regardless. Note this only
// *logs* - it doesn't abort/exit on its own; the caller decides what to do
// after a fatal log, so the logging system never has surprise control-flow
// side effects.
#define RT_LOG_FATAL(expr) RT_LOG_IMPL(::RT_PhysicsCore::LogLevel::Fatal, expr)
