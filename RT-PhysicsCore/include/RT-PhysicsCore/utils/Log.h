/**
 * @file Log.h
 * @brief Multi-sink thread-safe logging subsystem with compile-time level stripping.
 *
 * Provides formatted output to console (with ANSI color codes) and files, with
 * stream-operator concatenation macros and zero-overhead dead-stripping in release builds.
 */

#pragma once

#include <memory>
#include <sstream>
#include <string>

namespace RT_PhysicsCore
{
    /**
     * @enum LogLevel
     * @brief Severity classification levels for log events.
     */
    enum class LogLevel : int
    {
        Trace = 0,
        Debug = 1,
        Info = 2,
        Warn = 3,
        Error = 4,
        Fatal = 5
    };

    /**
     * @brief Converts a LogLevel enum value to a 5-character uppercase string representation.
     * @param level Severity level.
     * @return String literal token (e.g. "INFO ", "ERROR").
     */
    const char* ToString(LogLevel level);

    /**
     * @class ILogSink
     * @brief Abstract destination sink receiving formatted log messages.
     */
    class ILogSink
    {
    public:
        virtual ~ILogSink();

        /**
         * @brief Dispatches one pre-formatted log line to the destination output.
         * @param level Severity level of the message.
         * @param formattedLine Fully formatted text string including timestamps and source tags.
         */
        virtual void Write(LogLevel level, const std::string& formattedLine) = 0;
    };

    /**
     * @class ConsoleLogSink
     * @brief Terminal log sink with VT100/ANSI color highlighting.
     */
    class ConsoleLogSink : public ILogSink
    {
    public:
        /**
         * @brief Constructs console sink.
         * @param useColor True to format severity with ANSI terminal colors.
         */
        explicit ConsoleLogSink(bool useColor = true);

        void Write(LogLevel level, const std::string& formattedLine) override;

    private:
        bool useColor;
    };

    /**
     * @class FileLogSink
     * @brief Disk log sink appending timestamped records to a file with per-line flushing.
     */
    class FileLogSink : public ILogSink
    {
    public:
        /**
         * @brief Constructs a file sink targeting a specific file path.
         * @param path Target log destination path on disk.
         * @param append True to append to existing file; false to overwrite.
         */
        explicit FileLogSink(const std::string& path, bool append = false);
        ~FileLogSink() override;

        void Write(LogLevel level, const std::string& formattedLine) override;

    private:
        struct Impl;
        std::unique_ptr<Impl> impl;
    };

    /**
     * @brief Generates a standard timestamped log filename: `<dir>/RT-PhysicsCore_YYYY-MM-DD_HH-MM-SS.log`.
     * @param directory Output folder path (default: "logs").
     * @return Full relative file path string.
     */
    std::string DefaultLogFilePath(const std::string& directory = "logs");

    /**
     * @class Log
     * @brief Static facade coordinating sink distribution and severity thresholding.
     */
    class Log
    {
    public:
        /**
         * @brief Sets minimum severity level processed by registered sinks.
         * @param level Lowest active severity level.
         */
        static void SetMinLevel(LogLevel level);

        /**
         * @brief Gets current minimum active log severity level.
         * @return Active LogLevel.
         */
        static LogLevel GetMinLevel();

        /**
         * @brief Checks if a specific severity level is enabled for processing.
         * @param level Level to test.
         * @return True if level meets or exceeds GetMinLevel().
         */
        static bool IsLevelEnabled(LogLevel level);

        /**
         * @brief Registers an additional output destination sink.
         * @param sink Unique pointer to ILogSink instance.
         */
        static void AddSink(std::unique_ptr<ILogSink> sink);

        /**
         * @brief Clears all registered sinks, including the default console sink.
         */
        static void ClearSinks();

        /**
         * @brief Formats a log line and dispatches it to all active sinks.
         * @param level Severity level.
         * @param file Source file originating the log event.
         * @param line Source line number.
         * @param message Text payload.
         */
        static void Write(LogLevel level, const char* file, int line, const std::string& message);
    };
}

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

#define RT_LOG_FATAL(expr) RT_LOG_IMPL(::RT_PhysicsCore::LogLevel::Fatal, expr)