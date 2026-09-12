#include "RT-PhysicsCore/utils/Log.h"

#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <vector>

#ifdef _WIN32
    #include <windows.h>
#endif

namespace RT_PhysicsCore
{
    const char* ToString(LogLevel level)
    {
        switch (level)
        {
            case LogLevel::Trace: return "TRACE";
            case LogLevel::Debug: return "DEBUG";
            case LogLevel::Info:  return "INFO ";
            case LogLevel::Warn:  return "WARN ";
            case LogLevel::Error: return "ERROR";
            case LogLevel::Fatal: return "FATAL";
        }
        return "?????";
    }

    ILogSink::~ILogSink() = default;

    // --- internal state -----------------------------------------------
    // Function-local statics (Meyer's singletons) rather than class-level
    // static data members: lazily and thread-safely initialized on first
    // use (guaranteed since C++11), so there's no static-initialization-
    // order risk even if some other global/static object tries to log
    // during its own construction, before main() runs.
    namespace
    {
        std::mutex& SinkMutex()
        {
            static std::mutex m;
            return m;
        }

        std::vector<std::unique_ptr<ILogSink>>& Sinks()
        {
            static std::vector<std::unique_ptr<ILogSink>> sinks = [] {
                std::vector<std::unique_ptr<ILogSink>> initial;
                initial.push_back(std::make_unique<ConsoleLogSink>());
                return initial;
            }();
            return sinks;
        }

        LogLevel& MinLevelRef()
        {
            static LogLevel level = LogLevel::Trace;
            return level;
        }

        std::string Timestamp()
        {
            using namespace std::chrono;
            auto now = system_clock::now();
            auto ms = duration_cast<milliseconds>(now.time_since_epoch()) % 1000;
            std::time_t t = system_clock::to_time_t(now);

            std::tm tmBuf{};
#ifdef _WIN32
            localtime_s(&tmBuf, &t);
#else
            localtime_r(&t, &tmBuf);
#endif
            std::ostringstream oss;
            oss << std::put_time(&tmBuf, "%H:%M:%S") << '.'
                << std::setfill('0') << std::setw(3) << ms.count();
            return oss.str();
        }

        std::string FileNameOnly(const char* fullPath)
        {
            std::string path(fullPath);
            auto slash = path.find_last_of("/\\");
            return slash == std::string::npos ? path : path.substr(slash + 1);
        }
    }

    // --- Log -------------------------------------------------------------
    void Log::SetMinLevel(LogLevel level) { MinLevelRef() = level; }
    LogLevel Log::GetMinLevel() { return MinLevelRef(); }
    bool Log::IsLevelEnabled(LogLevel level) { return level >= MinLevelRef(); }

    void Log::AddSink(std::unique_ptr<ILogSink> sink)
    {
        std::lock_guard<std::mutex> lock(SinkMutex());
        Sinks().push_back(std::move(sink));
    }

    void Log::ClearSinks()
    {
        std::lock_guard<std::mutex> lock(SinkMutex());
        Sinks().clear();
    }

    void Log::Write(LogLevel level, const char* file, int line, const std::string& message)
    {
        if (!IsLevelEnabled(level))
            return;

        std::ostringstream oss;
        oss << "[" << Timestamp() << "] [" << ToString(level) << "] "
            << "[" << FileNameOnly(file) << ":" << line << "] " << message;
        std::string formattedLine = oss.str();

        std::lock_guard<std::mutex> lock(SinkMutex());
        for (auto& sink : Sinks())
            sink->Write(level, formattedLine);
    }

    // --- ConsoleLogSink ----------------------------------------------------
    namespace
    {
        const char* ColorCodeFor(LogLevel level)
        {
            switch (level)
            {
                case LogLevel::Trace: return "\x1b[90m";   // gray
                case LogLevel::Debug: return "\x1b[36m";   // cyan
                case LogLevel::Info:  return "\x1b[37m";   // white
                case LogLevel::Warn:  return "\x1b[33m";   // yellow
                case LogLevel::Error: return "\x1b[31m";   // red
                case LogLevel::Fatal: return "\x1b[1;31m"; // bold red
            }
            return "\x1b[0m";
        }
        const char* ColorReset() { return "\x1b[0m"; }

#ifdef _WIN32
        void EnableVirtualTerminalOnWindows()
        {
            HANDLE hOut = GetStdHandle(STD_OUTPUT_HANDLE);
            if (hOut == INVALID_HANDLE_VALUE) return;
            DWORD mode = 0;
            if (!GetConsoleMode(hOut, &mode)) return;
            SetConsoleMode(hOut, mode | ENABLE_VIRTUAL_TERMINAL_PROCESSING);
        }
#endif
    }

    ConsoleLogSink::ConsoleLogSink(bool useColor) : useColor(useColor)
    {
#ifdef _WIN32
        if (useColor)
            EnableVirtualTerminalOnWindows();
#endif
    }

    void ConsoleLogSink::Write(LogLevel level, const std::string& formattedLine)
    {
        std::ostream& out = (level >= LogLevel::Warn) ? std::cerr : std::cout;
        if (useColor)
            out << ColorCodeFor(level) << formattedLine << ColorReset() << '\n';
        else
            out << formattedLine << '\n';
    }

    // --- FileLogSink ---------------------------------------------------
    struct FileLogSink::Impl
    {
        std::ofstream file;
    };

    FileLogSink::FileLogSink(const std::string& path, bool append)
        : impl(std::make_unique<Impl>())
    {
        impl->file.open(path, append ? (std::ios::out | std::ios::app) : std::ios::out);
        if (!impl->file.is_open())
            std::cerr << "[Log] Failed to open log file: " << path << std::endl;
    }

    FileLogSink::~FileLogSink() = default;

    void FileLogSink::Write(LogLevel /*level*/, const std::string& formattedLine)
    {
        if (!impl->file.is_open())
            return;
        impl->file << formattedLine << '\n';
        impl->file.flush(); // a crash mid-run shouldn't lose the tail of the log
    }

    // --- DefaultLogFilePath ----------------------------------------------
    std::string DefaultLogFilePath(const std::string& directory)
    {
        std::error_code ec;
        std::filesystem::create_directories(directory, ec); // ignored; sink reports open failure

        using namespace std::chrono;
        auto now = system_clock::now();
        std::time_t t = system_clock::to_time_t(now);

        std::tm tmBuf{};
#ifdef _WIN32
        localtime_s(&tmBuf, &t);
#else
        localtime_r(&t, &tmBuf);
#endif
        std::ostringstream oss;
        oss << directory << "/RT-PhysicsCore_"
            << std::put_time(&tmBuf, "%Y-%m-%d_%H-%M-%S") << ".log";
        return oss.str();
    }
}
