/**
 * @file Log.cpp
 * @brief Implementation of logging sinks, formatting, and console output.
 */

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

        const char* LevelColorCode(LogLevel level)
        {
            switch (level)
            {
            case LogLevel::Trace: return "\033[90m";     // bright black (gray)
            case LogLevel::Debug: return "\033[36m";     // cyan
            case LogLevel::Info:  return "\033[32m";     // green
            case LogLevel::Warn:  return "\033[33m";     // yellow
            case LogLevel::Error: return "\033[31m";     // red
            case LogLevel::Fatal: return "\033[1;31;47m";// bold red on white
            }
            return "\033[0m";
        }

        constexpr const char* kResetColor = "\033[0m";

#ifdef _WIN32
        void EnableVirtualTerminalProcessing()
        {
            static const bool enabled = [] {
                HANDLE hOut = GetStdHandle(STD_OUTPUT_HANDLE);
                if (hOut == INVALID_HANDLE_VALUE) return false;
                DWORD dwMode = 0;
                if (!GetConsoleMode(hOut, &dwMode)) return false;
                dwMode |= ENABLE_VIRTUAL_TERMINAL_PROCESSING;
                return SetConsoleMode(hOut, dwMode) != 0;
                }();
            (void)enabled;
        }
#endif

        const char* StripPath(const char* fullPath)
        {
            const char* slash = fullPath;
            for (const char* p = fullPath; *p; ++p)
            {
                if (*p == '/' || *p == '\\')
                {
                    slash = p + 1;
                }
            }
            return slash;
        }
    } // namespace

    ConsoleLogSink::ConsoleLogSink(bool useColor)
        : useColor(useColor)
    {
#ifdef _WIN32
        if (useColor)
        {
            EnableVirtualTerminalProcessing();
        }
#endif
    }

    void ConsoleLogSink::Write(LogLevel level, const std::string& formattedLine)
    {
        std::ostream& stream = (level >= LogLevel::Warn) ? std::cerr : std::cout;

        if (useColor)
        {
            stream << LevelColorCode(level) << formattedLine << kResetColor << '\n';
        }
        else
        {
            stream << formattedLine << '\n';
        }
    }

    struct FileLogSink::Impl
    {
        std::ofstream stream;
    };

    FileLogSink::FileLogSink(const std::string& path, bool append)
        : impl(std::make_unique<Impl>())
    {
        std::error_code ec;
        std::filesystem::path p(path);
        if (p.has_parent_path())
        {
            std::filesystem::create_directories(p.parent_path(), ec);
        }

        auto mode = std::ios::out | (append ? std::ios::app : std::ios::trunc);
        impl->stream.open(path, mode);
    }

    FileLogSink::~FileLogSink() = default;

    void FileLogSink::Write(LogLevel /*level*/, const std::string& formattedLine)
    {
        if (impl->stream.is_open())
        {
            impl->stream << formattedLine << '\n';
            impl->stream.flush();
        }
    }

    std::string DefaultLogFilePath(const std::string& directory)
    {
        auto now = std::chrono::system_clock::now();
        std::time_t nowTime = std::chrono::system_clock::to_time_t(now);

        std::tm localTm{};
#ifdef _WIN32
        localtime_s(&localTm, &nowTime);
#else
        localtime_r(&nowTime, &localTm);
#endif

        std::ostringstream ss;
        ss << directory << "/RT-PhysicsCore_"
            << std::put_time(&localTm, "%Y-%m-%d_%H-%M-%S")
            << ".log";
        return ss.str();
    }

    void Log::SetMinLevel(LogLevel level)
    {
        std::lock_guard<std::mutex> lock(SinkMutex());
        MinLevelRef() = level;
    }

    LogLevel Log::GetMinLevel()
    {
        std::lock_guard<std::mutex> lock(SinkMutex());
        return MinLevelRef();
    }

    bool Log::IsLevelEnabled(LogLevel level)
    {
        std::lock_guard<std::mutex> lock(SinkMutex());
        return level >= MinLevelRef();
    }

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
        auto now = std::chrono::system_clock::now();
        auto micros = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()) % 1000000;
        std::time_t nowTime = std::chrono::system_clock::to_time_t(now);

        std::tm localTm{};
#ifdef _WIN32
        localtime_s(&localTm, &nowTime);
#else
        localtime_r(&nowTime, &localTm);
#endif

        std::ostringstream ss;
        ss << '[' << std::put_time(&localTm, "%H:%M:%S")
            << '.' << std::setfill('0') << std::setw(6) << micros.count()
            << "] [" << ToString(level)
            << "] [" << StripPath(file) << ':' << line
            << "] " << message;

        std::string lineStr = ss.str();

        std::lock_guard<std::mutex> lock(SinkMutex());
        if (level < MinLevelRef())
        {
            return;
        }

        for (auto& sink : Sinks())
        {
            sink->Write(level, lineStr);
        }
    }
}