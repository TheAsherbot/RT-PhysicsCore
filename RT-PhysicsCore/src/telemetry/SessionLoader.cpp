/**
 * @file SessionLoader.cpp
 * @brief Implementation of JSON trace file parsing and session file management.
 */

#include "RT-PhysicsCore/telemetry/SessionLoader.h"

#include <algorithm>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

namespace
{
    /**
     * @brief Skips whitespace characters in a string starting at a given position.
     * @param str The string to scan.
     * @param pos The position to start from, advanced past whitespace on return.
     */
    void SkipWhitespace(const std::string& str, size_t& pos)
    {
        while (pos < str.size() && (str[pos] == ' ' || str[pos] == '\t' || str[pos] == '\n' || str[pos] == '\r'))
        {
            ++pos;
        }
    }

    /**
     * @brief Extracts a JSON string value (without quotes) starting after the opening quote.
     * @param str The JSON string to parse from.
     * @param pos Current position (should be at the opening quote), advanced past the closing quote.
     * @return The extracted string content.
     */
    std::string ParseJsonString(const std::string& str, size_t& pos)
    {
        if (pos >= str.size() || str[pos] != '"')
        {
            return "";
        }
        ++pos; // skip opening quote

        std::string result;
        while (pos < str.size() && str[pos] != '"')
        {
            if (str[pos] == '\\' && pos + 1 < str.size())
            {
                ++pos;
            }
            result += str[pos];
            ++pos;
        }

        if (pos < str.size())
        {
            ++pos; // skip closing quote
        }
        return result;
    }

    /**
     * @brief Parses a numeric value (integer) from the JSON string at the given position.
     * @param str The JSON string to parse from.
     * @param pos Current position, advanced past the number.
     * @return The parsed integer value.
     */
    long long ParseJsonNumber(const std::string& str, size_t& pos)
    {
        size_t start = pos;
        if (pos < str.size() && str[pos] == '-')
        {
            ++pos;
        }
        while (pos < str.size() && str[pos] >= '0' && str[pos] <= '9')
        {
            ++pos;
        }
        // Skip decimal portion if present (we only need integers)
        if (pos < str.size() && str[pos] == '.')
        {
            ++pos;
            while (pos < str.size() && str[pos] >= '0' && str[pos] <= '9')
            {
                ++pos;
            }
        }
        return std::stoll(str.substr(start, pos - start));
    }
}

namespace RT_PhysicsCore
{
    bool SessionLoader::Load(const std::string& filepath, std::vector<ProfileResult>& outResults)
    {
        std::ifstream file(filepath);
        if (!file.is_open())
        {
            return false;
        }

        std::stringstream buffer;
        buffer << file.rdbuf();
        std::string content = buffer.str();
        file.close();

        // Find the start of the traceEvents array
        size_t arrayStart = content.find('[');
        if (arrayStart == std::string::npos)
        {
            return false;
        }

        size_t pos = arrayStart + 1;
        SkipWhitespace(content, pos);

        // Temporary storage for string names since ProfileResult uses const char*
        // We store them in a static deque to guarantee pointers are never invalidated during growth
        static std::deque<std::string> nameStorage;
        nameStorage.clear();

        while (pos < content.size() && content[pos] != ']')
        {
            if (content[pos] == ',')
            {
                ++pos;
                SkipWhitespace(content, pos);
                continue;
            }

            if (content[pos] != '{')
            {
                ++pos;
                continue;
            }
            ++pos; // skip '{'

            std::string name;
            long long dur = 0;
            long long ts = 0;
            uint32_t tid = 0;
            std::size_t memoryCurrentBytes = 0;
            std::size_t memoryPeakBytes = 0;
            std::size_t memoryAllocations = 0;

            // Parse key-value pairs in this object
            while (pos < content.size() && content[pos] != '}')
            {
                SkipWhitespace(content, pos);
                if (content[pos] == ',')
                {
                    ++pos;
                    SkipWhitespace(content, pos);
                    continue;
                }

                std::string key = ParseJsonString(content, pos);
                SkipWhitespace(content, pos);
                if (pos < content.size() && content[pos] == ':')
                {
                    ++pos;
                }
                SkipWhitespace(content, pos);

                if (key == "name")
                {
                    name = ParseJsonString(content, pos);
                }
                else if (key == "dur")
                {
                    dur = ParseJsonNumber(content, pos);
                }
                else if (key == "ts")
                {
                    ts = ParseJsonNumber(content, pos);
                }
                else if (key == "tid")
                {
                    tid = static_cast<uint32_t>(ParseJsonNumber(content, pos));
                }
                else if (key == "memory_current")
                {
                    memoryCurrentBytes = static_cast<std::size_t>(ParseJsonNumber(content, pos));
                }
                else if (key == "memory_peak")
                {
                    memoryPeakBytes = static_cast<std::size_t>(ParseJsonNumber(content, pos));
                }
                else if (key == "memory_allocs")
                {
                    memoryAllocations = static_cast<std::size_t>(ParseJsonNumber(content, pos));
                }
                else if (pos < content.size() && content[pos] == '"')
                {
                    ParseJsonString(content, pos); // skip string value
                }
                else
                {
                    ParseJsonNumber(content, pos); // skip numeric value
                }
            }

            if (pos < content.size())
            {
                ++pos; // skip '}'
            }

            nameStorage.push_back(name);
            ProfileResult result;
            result.name = nameStorage.back().c_str();
            result.startTimestamp = ts;
            result.endTimestamp = ts + dur;
            result.threadId = tid;
            result.memoryCurrentBytes = memoryCurrentBytes;
            result.memoryPeakBytes = memoryPeakBytes;
            result.memoryAllocations = memoryAllocations;
            outResults.push_back(result);

            SkipWhitespace(content, pos);
        }

        return !outResults.empty();
    }

    std::vector<SessionInfo> SessionLoader::DiscoverSessions(const std::string& directory)
    {
        namespace fs = std::filesystem;
        std::vector<SessionInfo> sessions;

        if (!fs::exists(directory) || !fs::is_directory(directory))
        {
            return sessions;
        }

        for (const auto& entry : fs::directory_iterator(directory))
        {
            if (!entry.is_regular_file())
            {
                continue;
            }

            std::string filename = entry.path().filename().string();
            if (filename.find("run_") != 0 || filename.find(".json") == std::string::npos)
            {
                continue;
            }

            SessionInfo info;
            info.filepath = entry.path().string();
            info.fileSizeBytes = static_cast<std::size_t>(entry.file_size());
            info.isStarred = (filename.find("_STARRED") != std::string::npos);

            // Extract timestamp from filename: run_YYYY-MM-DD_HH-MM-SS[_STARRED].json
            std::string stem = entry.path().stem().string(); // "run_2026-10-01_23-44-38" or "run_2026-10-01_23-44-38_STARRED"
            std::string timestamp = stem.substr(4); // strip "run_"
            size_t starredPos = timestamp.find("_STARRED");
            if (starredPos != std::string::npos)
            {
                timestamp = timestamp.substr(0, starredPos);
            }

            info.timestamp = timestamp;

            // Format display name: "2026-10-01 23:44:38"
            std::string display = timestamp;
            // Replace the middle underscore with a space: "2026-10-01 23-44-38"
            size_t dateTimeSep = display.find('_');
            if (dateTimeSep != std::string::npos)
            {
                display[dateTimeSep] = ' ';
            }
            // Replace time dashes with colons: "2026-10-01 23:44:38"
            for (size_t i = dateTimeSep + 1; i < display.size(); ++i)
            {
                if (display[i] == '-')
                {
                    display[i] = ':';
                }
            }
            info.displayName = display;

            sessions.push_back(info);
        }

        // Sort newest first
        std::sort(sessions.begin(), sessions.end(), [](const SessionInfo& a, const SessionInfo& b)
            {
                return a.timestamp > b.timestamp;
            });

        return sessions;
    }

    bool SessionLoader::ToggleStar(SessionInfo& session)
    {
        namespace fs = std::filesystem;

        fs::path oldPath = session.filepath;
        std::string stem = oldPath.stem().string();
        std::string extension = oldPath.extension().string();
        fs::path directory = oldPath.parent_path();

        std::string newStem;
        if (session.isStarred)
        {
            // Remove _STARRED
            size_t pos = stem.find("_STARRED");
            if (pos != std::string::npos)
            {
                newStem = stem.substr(0, pos);
            }
            else
            {
                return false;
            }
        }
        else
        {
            newStem = stem + "_STARRED";
        }

        fs::path newPath = directory / (newStem + extension);

        std::error_code ec;
        fs::rename(oldPath, newPath, ec);
        if (ec)
        {
            return false;
        }

        session.filepath = newPath.string();
        session.isStarred = !session.isStarred;
        return true;
    }

    bool SessionLoader::DeleteSession(const SessionInfo& session)
    {
        if (session.isStarred)
        {
            return false;
        }

        namespace fs = std::filesystem;
        std::error_code ec;
        return fs::remove(session.filepath, ec);
    }

    int SessionLoader::DeleteAllUnstarred(std::vector<SessionInfo>& sessions)
    {
        int deleted = 0;
        auto it = sessions.begin();
        while (it != sessions.end())
        {
            if (!it->isStarred && DeleteSession(*it))
            {
                it = sessions.erase(it);
                ++deleted;
            }
            else
            {
                ++it;
            }
        }
        return deleted;
    }

    std::vector<FrameSummary> SessionLoader::ReconstructFrameSummaries(const std::vector<ProfileResult>& results)
    {
        std::vector<FrameSummary> summaries;

        for (size_t i = 0; i < results.size(); ++i)
        {
            const ProfileResult& r = results[i];
            if (!r.name || std::string(r.name) != "Frame")
            {
                continue;
            }

            FrameSummary summary;
            summary.totalFrameMs = static_cast<double>(r.endTimestamp - r.startTimestamp) / 1000.0;
            summary.physicsMs = 0.0;
            summary.updateMs = 0.0;
            summary.renderMs = 0.0;

            // Scan forward for child scopes that fall within this frame
            for (size_t j = i + 1; j < results.size(); ++j)
            {
                const ProfileResult& child = results[j];
                if (child.startTimestamp >= r.endTimestamp)
                {
                    break;
                }

                if (!child.name)
                {
                    continue;
                }

                std::string childName(child.name);
                double childMs = static_cast<double>(child.endTimestamp - child.startTimestamp) / 1000.0;

                if (childName == "FixedUpdate")
                {
                    summary.physicsMs += childMs;
                }
                else if (childName == "Update")
                {
                    summary.updateMs += childMs;
                }
                else if (childName == "Render")
                {
                    summary.renderMs += childMs;
                }
            }

            summaries.push_back(summary);
        }

        return summaries;
    }
}