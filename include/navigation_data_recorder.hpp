#pragma once

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <map>
#include <mutex>
#include <string>
#include <utility>

class NavigationDataRecorder {
public:
    bool initialize(
        const std::string& rootDirectory,
        uint64_t sessionStartOsalMis
    );

    void shutdown();
    void flush();

    template <typename... Values>
    void writeRow(
        const std::string& fileName,
        const std::string& header,
        Values&&... values
    ) {
        std::lock_guard<std::mutex> lock(mutex_);

        if(!initialized_) {
            return;
        }

        std::ofstream& stream = streams_[fileName];

        if(!stream.is_open()) {
            stream.open(sessionDirectory_ / fileName, std::ios::out | std::ios::trunc);

            if(!stream.is_open()) {
                return;
            }

            stream << std::setprecision(16);
            stream << header << "\n";
        }

        bool first = true;

        auto append = [&](const auto& value) {
            if(!first) {
                stream << ',';
            }

            first = false;
            stream << value;
        };

        (append(std::forward<Values>(values)), ...);
        stream << '\n';
    }

    void writeEvent(
        uint64_t osalTimeMs,
        const std::string& message
    );

    const std::filesystem::path& getSessionDirectory() const;

private:
    std::filesystem::path sessionDirectory_;
    std::map<std::string, std::ofstream> streams_;
    std::ofstream eventStream_;
    std::mutex mutex_;
    bool initialized_ = false;
};