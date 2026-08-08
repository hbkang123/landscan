#include "navigation_data_recorder.hpp"

#include <cstdint>
#include <filesystem>
#include <iostream>
#include <string>
#include <system_error>

bool NavigationDataRecorder::initialize(
    const std::string& rootDirectory,
    std::uint64_t sessionSTartOsalMs
) {
    std::lock_guard<std::mutex> lock(mutex_);

    if(initialized_) {
        return true;
    }

    const std::filesystem::path root(rootDirectory);
    const std::string sessionName = "session_" + std::to_string(sessionSTartOsalMs);
    sessionDirectory_ = root / sessionName;
    std::error_code error;
    unsigned int suffix = 1;

    while (std::filesystem::exists(sessionDirectory_, error) && !error) {
        sessionDirectory_ = root / (sessionName + "_" + std::to_string(suffix));

        ++suffix;
    }

    if(error) {
        std::cerr << "[Recorder] Failed to inspect session directory: " << error.message() << std::endl;

        return false;
    }

    std::filesystem::create_directories(sessionDirectory_, error);

    if(error) {
        std::cerr << "[Recorder] Failed to create directory: " << sessionDirectory_ << ", error=" << error.message() << std::endl;

        return false;
    }

    eventStream_.open(sessionDirectory_ / "events.log", std::ios::out | std::ios::trunc);

    if(!eventStream_.is_open()) {
        std::cerr << "[Recorder] Failed to open events.log" << std::endl;

        return false;
    }

    initialized_ = true;

    std::cout << "[Recorder] Session directory: " << sessionDirectory_ << std::endl;

    return true;
}

void NavigationDataRecorder::shutdown() {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!initialized_) {
        return;
    }

    for(auto& item : streams_) {
        if(item.second.is_open()) {
            item.second.flush();
            item.second.close();
        }
    }

    streams_.clear();

    if(eventStream_.is_open()) {
        eventStream_.flush();
        eventStream_.close();
    }

    initialized_ = false;

    std::cout << "[Recorder] Recording stopped" << std::endl;
}

void NavigationDataRecorder::flush() {
    std::lock_guard<std::mutex> lock(mutex_);

    for(auto& item : streams_) {
        if(item.second.is_open()) {
            item.second.flush();
        }
    }

    if(eventStream_.is_open()) {
        eventStream_.flush();
    }
}

void NavigationDataRecorder::writeEvent(std::uint64_t osalTimeMs, const std::string& message) {
    std::lock_guard<std::mutex> lock(mutex_);

    if(!initialized_ || ! eventStream_.is_open()) {
        return;
    }

    eventStream_ << osalTimeMs << ',' << message << '\n';
}

const std::filesystem::path& NavigationDataRecorder::getSessionDirectory() const  {
    return sessionDirectory_;
}