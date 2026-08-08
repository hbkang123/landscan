#include "payload_manager.hpp"
#include "application.hpp"

#include <iostream>

bool PayloadManager::initialize()
{
    std::cout << "[INFO] PayloadManager initialize" << std::endl;

    if (!ApplicationInit()) {
        std::cerr << "[ERROR] ApplicationInit failed" << std::endl;
        return false;
    }

    if (!ApplicationStart()) {
        std::cerr << "[ERROR] ApplicationStart failed" << std::endl;
        return false;
    }

    return true;
}

void PayloadManager::shutdown()
{
    ApplicationStop();
}