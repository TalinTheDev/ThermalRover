#include "utils/logger.hpp"
#include <iostream>
#include <string>

Logger::Logger() {
    std::cout << "[INFO] Log Initialized" << std::endl;
}

Logger::~Logger() {
    std::cout << "[INFO] Log Destroyed" << std::endl;
}

void Logger::info(const std::string& msg) {
    std::cout << "[INFO] " << msg << std::endl;
}

void Logger::warn(const std::string& msg) {
    std::cout << "[WARN] " << msg << std::endl;
}

void Logger::error(const std::string& msg) {
    std::cout << "[ERROR] " << msg << std::endl;
}

void Logger::debug(const std::string& msg) {
    std::cout << "[DEBUG] " << msg << std::endl;
}