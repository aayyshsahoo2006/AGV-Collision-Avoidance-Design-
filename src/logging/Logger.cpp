#include "../../include/logging/Logger.hpp"
#include <iomanip>

namespace agv {
namespace logging {

Logger& Logger::getInstance() {
    static Logger instance;
    return instance;
}

Logger::~Logger() {
    close();
}

void Logger::initialize(const std::string& log_file_path, bool console_output) {
    console_output_ = console_output;
    if (!log_file_path.empty()) {
        file_stream_.open(log_file_path, std::ios::out | std::ios::trunc);
        if (file_stream_.is_open()) {
            file_open_ = true;
            file_stream_ << "timestamp_s,agv_x,agv_y,agv_v,safety_state,min_dist_m,ttc_s,cmd_v\n";
        }
    }
}

void Logger::close() {
    if (file_open_ && file_stream_.is_open()) {
        file_stream_.close();
        file_open_ = false;
    }
}

void Logger::log(LogLevel level, const std::string& message) {
    const char* lvl_str = "INFO";
    switch (level) {
        case LogLevel::DEBUG: lvl_str = "DEBUG"; break;
        case LogLevel::INFO:  lvl_str = "INFO";  break;
        case LogLevel::WARN:  lvl_str = "WARN";  break;
        case LogLevel::CRITICAL: lvl_str = "CRITICAL"; break;
    }

    if (console_output_) {
        std::cout << "[" << lvl_str << "] " << message << "\n";
    }
}

void Logger::logStateTransition(controller::SafetyState from,
                                controller::SafetyState to,
                                double timestamp,
                                const std::string& reason) {
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(3)
       << "t=" << timestamp << "s: TRANSITION ["
       << controller::toString(from) << " -> " << controller::toString(to)
       << "] Reason: " << reason;
    log(LogLevel::INFO, ss.str());
}

void Logger::logCycle(double timestamp,
                      double agv_x,
                      double agv_y,
                      double agv_v,
                      controller::SafetyState state,
                      double min_dist,
                      double ttc,
                      double cmd_v) {
    if (file_open_) {
        file_stream_ << std::fixed << std::setprecision(4)
                     << timestamp << ","
                     << agv_x << ","
                     << agv_y << ","
                     << agv_v << ","
                     << controller::toString(state) << ","
                     << (min_dist > 1e6 ? -1.0 : min_dist) << ","
                     << (ttc > 1e6 ? -1.0 : ttc) << ","
                     << cmd_v << "\n";
    }
}

}
}
