#ifndef LOGGING_LOGGER_HPP
#define LOGGING_LOGGER_HPP

#include "../controller/SafetyState.hpp"
#include "../collision/RiskEngine.hpp"
#include <string>
#include <fstream>
#include <iostream>
#include <memory>

namespace agv {
namespace logging {

enum class LogLevel {
    DEBUG,
    INFO,
    WARN,
    CRITICAL
};

class Logger {
public:
    static Logger& getInstance();

    void initialize(const std::string& log_file_path = "", bool console_output = true);
    void close();

    void log(LogLevel level, const std::string& message);
    void logStateTransition(controller::SafetyState from,
                            controller::SafetyState to,
                            double timestamp,
                            const std::string& reason);
    void logCycle(double timestamp,
                  double agv_x,
                  double agv_y,
                  double agv_v,
                  controller::SafetyState state,
                  double min_dist,
                  double ttc,
                  double cmd_v);

    void setConsoleOutput(bool enable) { console_output_ = enable; }

private:
    Logger() = default;
    ~Logger();
    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;

    std::ofstream file_stream_;
    bool console_output_{true};
    bool file_open_{false};
};

}
}

#endif
