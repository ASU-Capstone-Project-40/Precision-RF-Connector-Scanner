#ifndef LOGGING_H
#define LOGGING_H

#include <iostream>

class Logger {
public:

    enum Level {
    OFF = 0,
    ERR = 1,
    WARN = 2,
    INFO = 3,
    DEBUG = 4,
    VERBOSE = 5,
    };

    static void setLogLevel(int level) {
        switch (level) {
            case Level::OFF:
                log_level_ = Level::OFF;
                break;
            case Level::ERR:
                log_level_ = Level::ERR;
                break;
            case Level::WARN:
                log_level_ = Level::WARN;
                break;
            case Level::INFO:
                log_level_ = Level::INFO;
                info("Showing info level logs");
                break;
            case Level::DEBUG:
                log_level_ = Level::DEBUG;
                debug("Showing debug level logs");
                break;
            default:
                warn("Provided log level '" + std::to_string(level) + "' is not recognized.");
        }
    }

    static void setLogLevel(std::string level) {
        if (level == "off")
            log_level_ = Level::OFF;
        else if (level == "err" || level == "error")
            log_level_ = Level::ERR;
        else if (level == "warn" |level == "w")
            log_level_ = Level::WARN;
        else if (level == "info" || level == "i")
            log_level_ = Level::INFO;
        else if (level == "debug" || level == "d" )
            log_level_ = Level::DEBUG;
        else if (level == "verbose" | level == "v")
            log_level_ = Level::VERBOSE;
        else
            warn("Provided log level '" + level + "' is not recognized.");
    }

    static void error(const std::string& message) {
        if (log_level_ >= Level::ERR) {
            std::cout << RED << "[ERROR] " << message << RESET << std::endl;
        }
    }

    static void warn(const std::string& message) {
        if (log_level_ >= Level::WARN) {
            std::cout << ORANGE << "[WARN] " << message << RESET << std::endl;
        }
    }

    static void info(const std::string& message) {
        if (log_level_ >= Level::INFO) {
            std::cout << "[INFO] " << message << std::endl;
        }
    }

    static void debug(const std::string& message) {
        if (log_level_ >= Level::DEBUG) {
            std::cout << YELLOW << "[DEBUG] " << message << RESET << std::endl;
        }
    }

    static void verbose(const std::string& message) {
        if (log_level_ >= Level::VERBOSE) {
            std::cout << BLUE << "[DEBUG] " << message << RESET << std::endl;
        }
    }
        

    static void verbose_stream(char c){
        if (log_level_ >= Level::VERBOSE) {
            std::cout << BLUE << c << RESET << std::flush;
        }
    }

private:
    static int log_level_;
    static inline const std::string RED = "\033[31m";       // Red text
    static inline const std::string ORANGE = "\033[32m";   // Orange text 
    static inline const std::string YELLOW = "\033[33m";  // Yellow text
    static inline const std::string BLUE = "\033[34m";   // Blue text
    static inline const std::string RESET = "\033[0m";  // Reset
};

#endif // LOGGING_H