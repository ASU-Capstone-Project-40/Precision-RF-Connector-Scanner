#ifndef LOGGING_H
#define LOGGING_H

#include <iostream>

bool VERBOSE_LOGGING;

void logv(const std::string& message) {
    if (VERBOSE_LOGGING) {
        std::cout << message << std::endl;
    }
}

#endif // LOGGING_H