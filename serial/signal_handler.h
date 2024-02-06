#ifndef SIGNAL_HANDLER_H
#define SIGNAL_HANDLER_H

#include <iostream>
#include <csignal>
#include "simple_serial.h"

void signalHandler(int signum) {
    std::cout << "Interrupt signal (" << signum << ") received. Attempting to exit gracefully..." << std::endl;

    COM3->Close();

    std::cout << "Successfully closed the serial port." << std::endl;

    exit(signum);
}

#endif // SIGNAL_HANDLER_H