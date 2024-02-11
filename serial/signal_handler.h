#ifndef SIGNAL_HANDLER_H
#define SIGNAL_HANDLER_H

#include <iostream>
#include <csignal>
#include "simple_serial.h" 
#include "sel_interface.h"

void signalHandler(int signum) {
    Logger::warn("Interrupt signal '" + std::to_string(signum) + "' received. Exiting gracefully...");

    SEL_Interface::HaltAll();
    SEL->Close();

    exit(signum);
}

#endif // SIGNAL_HANDLER_H