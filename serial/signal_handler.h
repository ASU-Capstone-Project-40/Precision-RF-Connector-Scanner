#ifndef SIGNAL_HANDLER_H
#define SIGNAL_HANDLER_H

#include <iostream>
#include <csignal>
#include "simple_serial.h" 
#include "sel_commands.h"

void signalHandler(int signum) {
    std::cout << "Interrupt signal (" << signum << ") received. Exiting gracefully..." << std::endl;

    SelCommands::HaltAll();
    SEL->Close();

    exit(signum);
}

#endif // SIGNAL_HANDLER_H