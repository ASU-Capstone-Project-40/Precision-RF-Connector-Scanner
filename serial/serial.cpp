#include "datastore.h"
#include "logging.h"
#include "signal_handler.h"

std::unique_ptr<SimpleSerial> SimpleSerial::instance_ = nullptr; // Initialize pointer to SimpleSerial singleton

int main(int argc, char* argv[]) {
    COM_PORT = "COM3";
    VERBOSE_LOGGING = false;
    std::cout << "argc: " << std::to_string(argc) << std::endl;
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--verbose" || arg == "-v") {
            VERBOSE_LOGGING = true;
            continue;
        }
        COM_PORT = arg;
    }

    logv("Showing verbose logs");
    logv("Using serial port " + COM_PORT);

    SelCommands SelCommand;

    // Register signal handler to close serial port on ctrl+c
    signal(SIGINT, signalHandler);

    try {
        Datastore DS;

        SelCommand.Test("helloworld");
        SelCommand.Home(true, true);
        DS.Update();
    }
    catch (const std::exception& e) {
        std::cerr << "Exception caught. " << std::string(e.what()) << " Attempting to exit gracefully." << std::endl;
        // TODO: Cancel motion
        SelCommand.CloseSerial();
        std::cerr << "Successfully closed the serial port. Now re-throwing the error.";
        throw;
    }
    catch (...) {
        std::cout << "Unknown exception caught. Attempting to exit gracefully." << std::endl;
        // TODO: Cancel motion
        SelCommand.CloseSerial();
        std::cerr << "Successfully closed the serial port. Now re-throwing the error.";
        throw;
    }

    return 0;
}
