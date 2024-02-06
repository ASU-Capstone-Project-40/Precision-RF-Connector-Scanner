#include "signal_handler.h" // Handles graceful shutdowns when ctrl+c is pressed
#include "logging.h"        // Supports optional verbose logging
#include "simple_serial.h"  // Handles serial communication
#include "sel_commands.h"   // Defines SEL controller commands
#include "datastore.h"      // Parses and stores system data for easy access

SimpleSerial *COM3 = nullptr;

int main(int argc, char* argv[]) {
    
    VERBOSE_LOGGING = false;
    std::cout << "argc: " << std::to_string(argc) << std::endl;
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--verbose" || arg == "-v") {
            VERBOSE_LOGGING = true;
            continue;
        }
        else {
            std::cout << "Ignoring unknown flag " << arg << std::endl;
        }
    }

    logv("Showing verbose logs");

    COM3 = new SimpleSerial("COM3", 9600);

    // Register signal handler to close serial port when ctrl+c is pressed
    signal(SIGINT, signalHandler);

    try {
        auto& DS = Datastore::getInstance();
        SelCommands::Test("helloworld", COM3);
        SelCommands::Home(true, true, COM3);
        DS.Update(COM3);
    }
    catch (const std::exception& e) {
        std::cerr << "Exception caught. " << std::string(e.what()) << " Attempting to exit gracefully." << std::endl;
        // TODO: Cancel motion
        COM3->Close();
        std::cerr << "Successfully closed the serial port. Now re-throwing the error.";
        throw;
    }
    catch (...) {
        std::cout << "Unknown exception caught. Attempting to exit gracefully." << std::endl;
        // TODO: Cancel motion
        COM3->Close();
        std::cerr << "Successfully closed the serial port. Now re-throwing the error.";
        throw;
    }

    return 0;
}
