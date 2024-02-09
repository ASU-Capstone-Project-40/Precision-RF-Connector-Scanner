#include "signal_handler.h" // Handles graceful shutdowns when ctrl+c is pressed
#include "logging.h"        // Supports optional verbose logging
#include "simple_serial.h"  // Handles serial communication
#include "sel_commands.h"   // Defines SEL controller commands
#include "datastore.h"      // Parses and stores system data for easy access

SimpleSerial *SEL = nullptr;

int main(int argc, char* argv[]) {
    
    VERBOSE_LOGGING = false;
    std::string sel_port = "COM3";
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--verbose" || arg == "-v") {
            VERBOSE_LOGGING = true;
            continue;
        }
        else {
            sel_port = arg;
        }
    }

    logv("Showing verbose logs");

    logv("Opening new serial connection on " + sel_port + " at rate " + std::to_string(9600)); // TODO: Make this part of the SimpleSerial constructor
    SEL = new SimpleSerial(sel_port, 9600);

    // Register signal handler to close serial port when ctrl+c is pressed
    signal(SIGINT, signalHandler);

    try {
        auto& DS = Datastore::getInstance();
        SelCommands::Test("helloworld");
        SelCommands::Home(true, true);
        DS.Update();

        while(DS.x_axis.in_motion_ || DS.y_axis.in_motion_) {
            DS.Update();
        }

        SelCommands::MoveToPosition({100.0, 200.0});
        DS.Update();

        while(DS.x_axis.in_motion_ || DS.y_axis.in_motion_) {
            DS.Update();
        }
        
        SEL->Close();
        logv("All done!");
    }
    catch (const std::exception& e) {
        std::cerr << "Exception caught. " << std::string(e.what()) << " Attempting to exit gracefully." << std::endl;
        // TODO: Cancel motion
        SEL->Close();
        std::cerr << "Successfully closed the serial port. Now re-throwing the error.";
        throw;
    }
    catch (...) {
        std::cout << "Unknown exception caught. Attempting to exit gracefully." << std::endl;
        // TODO: Cancel motion
        SEL->Close();
        std::cerr << "Successfully closed the serial port. Now re-throwing the error.";
        throw;
    }

    return 0;
}
