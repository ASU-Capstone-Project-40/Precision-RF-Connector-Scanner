#include "signal_handler.h" // Handles graceful shutdowns when ctrl+c is pressed
#include "logging.h"        // Supports optional verbose logging
#include "simple_serial.h"  // Handles serial communication
#include "sel_interface.h"   // Defines SEL controller commands
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
        SEL_Interface::Test("helloworld");
        SEL_Interface::Home(SEL_Interface::Axis::XY);
        DS.Update();

        while(DS.x_axis.in_motion_ || DS.y_axis.in_motion_) {
            DS.Update();
        }

        SEL_Interface::MoveToPosition({100.0, 200.0});
        DS.Update();

        DS.waitForMotionComplete();

        SEL_Interface::Jog(SEL_Interface::Axis::XY, SEL_Interface::Direction::NEGATIVE);

        DS.waitForMotionComplete();
        
        SEL->Close();
        logv("All done!");
    }

    catch (const std::exception& e) {
        std::cerr << "Exception caught. " << std::string(e.what()) << "\n Attempting to exit gracefully." << std::endl;
        SEL_Interface::HaltAll();
        SEL->Close();
        std::cerr << "Shutdown complete, re-throwing the error.";
        throw;
    }

    catch (...) {
        std::cout << "Unknown exception caught. Attempting to exit gracefully." << std::endl;
        SEL_Interface::HaltAll();
        SEL->Close();
        std::cerr << "Shutdown complete, re-throwing the error.";
        throw;
    }

    return 0;
}
