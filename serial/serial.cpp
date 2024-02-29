#include "signal_handler.h" // Handles graceful shutdowns when ctrl+c is pressed
#include "logging.h"        // Supports optional verbose logging
#include "simple_serial.h"  // Handles serial communication
#include "sel_interface.h"   // Defines SEL controller commands
#include "datastore.h"      // Parses and stores system data for easy access

SimpleSerial *SEL = nullptr;
int Logger::log_level_ = Logger::Level::INFO;

int main(int argc, char* argv[]) {

    std::string sel_port = "COM3";
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--log-level" || arg == "-log") {
            if (argc < i+1) {
                Logger::warn(arg + " flag provided but no value specified. Using default log level.");
                continue;
            }
            Logger::setLogLevel((argv[i+1]));
            i++;
            continue;
        }
        else if (arg == "--sel-port" || arg == "-p") {
            if (argc < i+1) {
                Logger::warn(arg + " flag provided but no value specified. Using default SEL port " + sel_port);
                continue;
            }
            sel_port = argv[i+1];
            i++;
            continue;
        }
        else {
            Logger::warn(arg + " flag not recognized. Ignoring.");
        }
    }

    Logger::info("Opening new serial connection on " + sel_port + " at rate " + std::to_string(9600));
    SEL = new SimpleSerial(sel_port, 9600);

    // Register signal handler to close serial port when ctrl+c is pressed
    signal(SIGINT, signalHandler);

    try {
        SEL_Interface::Test("helloworld");

        auto& DS = Datastore::getInstance();
        DS.Update();

        SEL_Interface::Home(SEL_Interface::Axis::XY);
        DS.waitForMotionComplete();

        SEL_Interface::MoveToPosition({0.0, 0.0});
        DS.waitForMotionComplete();

        SEL_Interface::MoveToPosition({0.0, 200.0});
        DS.waitForMotionComplete();

        SEL_Interface::MoveToPosition({100.0, 200.0});
        DS.waitForMotionComplete();

        SEL_Interface::MoveToPosition({100.0, 0.0});
        DS.waitForMotionComplete();

        SEL_Interface::MoveToPosition({200.0, 0.0});
        DS.waitForMotionComplete();

        SEL_Interface::MoveToPosition({200.0, 200.0});
        DS.waitForMotionComplete();

        SEL_Interface::MoveToPosition({0.0, 0.0});
        DS.waitForMotionComplete();

        SEL_Interface::HaltAll();

        SEL->Close();
        Logger::info("All done!");
    }

    catch (const std::exception& e) {
        Logger::error("Exception caught. " + std::string(e.what()) + "\nAttempting to exit gracefully.");
        SEL_Interface::HaltAll();
        SEL->Close();
        Logger::info("Shutdown complete, re-throwing the error.");
        throw;
    }

    catch (...) {
        Logger::error("Unknown exception caught. Attempting to exit gracefully.");
        SEL_Interface::HaltAll();
        SEL->Close();
        Logger::info("Shutdown complete, re-throwing the error.");
        throw;
    }

    return 0;
}
