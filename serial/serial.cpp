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
        if (arg == "--log_level" || arg == "-log") {
            if (argc < i+1) {
                Logger::warn(arg + " flag provided but no value specified. Using default log level.");
                continue;
            }
            Logger::setLogLevel((argv[i+1]));
            i++;
            continue;
        }
        else if (arg == "--sel_port" || arg == "-port") {
            if (argc < i+1) {
                Logger::warn(arg + " flag provided but no value specified. Using default log level.");
                continue;
            }
            sel_port = argv[i+1];
            i++;
            continue;
        }
    }

    Logger::info("Opening new serial connection on " + sel_port + " at rate " + std::to_string(9600));
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
