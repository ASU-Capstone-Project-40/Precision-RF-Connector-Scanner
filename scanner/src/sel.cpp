#include "../include/signal_handler.h" // Handles graceful shutdowns when ctrl+c is pressed
#include "../include/logging.h"        // Supports optional verbose logging
#include "../include/simple_serial.h"  // Handles serial communication
#include "../include/sel_interface.h"  // Defines SEL controller commands
#include "../include/datastore.h"      // Parses and stores system data for easy access
#include"../include/gripper_interface.h"
#include <thread>

SimpleSerial *SEL = nullptr;
SimpleSerial *Gripper = nullptr;
int Logger::log_level_ = Logger::Level::INFO;

void wait(unsigned int ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

int main(int argc, char* argv[]) {
    // SEL controller default parameters
    std::string sel_port = "COM3";
    int sel_rate = 9600;

    // Gripper default parameters
    std::string gripper_port = "COM6";
    int gripper_rate = 115200;
    // Testing Parameters
    auto test_point = XYZ(250.0, 250.0);

    enum RCPositions {
        HOME = 0,
        POUNCE = 13,
        GRASP = 14,
        MATE = 15,
    };

    // Handle command line arguments
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--log-level" || arg == "-log") {
            if (argc < i+1) {
                Logger::warn(arg + " flag provided but no value specified. Using default log level.");
                continue;
            }
            Logger::setLogLevel((argv[i+1]));
            ++i;
        }
        else if (arg == "--sel-port") {
            if (argc < i+1) {
                Logger::warn(arg + " flag provided but no value specified. Using default SEL port " + sel_port);
                continue;
            }
            sel_port = argv[i+1];
            ++i;
        }
        else if (arg == "--gripper-port") {
            if (argc < i+1) {
                Logger::warn(arg + " flag provided but no value specified. Using default gripper port " + gripper_port);
                continue;
            }
            gripper_port = argv[i+1];
            ++i;
        }
        else {
            Logger::warn(arg + " flag not recognized. Ignoring.");
        }
    }

    try
    {
        // Initialize serial connections
        SEL = new SimpleSerial(sel_port, sel_rate);
        SEL_Interface::HaltAll(); // Halt all for safety

        Gripper = new SimpleSerial(gripper_port, gripper_rate);

        // Create datastore
        auto& DS = Datastore::getInstance();

        Gripper_Interface::Initialize();
        wait(500);

        Logger::info("Confirm when ready to close gripper.");
        system("pause");
        Gripper_Interface::Close();

        Logger::info("Confirm when ready to continue.");
        system("pause");

        // Ensure the end effector starts from the origin
        DS.MoveRC(RCPositions::HOME);
        DS.waitForZMotionComplete();

        SEL_Interface::MoveToPosition(test_point);
        DS.waitForMotionComplete();

        SEL_Interface::MoveToPosition(test_point, 1);
        DS.waitForMotionComplete();

        DS.MoveRC(RCPositions::MATE);
        DS.waitForZMotionComplete();

        Logger::info("Confirm when ready to open gripper.");
        system("pause");
        Gripper_Interface::Open();

        Logger::info("Confirm when ready to continue.");
        system("pause");

        DS.MoveRC(RCPositions::HOME);
        DS.waitForZMotionComplete();

        SEL_Interface::MoveToPosition({0, 0});
        DS.waitForMotionComplete();

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
