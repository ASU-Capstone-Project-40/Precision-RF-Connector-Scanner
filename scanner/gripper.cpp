#include "logging.h"        // Supports optional verbose logging
#include "simple_serial.h"  // Handles serial communication
#include <iomanip>
#include <stdlib.h>     //for using the function sleep
#include <chrono>
#include <thread>


SimpleSerial *Gripper = nullptr;

int Logger::log_level_ = Logger::Level::DEBUG;

int main(int argc, char* argv[]) {

    std::string gripper_port = "COM4";
    int gripper_rate = 115200;
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--log_level" || arg == "-log") {
            if (argc < i+1) {
                Logger::warn(arg + " flag provided but no value specified. Using default log level.");
                continue;
            }
            Logger::setLogLevel((argv[i+1]));
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
        else if (arg == "--gripper-rate") {
            if (argc < i+1) {
                Logger::warn(arg + " flag provided but no value specified. Using default gripper baud rate " + std::to_string(gripper_rate));
                continue;
            }
            gripper_rate = std::stoi(argv[i+1]);
            ++i;
        }
        else {
            Logger::warn("Arg " + arg + " not recognized. Ignoring...");
        }
    }

    Logger::info("Opening new serial connection on " + gripper_port + " at rate " + std::to_string(gripper_rate));
    Gripper = new SimpleSerial(gripper_port, gripper_rate);

    Logger::warn("Attempting to initialize gripper.");

    unsigned char data[] = {0x01, 0x06, 0x01, 0x00, 0x00, 0x01, 0x49, 0xF6};
    Gripper->writeBytes(data, sizeof(data));
    Gripper->readBytes(sizeof(data));

    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    // Move to 500 command from manual
    unsigned char move[] = {0x01, 0x06, 0x01, 0x03, 0x01, 0xF4, 0x78, 0x21};
    Gripper->writeBytes(move, sizeof(move));
    Gripper->readBytes();

    Gripper->Close();
    Logger::info("All done!");

    return 0;
}
