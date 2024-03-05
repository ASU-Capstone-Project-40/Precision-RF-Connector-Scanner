#include "logging.h"        // Supports optional verbose logging
#include "simple_serial.h"  // Handles serial communication
#include <iomanip>
#include <stdlib.h>     //for using the function sleep


SimpleSerial *Gripper = nullptr;

int Logger::log_level_ = Logger::Level::DEBUG;

int main(int argc, char* argv[]) {

    std::string gripper_port = "COM4";
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
        else if (arg == "--gripper-port") {
            if (argc < i+1) {
                Logger::warn(arg + " flag provided but no value specified. Using default gripper port.");
                continue;
            }
            gripper_port = argv[i+1];
            i++;
            continue;
        }
        else {
            Logger::warn("Arg " + arg + " not recognized. Ignoring...");
        }
    }

    Logger::info("Opening new serial connection on " + gripper_port + " at rate " + std::to_string(115200));
    Gripper = new SimpleSerial(gripper_port, 115200);


    Logger::warn("Attempting to initialize gripper.");

    unsigned char data[] = {0x01, 0x06, 0x01, 0x00, 0x01, 0x49, 0xF6};
    Gripper->writeBytes(data, sizeof(data));
    Gripper->readLine();

    Logger::info("All done!");

    return 0;
}
