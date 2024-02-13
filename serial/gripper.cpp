#include "logging.h"        // Supports optional verbose logging
#include "simple_serial.h"  // Handles serial communication

SimpleSerial *Gripper = nullptr;

int Logger::log_level_ = Logger::Level::DEBUG;

int main(int argc, char* argv[]) {
    
    std::string gripper_port = "COM3";
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
        else if (arg == "--gripper_port" || arg == "-port") {
            if (argc < i+1) {
                Logger::warn(arg + " flag provided but no value specified. Using default gripper port.");
                continue;
            }
            gripper_port = argv[i+1];
            i++;
            continue;
        }
    }

    Logger::info("Opening new serial connection on " + gripper_port + " at rate " + std::to_string(115200));
    Gripper = new SimpleSerial(gripper_port, 115200);

    Logger::warn("Attempting to initialize gripper.");
    Gripper->writeString("01 06 01 00 01 49 F6");
    Gripper->readLine();
    Logger::warn("Success!");
    
    Logger::warn("Attempting to read the reference position currently set.");
    Gripper->writeString("01 03 01 03 00 01 75 F6");
    Gripper->readLine();
    Logger::warn("Success!");

    Logger::warn("Attempting to set gripper to 500 position.");
    Gripper->writeString("01 06 01 03 01 F4 78 21");
    Gripper->readLine();
    Logger::warn("Success!"); 

    return 0;
}
