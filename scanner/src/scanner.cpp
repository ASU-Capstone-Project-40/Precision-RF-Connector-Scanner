#include <WinSock2.h>
#include <iostream>

#include "../include/ResultData.h"
#include "../include/OutputObserver.h"

#include "../include/logging.h"           // Supports optional verbose logging
#include "../include/simple_serial.h"     // Handles serial communication
#include "../include/sel_interface.h"     // Defines SEL controller commands
#include "../include/gripper_interface.h" // Defines gripper commands
#include "../include/commander.h"         // Parses and stores system data for easy access
#include "../include/scanner.h"
#include "../include/xy.h"

// Namespaces for using pylon objects
using namespace Pylon;
using namespace Pylon::DataProcessing;

// Initialize globals to allow header files access to these objects
SimpleSerial *SEL = nullptr;
SimpleSerial *Gripper = nullptr;
Commander *commander = nullptr; 

int Logger::log_level_ = Logger::Level::INFO;

int main(int argc, char* argv[])
{
    // The exit code of the sample application.
    int exitCode = 0;

    // SEL controller default parameters
    std::string sel_port = "COM3";
    int sel_rate = 9600;

    // Gripper default parameters
    std::string gripper_port = "COM6";
    int gripper_rate = 115200;

    enum AxisAlignment {
        ALIGNED = 1,
        INVERTED = -1
    };

    // Camera parameters
    XY camera_alignment = XY(AxisAlignment::INVERTED, AxisAlignment::ALIGNED); 
    double fixed_tolerance = 0.1;
    double mobile_tolerance = 1.0; // mm
    double fixed_scale_factor = 0.59; // Prevents overshoot if the distance measured is greater than actual distance
    double mobile_scale_factor = 0.59; 

    // Workspace parameters
    XY workspace = XY(400.0, 450.0);
    XY camera_to_gripper = XY(-164.1, 0.5); // mm

    // Scanning parameters
    double scan_width = 35.0; // mm
    int scan_speed = 200; // mm/s
    int refinement_speed = 200; // mm/s
    XY mobile_scan_start = XY(-camera_to_gripper.x + scan_width, 0.0);

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
        
        // Create commander
        commander = Commander::getInstance();

        // Ensure the end effector starts from the origin
        commander->MoveRC(RCPositions::HOME);
        commander->waitForZMotionComplete();

        SEL_Interface::MoveToPosition(mobile_scan_start, scan_speed);
        commander->waitForXYMotionComplete();

        // Initialize the gripper
        Gripper_Interface::Initialize();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        Gripper_Interface::Open();

        // Initialize object recognition model
        auto Scanner = PylonRecipe(SCANNER_RECIPE, camera_alignment);

        // Create scan path
        Path scan_path = buildScanPath(mobile_scan_start, workspace, scan_width);
        
        // Find mobile connector, record fixed connector location if seen
        auto fixed_position = XY();
        auto [success, fixed_found] = ScanForMobile(Scanner, scan_path, scan_speed, fixed_position);

        if (success) {
            success = RefineToMobile(Scanner, refinement_speed, mobile_tolerance, mobile_scale_factor, camera_alignment);
        }

        if (success) {
            // Grasp mobile connector
            commander->UpdateSEL();
            commander->GraspMobile(camera_to_gripper, scan_speed, false);
        
            // Set up to find fixed connector
            auto fixed_scan_start = (fixed_found ? fixed_position : (commander->position - camera_to_gripper));

            SEL_Interface::MoveToPosition(fixed_scan_start, scan_speed);

            commander->waitForAllMotionComplete();
            
            success = ScanForFixed(Scanner, scan_path, scan_speed);

            if (!success) {
                Path new_scan_path = buildScanPath(mobile_scan_start, workspace, scan_width);
                success = ScanForFixed(Scanner, new_scan_path, scan_speed);
            }
        }

        if (success) {
            success = RefineToFixed(Scanner, refinement_speed, fixed_tolerance, fixed_scale_factor, camera_alignment);
        }

        Scanner.Stop();

        if (success) {
            commander->MateMobileToFixed(camera_to_gripper, scan_speed, false);
        }

        SEL_Interface::MoveToPosition(mobile_scan_start, scan_speed);
        commander->waitForAllMotionComplete();
        Gripper_Interface::Open();
    }

    catch (const GenericException& e)
    {
        SEL_Interface::HaltAll();
        std::cerr << "An exception occurred (likely pylon) - " <<std::endl << e.GetDescription() <<std::endl;
        exitCode = 1;
        throw;
    }

    catch (const std::exception& e)
    {
        SEL_Interface::HaltAll();
        std::cout << "Exception caught :" << e.what() << std::endl;
        exitCode = 1;
        throw;
    }

    return exitCode;
}