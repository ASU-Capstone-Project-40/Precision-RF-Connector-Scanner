#include <WinSock2.h>
#include <iostream>

#include "../include/ResultData.h"
#include "../include/OutputObserver.h"

#include "../include/logging.h"           // Supports optional verbose logging
#include "../include/simple_serial.h"     // Handles serial communication
#include "../include/sel_interface.h"     // Defines SEL controller commands
#include "../include/gripper_interface.h" // Defines gripper commands
#include "../include/datastore.h"         // Parses and stores system data for easy access
#include "../include/scanner.h"
#include "../include/xyz.h"

// Namespaces for using pylon objects
using namespace Pylon;
using namespace Pylon::DataProcessing;

// Initialize globals to allow header files access to these objects
SimpleSerial *SEL = nullptr;
SimpleSerial *Gripper = nullptr;
Datastore *DS = nullptr; 

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

    enum RCPositions {
        HOME = 0,
        POUNCE = 13,
        GRASP = 14,
        MATE = 15,
    };

    // Camera parameters
    XYZ camera_alignment = XYZ(AxisAlignment::INVERTED, AxisAlignment::ALIGNED); 
    double fixed_tolerance = 0.1;
    double mobile_tolerance = 1.0; // mm
    double fixed_scale_factor = 0.59; // Prevents overshoot if the distance measured is greater than actual distance
    double mobile_scale_factor = 0.59; 

    // Workspace parameters
    XYZ workspace = XYZ(400.0, 600.0);
    XYZ camera_to_gripper = XYZ(-165.1, 1.6); // mm

    // Scanning parameters
    double scan_width = 50.0; // mm
    int scan_speed = 200.0; // mm/s
    double refinement_speed = 200.0; // mm/s
    XYZ scan_start = XYZ(-camera_to_gripper.x + scan_width/2, 0.0);

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
        DS = Datastore::getInstance();

        // Ensure the end effector starts from the origin
        DS->MoveRC(RCPositions::HOME);
        DS->waitForZMotionComplete();

        SEL_Interface::MoveToPosition(scan_start, scan_speed);
        DS->waitForMotionComplete();

        // Initialize the gripper
        Gripper_Interface::Initialize();
        wait(100);
        Gripper_Interface::Open();

        auto Scanner = PylonRecipe(MOBILE_CONNECTOR_RECIPE, camera_alignment);
        Path path = buildScanPath(-camera_to_gripper.x + scan_width/2, workspace, scan_width);
        bool success = Scan(Scanner, path, scan_speed);

        if (success) {
            success = Refine(Scanner, refinement_speed, mobile_tolerance, mobile_scale_factor);
        }

        Scanner.Stop();

        if (success) {
            // Translate xy to place gripper directly over connector
            DS->UpdateSEL();
            SEL_Interface::MoveToPosition(DS->position + camera_to_gripper, scan_speed);

            DS->waitForMotionComplete();
            
            // Z down to mate with connector
            DS->MoveRC(RCPositions::GRASP);
            DS->waitForZMotionComplete();

            // Logger::info("Confirm position before grasping.");
            // system("pause");

            // Close Gripper
            Gripper_Interface::Close();
            wait(200);

            // Z up
            DS->MoveRC(RCPositions::HOME);
        }

        SEL_Interface::MoveToPosition(scan_start, scan_speed);
        Scanner.Load(FIXED_CONNECTOR_RECIPE);

        DS->waitForMotionComplete();
        DS->waitForZMotionComplete();

        if (success) {
            success = Scan(Scanner, path, scan_speed);
        }

        if (success) {
            success = Refine(Scanner, refinement_speed, fixed_tolerance, fixed_scale_factor);
        }

        Scanner.Stop();

        if (success) {
            // Translate xy to place gripper directly over connector
            DS->UpdateSEL();
            SEL_Interface::MoveToPosition(DS->position + camera_to_gripper, scan_speed);
            DS->waitForMotionComplete();
            
            // Z down to mate with connector
            DS->MoveRC(RCPositions::POUNCE);
            DS->waitForZMotionComplete();

            // Logger::info("Confirm position before mating.");
            // system("pause");
            DS->MoveRC(RCPositions::MATE);
            DS->waitForZMotionComplete();

            // Open gripper
            Gripper_Interface::Open();
            wait(100);

            // Z up
            DS->MoveRC(RCPositions::HOME);
        }

        SEL_Interface::MoveToPosition(scan_start, scan_speed);
        DS->waitForMotionComplete();
        DS->waitForZMotionComplete();
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