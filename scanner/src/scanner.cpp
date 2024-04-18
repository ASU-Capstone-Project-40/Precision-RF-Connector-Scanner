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
    double tolerance = 0.1; // mm
    double measurement_scale_factor = 0.65; // Prevents overshoot if the distance measured is greater than actual distance

    // Workspace parameters
    double workspace_x = 400.0;  // mm
    double workspace_y = 600.0; // mm
    double camera_to_gripper_x = -165.6673; // mm
    double camera_to_gripper_y = 1.75; // mm

    // Scanning parameters
    double scan_width = 50.0; // mm
    int scan_speed = 100.0; // mm/s
    double refinement_speed = 25.0; // mm/s

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

        SEL_Interface::MoveToPosition({-camera_to_gripper_x + scan_width/2, 0.0}, scan_speed);
        DS->waitForMotionComplete();

        // Initialize the gripper
        Gripper_Interface::Initialize();
        wait(500);

        Logger::info("Confirm when ready to close gripper.");
        system("pause");
        Gripper_Interface::Close();

        Logger::info("Confirm when ready to continue.");
        system("pause");

        Path path = buildScanPath(-camera_to_gripper_x + scan_width/2, workspace_x, workspace_y, scan_width);

        auto mobileScanner = PylonRecipe(MOBILE_CONNECTOR_RECIPE, camera_alignment);
        
        bool success = Scan(mobileScanner, path, scan_speed);

        if (success) {
            success = Refine(mobileScanner, refinement_speed, tolerance, measurement_scale_factor);
        }

        mobileScanner.Stop();

        if (success) {
            // Translate xy to place gripper directly over connector
            DS->UpdateSEL();
            SEL_Interface::MoveToPosition({DS->x_axis.position + camera_to_gripper_x, DS->y_axis.position + camera_to_gripper_y}, scan_speed);
            DS->waitForMotionComplete();
            
            // Z down to mate with connector
            DS->MoveRC(RCPositions::POUNCE);
            DS->waitForZMotionComplete();

            Logger::info("Confirm position before mating.");
            system("pause");
            DS->MoveRC(RCPositions::MATE);
            DS->waitForZMotionComplete();

            // Close Gripper
            wait(100);
            Gripper_Interface::Close();
            wait(500);

            // Z up
            DS->MoveRC(RCPositions::HOME);
            DS->waitForZMotionComplete();
        }

        auto fixedScanner = PylonRecipe(FIXED_CONNECTOR_RECIPE, camera_alignment);

        success = Scan(fixedScanner, path, scan_speed);

        if (success) {
            success = Refine(fixedScanner, refinement_speed, tolerance, measurement_scale_factor);
        }

        fixedScanner.Stop();

        if (success) {
            // Translate xy to place gripper directly over connector
            DS->UpdateSEL();
            SEL_Interface::MoveToPosition({DS->x_axis.position + camera_to_gripper_x, DS->y_axis.position + camera_to_gripper_y}, scan_speed);
            DS->waitForMotionComplete();
            
            // Z down to mate with connector
            DS->MoveRC(RCPositions::POUNCE);
            DS->waitForZMotionComplete();

            Logger::info("Confirm position before mating.");
            system("pause");
            DS->MoveRC(RCPositions::MATE);
            DS->waitForZMotionComplete();

            // Open gripper
            wait(100);
            Gripper_Interface::Open();
            wait(500);

            // Z up
            DS->MoveRC(RCPositions::HOME);
            DS->waitForZMotionComplete();
        }

        SEL_Interface::MoveToPosition({-camera_to_gripper_x + scan_width/2, 0}, scan_speed);
        DS->waitForMotionComplete();
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