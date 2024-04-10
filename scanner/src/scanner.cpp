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

SimpleSerial *SEL = nullptr;
SimpleSerial *Gripper = nullptr;
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
    int camera_x_alignment = AxisAlignment::INVERTED; // Camera x with respect to robot x
    int camera_y_alignment = AxisAlignment::ALIGNED; // Camera y with respect to robot y
    double tolerance = 0.1; // mm
    double distance_scale_factor = 0.625; // Prevents overshoot if the distance measured is greater than actual distance

    // Workspace parameters
    double workspace_x = 400.0;  // mm
    double workspace_y = 600.0; // mm
    double camera_to_gripper_x = -165.6673; // mm
    double camera_to_gripper_y = 1.75; // mm

    // Scanning parameters (mm)
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
        auto& DS = Datastore::getInstance();

        // Ensure the end effector starts from the origin
        DS.MoveRC(RCPositions::HOME);
        DS.waitForZMotionComplete();

        SEL_Interface::MoveToPosition({-camera_to_gripper_x + scan_width/2, 0.0}, scan_speed);
        DS.waitForMotionComplete();

        // Initialize the gripper
        Gripper_Interface::Initialize();
        wait(500);

        Logger::info("Confirm when ready to close gripper.");
        system("pause");
        Gripper_Interface::Close();

        Logger::info("Confirm when ready to continue.");
        system("pause");

        // Before using any pylon methods, the pylon runtime must be initialized.
        PylonInitialize();

        // Initialize Pylon stuff
        // This object is used for collecting the output data.
        // If placed on the stack, it must be created before the recipe
        // so that it is destroyed after the recipe.
        RecipeOutputObserver resultCollector;
        CRecipe recipe; // Create a recipe object representing a recipe file created using the pylon Viewer Workbench.
        recipe.Load(PYLON_RECIPE); // Load the recipe file.
        recipe.PreAllocateResources(); // Now we allocate all resources we need. This includes the camera device if used in the recipe.
        recipe.RegisterAllOutputsObserver(&resultCollector, RegistrationMode_Append); // This is where the output goes.

        // Start the processing.
        recipe.Start();

        bool object_detected = false;

        Path path = buildScanPath(-camera_to_gripper_x + scan_width/2, workspace_x, workspace_y, scan_width);

        XYZ initial_measurement; // The measured location of the connector as of the first detection
        XYZ initial_detection_position; // The joint state of the robot when the connector was first detected

        // Begin scan
        Logger::debug("Entering Scan Loop");
        for (size_t i = 0; i < path.size(); ++i) {
            SEL_Interface::MoveToPosition(path[i], scan_speed);
            DS.UpdateSEL();

            while(DS.in_motion) { // Continously get camera data and check if move has completed
                DS.UpdateSEL();
                XYZ initial_detection_position = DS.position;

                // Get camera data
                ResultData result;
                resultCollector.ClearOutputData();
                if(!detectObject(resultCollector, result)) {
                    continue;
                }
                
                SEL_Interface::HaltAll(); // Calling this first allows the robot to drift a little further over the connector
                                         // before saving the position, hopefully preventing us from moving back to a position
                                        // where the connector is not yet in frame, since it's always initially detected at the
                                       // edge of the frame.

                DS.waitForMotionComplete();
                wait(2000);
                resultCollector.ClearOutputData();

                if (object_detected) {
                    break;
                }

                SEL_Interface::MoveToPosition(path[i-1], refinement_speed);
                DS.UpdateSEL();
                object_detected = true;
            }

            // Robot is now stationary, whether through halting or arriving at target
            if (object_detected)
                break;
        }

        // At this point, scan is complete
        if (object_detected) {

            // Refine position to place camera directly over connector
            bool within_tolerance = false;
            Logger::debug("Entering refinement loop...");

            while (!within_tolerance) {
                // Clear any old results (likely overkill)
                resultCollector.ClearOutputData();

                ResultData result;
                if(!detectObject(resultCollector, result)) {
                    Logger::error("No object detected in refinement loop");
                    continue;
                }

                DS.UpdateSEL();

                double x_err = result.positions_m[0].X * camera_x_alignment * 1000;
                double y_err = result.positions_m[0].Y * camera_y_alignment * 1000;
                XYZ err = XYZ(x_err, y_err);
                XYZ detected_location = DS.position - err * distance_scale_factor;

                within_tolerance = err.magnitude() < tolerance;

                Logger::info("Current Position: " + DS.position.toString());
                Logger::info("Detected Error: " + err.toString());
                Logger::info("Target position: " + detected_location.toString());

                if (within_tolerance) {
                    break;
                }

                if (detected_location.x > workspace_x || detected_location.y > workspace_y) {
                    Logger::error("Invalid target position: " + std::to_string(detected_location.x) + ", " + std::to_string(detected_location.y));
                    continue;
                }
                
                SEL_Interface::MoveToPosition(detected_location, refinement_speed);
                DS.waitForMotionComplete();
                wait(2000);

            }

            // Translate xy to place gripper directly over connector
            DS.UpdateSEL();
            SEL_Interface::MoveToPosition({DS.x_axis.position + camera_to_gripper_x, DS.y_axis.position + camera_to_gripper_y}, scan_speed);
            DS.waitForMotionComplete();
            
            // Z down to mate with connector
            DS.MoveRC(RCPositions::POUNCE);
            DS.waitForZMotionComplete();

            Logger::info("Confirm position before mating.");
            system("pause");
            DS.MoveRC(RCPositions::MATE);
            DS.waitForZMotionComplete();

            // Open gripper
            wait(100);
            Gripper_Interface::Open();
            wait(500);

            // Z up
            DS.MoveRC(RCPositions::HOME);
            DS.waitForZMotionComplete();
        }

        // Stop the image processing.
        recipe.Stop();
        SEL_Interface::MoveToPosition({-camera_to_gripper_x + scan_width/2, 0}, scan_speed);
        DS.waitForMotionComplete();
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

    // Releases all pylon resources.
    PylonTerminate();

    return exitCode;
}