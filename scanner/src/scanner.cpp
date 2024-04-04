#include <WinSock2.h>
#include <thread>

#include "../include/ResultData.h"
#include "../include/OutputObserver.h"

#include "../include/logging.h"           // Supports optional verbose logging
#include "../include/simple_serial.h"     // Handles serial communication
#include "../include/sel_interface.h"     // Defines SEL controller commands
#include "../include/gripper_interface.h" // Defines gripper commands
#include "../include/datastore.h"         // Parses and stores system data for easy access
#include "../include/scanner.h"
#include "../include/point.h"

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
    std::string gripper_port = "COM4";
    int gripper_rate = 115200;

    enum AxisAlignment {
        ALIGNED = 1,
        INVERTED = -1
    };

    // Camera parameters
    int camera_x_alignment = AxisAlignment::INVERTED; // Camera x with respect to robot x
    int camera_y_alignment = AxisAlignment::ALIGNED; // Camera y with respect to robot y
    double tolerance = 1.0; // mm
    double distance_scale_factor = 0.9; // Prevents overshoot if the distance measured is greater than actual distance

    // Workspace parameters
    
    double workspace_x = 250.0;  // mm
    double workspace_y = 450.0; // mm
    double camera_to_gripper_x = -163.8173; // mm
    double camera_to_gripper_y = 0.0; // mm

    // Scanning parameters (mm)
    double scan_width = 90.0; // mm
    int scan_speed = 100; // mm/s
    double refinement_speed = 35.0; // mm/s

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
        Logger::info("Opening new serial connection on " + sel_port + " at rate " + std::to_string(sel_rate));
        SEL = new SimpleSerial(sel_port, sel_rate);

        SEL_Interface::HaltAll(); // Halt all for safety

        Logger::info("Opening new serial connection on " + gripper_port + " at rate " + std::to_string(gripper_rate));
        Gripper = new SimpleSerial(gripper_port, gripper_rate);

        // Create datastore
        auto& DS = Datastore::getInstance();

        // Ensure the end effector starts from the origin
        DS.MoveRC(0);
        DS.waitForZMotionComplete();

        SEL_Interface::MoveToPosition({0.0, 0.0}, scan_speed);
        DS.waitForMotionComplete();

        // Initialize the gripper
        Gripper_Interface::Initialize();
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        Gripper_Interface::Open();

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

        Path path = buildScanPath(workspace_x, workspace_y, scan_width);

        Point initial_position_measurement; // The measured location of the connector as of the first detection
        Point initial_detection_position; // The joint state of the robot when the connector was first detected

        // Begin scan
        Logger::debug("Entering Scan Loop");
        for (auto& point : path) {
            SEL_Interface::MoveToPosition(point, refinement_speed);
            DS.UpdateSEL();

            while(DS.x_axis.in_motion || DS.y_axis.in_motion) { // Continously get camera data and check if move has completed
                DS.UpdateSEL();

                // Get camera data
                ResultData result;
                if(!detectObject(resultCollector, result)) {
                    continue;
                }

                DS.UpdateSEL();

                double x_err = result.positions_m[0].X * camera_x_alignment * 1000;
                double y_err = result.positions_m[0].Y * camera_y_alignment * 1000;
                initial_position_measurement.x = DS.x_axis.position - x_err; 
                initial_position_measurement.y = DS.y_axis.position - y_err;

                initial_detection_position.x = DS.x_axis.position;
                initial_detection_position.y = DS.y_axis.position;

                SEL_Interface::HaltAll();
                DS.waitForMotionComplete();

                SEL_Interface::MoveToPosition(initial_detection_position, refinement_speed); // Try switching this to initial_position_measurement
                DS.waitForMotionComplete();

                std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // Remove this later

                object_detected = true;
                break;
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

                DS.Update();

                double x_err = result.positions_m[0].X * camera_x_alignment * 1000;
                double y_err = result.positions_m[0].Y * camera_y_alignment * 1000;
                Point err = Point(x_err, y_err);
                Point detected_location = DS.position + err * distance_scale_factor;

                within_tolerance = err.magnitude() < tolerance;

                if (within_tolerance) {
                    break;
                }

                Logger::info(": " + std::to_string(x_err) + ", " + std::to_string(y_err));

                if (detected_location.x > workspace_x || detected_location.y > workspace_y) {
                    Logger::error("Invalid target position: " + std::to_string(detected_location.x) + ", " + std::to_string(detected_location.y));
                    continue;
                }
                
                SEL_Interface::MoveToPosition(detected_location, refinement_speed);
                DS.waitForMotionComplete();
            }

            // Translate xy to place gripper directly over connector
            DS.Update();
            SEL_Interface::MoveToPosition({DS.x_axis.position + camera_to_gripper_x, DS.y_axis.position + camera_to_gripper_y}, scan_speed);
            DS.waitForMotionComplete();
            // Z down to mate with connector
            DS.MoveRC(14);
            DS.waitForZMotionComplete();

            // Open gripper

            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            Gripper_Interface::Close();
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));

            // Z up
            DS.MoveRC(0);
            DS.waitForZMotionComplete();
        }

        // Stop the image processing.
        recipe.Stop();
        SEL_Interface::MoveToPosition({0, 0}, scan_speed);
        DS.waitForMotionComplete();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
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