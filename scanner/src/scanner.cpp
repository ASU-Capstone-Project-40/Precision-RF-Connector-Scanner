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
    int resolution_x = 1280;  // px
    int resolution_y = 1024; // px
    double tolerance = 10.0; // px

    // Workspace parameters
    double workspace_x = 250.0;  // mm
    double workspace_y = 450.0; // mm
    double camera_to_gripper_x = -163.8173; // mm
    double camera_to_gripper_y = 0.0; // mm

    // Scanning parameters (mm)
    double scan_width = 30.0; // mm
    double scan_speed = 30.0; // mm/s
    double max_refinement_speed = 1.0; // mm/s
    double refinement_speed_scale_factor = max_refinement_speed / ((std::max)(resolution_x, resolution_y) / 2); // Scales the jog speed proportionally to the error

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

        SEL_Interface::MoveToPosition({0.0, 0.0});
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
        double object_x_px = 0;
        double object_y_px = 0;

        Path path = buildScanPath(workspace_x, workspace_y, scan_width);

        std::vector<double> detected_location; // The position of the end effector when the object was detected

        // Begin scan
        Logger::debug("Entering Scan Loop");
        for (auto& point : path) {
            SEL_Interface::MoveToPosition(point, scan_speed);
            DS.UpdateSEL();

            ResultData result;
            while(DS.x_axis.in_motion_ || DS.y_axis.in_motion_) { // Continously get camera data and check if move has completed
                DS.UpdateSEL();

                // Get camera data
                if (!object_detected)
                {
                    if(!detectObject(resultCollector, result)) {
                        continue;
                    }
                }
                
                if (detected_location.empty()) {
                    detected_location.push_back(DS.x_axis.position_);
                    detected_location.push_back(DS.y_axis.position_);
                }

                SEL_Interface::HaltAll();
                object_detected = true;
                object_x_px = result.positions_px[0].X;
                object_y_px = result.positions_px[0].Y;
                Logger::info("Detected object at " + std::to_string(object_x_px) + ", " + std::to_string(object_y_px));
            }

            // Robot is now stationary, whether through halting or arriving at target
            if (object_detected)
                break;
        }

        // At this point, scan is complete
        if (object_detected) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            resultCollector.ClearOutputData();

            SEL_Interface::MoveToPosition(detected_location, scan_speed);
            DS.waitForMotionComplete();

            // Refine position to place camera directly over connector
            bool within_tolerance = false;
            Logger::debug("Entering refinement loop...");
            ResultData result;
            while (!within_tolerance || DS.x_axis.in_motion_ || DS.y_axis.in_motion_) {
                if(!detectObject(resultCollector, result)) {
                    SEL_Interface::HaltAll();
                    continue;
                }

                object_x_px = result.positions_px[0].X;
                object_y_px = result.positions_px[0].Y;

                bool x_within_tolerance = std::abs(object_x_px - resolution_x/2) < tolerance;
                if (!x_within_tolerance && !DS.x_axis.in_motion_) {
                    double x_err = object_x_px - resolution_x/2;
                    auto jog_direction = static_cast<SEL_Interface::Direction>((std::max)((x_err > 0 ? 1 : -1) * camera_x_alignment * -1, 0)); //TODO: Clean this up
                    SEL_Interface::Jog(SEL_Interface::Axis::X, jog_direction, 1);
                }
                else if (x_within_tolerance && DS.x_axis.in_motion_) {
                    SEL_Interface::Halt(SEL_Interface::Axis::X);
                }


                bool y_within_tolerance = std::abs(object_y_px - resolution_y/2) < tolerance;
                if (!y_within_tolerance && !DS.y_axis.in_motion_) {
                    double y_err = object_y_px - resolution_y/2;
                    auto jog_direction = static_cast<SEL_Interface::Direction>((std::max)((y_err > 0 ? 1 : -1) * camera_y_alignment * -1,0));
                    SEL_Interface::Jog(SEL_Interface::Axis::Y, jog_direction, 1);
                }
                else if (y_within_tolerance && DS.y_axis.in_motion_) {
                    SEL_Interface::Halt(SEL_Interface::Axis::Y);
                }

                within_tolerance = x_within_tolerance && y_within_tolerance;

                Logger::info("px: " + std::to_string(result.positions_px[0].X) + ", " + std::to_string(result.positions_px[0].Y));
                std::string tolerance_string = within_tolerance ? "true" : "false";
                Logger::info("Within tolerance: " + tolerance_string);
                DS.UpdateSEL();

                if (DS.x_axis.position_ > workspace_x || DS.y_axis.position_ > workspace_y) {
                    SEL_Interface::HaltAll();
                    throw RUNTIME_EXCEPTION("End effector detected leaving the workspace!");
                }
            }

            // Translate xy to place gripper directly over connector
            SEL_Interface::MoveToPosition({DS.x_axis.position_ + camera_to_gripper_x, DS.y_axis.position_ + camera_to_gripper_y}, scan_speed);
            DS.waitForMotionComplete();
            // Z down to mate with connector
            DS.MoveRC(15);
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
        SEL_Interface::MoveToPosition({0, 0});
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