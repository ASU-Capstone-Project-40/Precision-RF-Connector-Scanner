#include <WinSock2.h>
// Include files to use the pylon API.
#include <pylon/PylonIncludes.h>

// Extend the pylon API for using pylon data processing.
#include <pylondataprocessing/PylonDataProcessingIncludes.h>

// The sample uses the std::list.
#include <list>
#include <algorithm>
#include <thread>

#include "../include/ResultData.h"
#include "../include/OutputObserver.h"

#include "../include/logging.h"           // Supports optional verbose logging
#include "../include/simple_serial.h"     // Handles serial communication
#include "../include/sel_interface.h"     // Defines SEL controller commands
#include "../include/gripper_interface.h" // Defines gripper commands
#include "../include/datastore.h"         // Parses and stores system data for easy access

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
    double tolerance = 1.0; // px

    // Workspace parameters
    double workspace_x = 200.0;  // mm
    double workspace_y = 400.0; // mm
    double camera_to_gripper_x = 163.8173; // mm
    double camera_to_gripper_y = 7.46506; // mm

    // Scanning parameters (mm)
    double scan_width = 30.0; // mm
    double scan_speed = 0.0; // mm/s TODO: Implement this
    double max_refinement_speed = 25.0; // mm/s
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
        Logger::info("Opening new serial connection on " + gripper_port + " at rate " + std::to_string(gripper_rate));
        Gripper = new SimpleSerial(gripper_port, gripper_rate);

        // Create datastore
        auto& DS = Datastore::getInstance();

        // Ensure the end effector starts from the origin
        SEL_Interface::MoveToPosition({0.0, 0.0});
        DS.waitForMotionComplete();
        SEL_Interface::SetOutputs({306, 305, 304, 303}, {1, 0, 0, 0, 0}, DS.SEL_outputs); // RC to p0
        SEL_Interface::SetOutputs({302}, {1}, DS.SEL_outputs);
        SEL_Interface::SetOutputs({302}, {0}, DS.SEL_outputs);
        // DS.waitForZMotionComplete();

        // Initialize the gripper
        Gripper_Interface::Initialize();
        Gripper_Interface::MoveTo();

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
        int num_passes = std::ceil(workspace_x / scan_width) + 1;

        // Begin scan
        for (int i = 0; i < num_passes*2; ++i) {
            double x_coordinate = (std::min)(scan_width * (i/2), workspace_x);
            double y_coordinate = workspace_y * (((i+1)/2) % 2);
            SEL_Interface::MoveToPosition({x_coordinate, workspace_y});

            DS.UpdateSEL();
            // Continously get camera data and check if move has completed
            while(DS.x_axis.in_motion_ || DS.y_axis.in_motion_) {
                Logger::warn("In the scan loop, updating SEL");
                DS.UpdateSEL();

                // Get camera data
                Logger::warn("In the scan loop, getting image");
                if (resultCollector.GetWaitObject().Wait(200)) // Blocks until image received, wait is ms
                {
                    ResultData result;
                    resultCollector.GetResultData(result);
                    if (!result.hasError) {
                        if (!result.scores.empty()) {
                            object_detected = true;
                            object_x_px = result.positions_px[0].X;
                            object_y_px = result.positions_px[0].Y;

                            // Calling halt instead of breaking here allows the camera to continue to take images
                            // while the axis drifts, ensuring that the last pictures is taken while the camera
                            // is stable
                            SEL_Interface::HaltAll();
                        }
                        else {
                            if(object_detected) {
                                Logger::error("Lost sight of object after initial detection!");
                            }
                        }
                    }
                    else {
                        std::cout << "An error occurred during processing recipe: " << result.errorMessage <<std::endl;
                    }
                }
                else
                {
                    throw RUNTIME_EXCEPTION("Result timeout");
                }

            }
            // Robot is now stationary, whether through halting or arriving at target
            if (object_detected)
                break;

            // No object detected but scan still in progress, continue to next pass
        }
        // At this point, scan is complete

        if (object_detected) {
            // Refine position to place camera directly over connector
            bool within_tolerance = false;
            while (!within_tolerance || DS.x_axis.in_motion_ || DS.y_axis.in_motion_) {
                Logger::debug("Inside refinement loop, taking an image...");
                // Take another image TODO: Make this a function
                if (resultCollector.GetWaitObject().Wait(200)) {
                    ResultData result;
                    resultCollector.GetResultData(result);
                    if (!result.hasError)
                    {
                        if (!result.scores.empty()) {
                            object_x_px = result.positions_px[0].X;
                            object_y_px = result.positions_px[0].Y;
                        }
                        else
                        {
                            SEL_Interface::HaltAll();
                            Logger::error("No object detected in refinement loop! Halting motion.");
                            continue;
                        }
                    }
                    else
                    {
                        SEL_Interface::HaltAll();
                        Logger::error("An error occurred during processing recipe. Halting Motion. ");
                        std::cout << result.errorMessage << std::endl;
                        continue;
                    }
                }
                else
                {
                    SEL_Interface::HaltAll();
                    Logger::error("Image acquisition timed out in refinement loop. Halting motion");
                    continue;
                }

                double x_err = object_x_px - resolution_x/2;
                if (std::abs(x_err) > tolerance) {
                    auto jog_direction = static_cast<SEL_Interface::Direction>((x_err > 0 ? 1 : -1) * camera_x_alignment * -1);
                    int jog_speed = std::abs(x_err) * refinement_speed_scale_factor;
                    jog_speed = (std::max)(jog_speed, 1);
                    SEL_Interface::Jog(SEL_Interface::Axis::X, jog_direction, jog_speed);
                }
                else {
                    SEL_Interface::Halt(SEL_Interface::Axis::X);
                }

                double y_err = object_y_px - resolution_y/2;
                if (std::abs(y_err) > tolerance) {
                    auto jog_direction = static_cast<SEL_Interface::Direction>((y_err > 0 ? 1 : -1) * camera_y_alignment * -1);
                    int jog_speed = std::abs(y_err) * refinement_speed_scale_factor;
                    jog_speed = (std::max)(jog_speed, 1);
                    SEL_Interface::Jog(SEL_Interface::Axis::Y, jog_direction, jog_speed);
                }
                else {
                    SEL_Interface::Halt(SEL_Interface::Axis::Y);
                }

                within_tolerance = std::abs(x_err) < tolerance && std::abs(y_err) < tolerance;
                DS.UpdateSEL();

                if (DS.x_axis.position_ > workspace_x || DS.y_axis.position_ > workspace_y) {
                    SEL_Interface::HaltAll();
                    throw RUNTIME_EXCEPTION("End effector detected leaving the workspace!");
                }
            }

            // Translate xy to place gripper directly over connector
            SEL_Interface::MoveToPosition({DS.x_axis.position_ + camera_to_gripper_x, DS.y_axis.position_ + camera_to_gripper_y});
            DS.waitForMotionComplete();
            // Z down to mate with connector
            SEL_Interface::SetOutputs({306, 305, 304, 303}, {1, 1, 1, 1}, DS.SEL_outputs);
            SEL_Interface::SetOutputs({302}, {1}, DS.SEL_outputs);
            SEL_Interface::SetOutputs({302}, {0}, DS.SEL_outputs);
            std::this_thread::sleep_for(std::chrono::milliseconds(15000)); //TODO: Replace with DS.waitForZMotionComplete();

            // Open gripper
            Gripper_Interface::Open();
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));

            // Z up
            SEL_Interface::SetOutputs({306, 305, 304, 303}, {0, 0, 0, 0}, DS.SEL_outputs);
            std::this_thread::sleep_for(std::chrono::milliseconds(15000));
            // DS.waitForZMotionComplete();
        }

        // Stop the image processing.
        recipe.Stop();
        SEL_Interface::MoveToPosition({0, 0});
    }

    catch (const GenericException& e)
    {
        // Error handling
        std::cerr << "An exception occurred (likely pylon) - " <<std::endl << e.GetDescription() <<std::endl;
        exitCode = 1;
        throw;
    }

    catch (const std::exception& e)
    {
        std::cout << "Exception caught :" << e.what() << std::endl;
        exitCode = 1;
        throw;
    }

    // Releases all pylon resources.
    PylonTerminate();

    return exitCode;
}