// Include files to use the pylon API.
#include <pylon/PylonIncludes.h>

// Extend the pylon API for using pylon data processing.
#include <pylondataprocessing/PylonDataProcessingIncludes.h>

// The sample uses the std::list.
#include <list>

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

// Namespace for using cout
using namespace std;

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

    // Workspace parameters
    double workspace_x = 300.0; // mm
    double workspace_y = 600.0; // mm

    // Scanning parameters (mm)
    double scan_width = 50.0; // mm
    double scan_speed = 0.0; // mm/s TODO: Implement this

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

    // Initialize the gripper
    Gripper_Interface::Initialize();
    Gripper_Interface::MoveTo();

    // Before using any pylon methods, the pylon runtime must be initialized.
    PylonInitialize();

    try
    {
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
            double x_coordinate = std::min(scan_width * (i/2), workspace_x);
            double y_coordinate = workspace_y * (((i+1)/2) % 2);
            SEL_Interface::MoveToPosition({x_coordinate, workspace_y});

            DS.UpdateSEL();
            // Continously get camera data and check if move has completed
            while(DS.x_axis.in_motion_ || DS.y_axis.in_motion_) {
                DS.UpdateSEL();
                // Get camera data
                if (resultCollector.GetWaitObject().Wait(5000)) // Reduce this timeout?
                {
                    ResultData result;
                    resultCollector.GetResultData(result);
                    if (!result.hasError)
                    {
                        if (!result.scores.empty()) {
                            cout << "Object detected!" << endl;
                            cout << "Score: " << result.scores[0] << endl;
                            cout << "Position: " << result.positions_px[0].X << ", " << result.positions_px[0].Y << endl;

                            object_detected = true;
                            object_x_px = result.positions_px[0].X;
                            object_y_px = result.positions_px[0].Y;

                            // Calling halt instead of breaking here allows the camera to continue to take images
                            // while the axis drifts, ensuring that the last pictures is taken while the camera
                            // is stable
                            SEL_Interface::HaltAll();
                        }
                    }
                    else
                    {
                        cout << "An error occurred during processing recipe image_recon: " << result.errorMessage << endl;
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
        // At this point, scan is complete.

        if (object_detected) {
            // Refine position to place camera directly over connector
            // Translate xy to place gripper directly over connector
            // Z down to mate with connector
            // Open gripper
            Gripper_Interface::Open();
            // Z up
        }

        // Stop the image processing.
        recipe.Stop();
        SEL_Interface::MoveToPosition({0, 0});

    }
    catch (const GenericException& e)
    {
        // Error handling
        cerr << "An exception occurred." << endl << e.GetDescription() << endl;
        exitCode = 1;
    }

    // Releases all pylon resources.
    PylonTerminate();

    return exitCode;
}