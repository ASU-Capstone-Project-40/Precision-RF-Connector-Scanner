#ifndef SCANNER_H
#define SCANNER_H

#include <WinSock2.h>
// Include files to use the pylon API.
#include <pylon/PylonIncludes.h>

// Extend the pylon API for using pylon data processing.
#include <pylondataprocessing/PylonDataProcessingIncludes.h>

#include <vector>
#include <list>
#include <algorithm>

#include "logging.h"
#include "xy.h"

// Namespaces for using pylon objects
using namespace Pylon;
using namespace Pylon::DataProcessing;

typedef std::vector<XY> Path;

Path buildScanPath (XY start, XY max, double width) {
    Path path {start};
    int num_passes = std::ceil((max.x - start.x) / width) + 1;
    for (int i = 0; i < num_passes*2; ++i) {
        double x_coordinate = (std::min)(width * (i/2) + start.x, max.x);
        double y_coordinate = max.y * (((i+1)/2) % 2);
        path.push_back(XY(x_coordinate, y_coordinate));
    }

    Logger::debug("Scan path:");
    for (auto& point : path) {
        Logger::debug(point.toString());
    }
    return path;
}

// Should be a singleton
class PylonRecipe {
public:
    XY alignment;

    PylonRecipe(const Pylon::String_t& recipePath, XY alignment)
        : resultCollector(), recipe(), alignment(alignment) {
        Logger::debug("Initializing new pylon recipe");
        PylonInitialize();  // Before using any pylon methods, the pylon runtime must be initialized.
        Logger::verbose("Loading recipe");

        recipe.Load(recipePath); // Load the recipe file.
        Logger::verbose("Allocating resources");

        recipe.PreAllocateResources(); // Now we allocate all resources we need. This includes the camera device if used in the recipe.
        Logger::verbose("Registering Outputs Observer");

        recipe.RegisterAllOutputsObserver(&resultCollector, RegistrationMode_Append); // This is where the output goes.
        Logger::verbose("Starting recipe");

        // Start the processing.
        recipe.Start();
        Logger::verbose("Successfully initialized pylon recipe");
    }

    void Load(const Pylon::String_t& recipePath) {
        Logger::debug("Initializing new pylon recipe");
        PylonInitialize();  // Before using any pylon methods, the pylon runtime must be initialized.
        
        Logger::verbose("Loading recipe");
        recipe.Load(recipePath); // Load the recipe file.
        
        Logger::verbose("Allocating resources");
        recipe.PreAllocateResources(); // Now we allocate all resources we need. This includes the camera device if used in the recipe.
        
        Logger::verbose("Registering Outputs Observer");
        recipe.RegisterAllOutputsObserver(&resultCollector, RegistrationMode_Append); // This is where the output goes.
        
        Logger::verbose("Starting recipe");
        recipe.Start();
    }

    bool Detect(ResultData& result) {
        if (!resultCollector.GetWaitObject().Wait(100)) {// Blocks until image received, wait is ms
            Logger::error("Scanner::detectObject: Camera data result timeout");
            return false;
        }

        resultCollector.GetResultData(result);

        if (result.hasError) {
            std::cout << "Scanner::detectObject: An error occurred while processing recipe: " << result.errorMessage << std::endl; // Todo: Make this work with Logger
            return false;
        }

        if (result.mobile_score.empty() && result.fixed_score.empty()) {
            Logger::verbose("Scanner::detectObject: No object detected...");
            return false;
        }
        
        Logger::info(
            !result.mobile_score.empty() ? "mobile connector detected!\n" : "" + 
            !result.fixed_score.empty() ? "fixed connector detected!" : ""
        );

        return true;
    }

    void Stop() {
        Logger::verbose("Stopping recipe.");
        recipe.Stop(); // Stop the image processing.
        Logger::verbose("Releasing pylon resources.");
        PylonTerminate(); // Releases all pylon resources
        Logger::debug("Recipe stopped. All pylon resources released.");
    }

private:
    RecipeOutputObserver resultCollector;
    CRecipe recipe;
};

std::pair<bool, bool> ScanForMobile (PylonRecipe& recipe, Path& path, int speed, XY& fixed_position) {
    // Begin scan
    Logger::debug("Entering mobile scan Loop");
    Path path_copy = path;
    bool fixed_detected = false;
    double fixed_error = (std::numeric_limits<double>::max)();
    for (size_t i = 0; i < path_copy.size(); ++i) {
        SEL_Interface::MoveToPosition(path_copy[i], speed);
        commander->UpdateSEL();

        while(commander->in_motion) { // Continously get camera data and check if move has completed
            // Get camera data
            ResultData result;
            if (!recipe.Detect(result)) {
                commander->UpdateSEL();
                continue;
            }

            // If mobile connector seen
            if (!result.mobile_score.empty()) {
                SEL_Interface::HaltAll();

                commander->waitForXYMotionComplete();

                ResultData new_result;
                if (recipe.Detect(new_result) && !new_result.mobile_score.empty()) {
                    return {true, fixed_detected};
                }

                SEL_Interface::MoveToPosition(path_copy[(std::max)(i-1, size_t(0))], (std::max)(int(speed/2), 1));
                commander->UpdateSEL();
                continue;
            }

            // if fixed connector seen
            if (!result.fixed_score.empty()) {
                commander->UpdateSEL();
                fixed_detected = true;
                double error = std::sqrt(std::pow(result.fixed_position[0].X, 2) + std::pow(result.fixed_position[0].Y, 2));
                if ( error < fixed_error) {
                    fixed_error = error;
                    fixed_position = commander->position;
                }
                continue;
            }

            // This should never be reached but if you somehow end up here, update the commander for good measure
            commander->UpdateSEL();
        }
    }

    return {false, fixed_detected};
}

bool ScanForFixed (PylonRecipe& recipe, Path path, int speed) {
    // Begin scan
    Logger::debug("Entering fixed scan Loop");
    for (size_t i = 0; i < path.size(); ++i) {
        SEL_Interface::MoveToPosition(path[i], speed);
        commander->UpdateSEL();

        while(commander->in_motion) { // Continously get camera data and check if move has completed
            // Get camera data
            ResultData result;
            if (!recipe.Detect(result)) {
                commander->UpdateSEL();
                continue;
            }

            // If fixed connector seen
            if (!result.fixed_score.empty()) {
                SEL_Interface::HaltAll();

                commander->waitForXYMotionComplete();

                ResultData new_result;
                if (recipe.Detect(new_result) && !new_result.fixed_score.empty()) {
                    return true;
                }

                SEL_Interface::MoveToPosition(path[(std::max)(i-1, size_t(0))], (std::max)(int(speed/2), 1));
                commander->UpdateSEL();
                continue;
            }

            commander->UpdateSEL();
        }
    }

    return false;
}

bool RefineToMobile(PylonRecipe& recipe, int speed, double tolerance, double scale_factor, XY alignment) {
    Logger::debug("Entering mobile refinement loop...");
    int detection_errors = 0;
    while (detection_errors <= 10) {
        ResultData result;
        if(!recipe.Detect(result)) {
            Logger::error("No mobile connector detected in refinement loop!");
            ++detection_errors;
            continue;
        }

        detection_errors = 0;

        if (!result.mobile_score.empty()) {
            commander->UpdateSEL();
            auto error = XY(result.mobile_position[0].X * alignment.x, result.mobile_position[0].Y * alignment.y) * 1000;

            Logger::info("Current Position: " + commander->position.toString());
            Logger::info("Detected Error: " + error.toString());

            if (error.magnitude() < tolerance) {
                Logger::info("Success! Total error " + std::to_string(error.magnitude()));
                return true;
            }

            XY target_position = commander->position - (error * scale_factor);
            Logger::info("Target position: " + target_position.toString());
            SEL_Interface::MoveToPosition(target_position, speed);
            commander->waitForXYMotionComplete();
        }
    }

    return false;
}

bool RefineToFixed(PylonRecipe& recipe, int speed, double tolerance, double scale_factor, XY alignment) {
    Logger::debug("Entering fixed refinement loop...");
    int detection_errors = 0;
    while (detection_errors <= 10) {
        ResultData result;
        if(!recipe.Detect(result)) {
            Logger::error("No fixed connector detected in refinement loop! (" + std::to_string(detection_errors) + ")" );
            ++detection_errors;
            continue;
        }

        detection_errors = 0;

        if (!result.fixed_score.empty()) {
            commander->UpdateSEL();
            auto error = XY(result.fixed_position[0].X * alignment.x, result.fixed_position[0].Y * alignment.y) * 1000;

            Logger::info("Current Position: " + commander->position.toString());
            Logger::info("Detected Error: " + error.toString());

            if (error.magnitude() < tolerance) {
                Logger::info("Success! Total error " + std::to_string(error.magnitude()));
                return true;
            }

            XY target_position = commander->position - error  * scale_factor;
            Logger::info("Target position: " + target_position.toString());
            SEL_Interface::MoveToPosition(target_position, speed);
            commander->waitForXYMotionComplete();
        }
    }

    return false;
}

#endif // SCANNER_H