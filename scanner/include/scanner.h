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
#include <thread>

#include "logging.h"
#include "xyz.h"

// Namespaces for using pylon objects
using namespace Pylon;
using namespace Pylon::DataProcessing;

typedef std::vector<XYZ> Path;

Path buildScanPath (double x_min, double x_max, double y_length, double width) {
    Path path;
    int num_passes = std::ceil((x_max - x_min) / width) + 1;
    for (int i = 0; i < num_passes*2; ++i) {
        double x_coordinate = (std::min)(width * (i/2) + x_min, x_max);
        double y_coordinate = y_length * (((i+1)/2) % 2);
        path.push_back(XYZ(x_coordinate, y_coordinate));
    }
    return path;
}

void wait(unsigned int ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

// Should be a singleton
class PylonRecipe {
public:
    XYZ alignment;

    PylonRecipe(const Pylon::String_t& recipePath, XYZ alignment)
        : resultCollector(), recipe(), alignment(alignment) {

        PylonInitialize();  // Before using any pylon methods, the pylon runtime must be initialized.
        recipe.Load(recipePath); // Load the recipe file.
        recipe.PreAllocateResources(); // Now we allocate all resources we need. This includes the camera device if used in the recipe.
        recipe.RegisterAllOutputsObserver(&resultCollector, RegistrationMode_Append); // This is where the output goes.
        // Start the processing.
        recipe.Start();
    }

    bool Detect(XYZ& position) {
    ResultData result;
    if (!resultCollector.GetWaitObject().Wait(100)) {// Blocks until image received, wait is ms
        Logger::error("Scanner::detectObject: Camera data result timeout");
        return false;
    }

    resultCollector.GetResultData(result);

    if (result.hasError) {
        std::cout << "Scanner::detectObject: An error occurred while processing recipe: " << result.errorMessage << std::endl; // Todo: Make this work with Logger
        return false;
    }

    if (result.scores.empty()) {
        Logger::verbose("Scanner::detectObject: No object detected...");
        return false;
    }

    position = XYZ(result.positions_m[0].X * alignment.x, result.positions_m[0].Y * alignment.y) * 1000;
    return true;
    }

    void Stop() {
        recipe.Stop(); // Stop the image processing.
        PylonTerminate(); // Releases all pylon resources
    }

private:
    RecipeOutputObserver resultCollector;
    CRecipe recipe;
};

bool Scan (PylonRecipe& recipe, Path path, int speed = 100) {
    // Begin scan
    Logger::debug("Entering Scan Loop");
    bool object_detected = false;
    for (size_t i = 0; i < path.size(); ++i) {
        SEL_Interface::MoveToPosition(path[i], speed);
        DS->UpdateSEL();

        while(DS->in_motion) { // Continously get camera data and check if move has completed
            DS->UpdateSEL();

            // Get camera data
            XYZ detected_position;
            if(!recipe.Detect(detected_position)) {
                continue;
            }
            
            SEL_Interface::HaltAll();

            DS->waitForMotionComplete();
            wait(2000);

            if (object_detected) {
                return true;
            }

            SEL_Interface::MoveToPosition(path[(std::max)(i-1, size_t(0))], (std::max)(int(speed/2), 1));
            DS->UpdateSEL();
            object_detected = true;
        }
    }
    return false;
}

bool Refine(PylonRecipe& recipe, int speed, double tolerance, double scale_factor) {
    bool within_tolerance = false;
    Logger::debug("Entering refinement loop...");

    while (!within_tolerance) {
        XYZ error;
        if(!recipe.Detect(error)) {
            Logger::error("No object detected in refinement loop");
            continue;
        }

        DS->UpdateSEL();

        XYZ target_position = DS->position - error * scale_factor;

        within_tolerance = error.magnitude() < tolerance;

        Logger::info("Current Position: " + DS->position.toString());
        Logger::info("Detected Error: " + error.toString());
        Logger::info("Target position: " + target_position.toString());

        if (within_tolerance) {
            return true;
        }
        
        SEL_Interface::MoveToPosition(target_position, speed);
        DS->waitForMotionComplete();
        wait(2000);
    }
    Logger::error("I don't think you should be seeing this message...");
    return false;
}

#endif // SCANNER_H