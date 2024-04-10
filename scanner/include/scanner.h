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

/**
 * Checks if an object is recognized in the camera image.
 * PylonInitialize() and recipe.Start() must have been called prior.
 * \param resultCollector a RecipeOutputObserver tied to the recipe using
 * recipe.RegisterAllOutputsObserver(&resultCollector, RegistrationMode_Append);
 * \param result a default-initialized ResultData object to be filled with data if an object is recognized
 * \returns true if an object is recognized, false if not.
*/
bool detectObject(RecipeOutputObserver& resultCollector, ResultData& result) {
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

    return true;
}

void wait(unsigned int ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

// class Pylon {
// public:
//     Pylon() {
//         // Before using any pylon methods, the pylon runtime must be initialized.
//         PylonInitialize();

//         RecipeOutputObserver resultCollector;
//         CRecipe recipe; // Create a recipe object representing a recipe file created using the pylon Viewer Workbench.
//         recipe.Load(PYLON_RECIPE); // Load the recipe file.
//         recipe.PreAllocateResources(); // Now we allocate all resources we need. This includes the camera device if used in the recipe.
//         recipe.RegisterAllOutputsObserver(&resultCollector, RegistrationMode_Append); // This is where the output goes.

//         // Start the processing.
//         recipe.Start();
//     }

//     bool Detect() {
//     ResultData result;
//     if (!resultCollector.GetWaitObject().Wait(100)) {// Blocks until image received, wait is ms
//         Logger::error("Scanner::detectObject: Camera data result timeout");
//         return false;
//     }

//     resultCollector.GetResultData(result);

//     if (result.hasError) {
//         std::cout << "Scanner::detectObject: An error occurred while processing recipe: " << result.errorMessage << std::endl; // Todo: Make this work with Logger
//         return false;
//     }

//     if (result.scores.empty()) {
//         Logger::verbose("Scanner::detectObject: No object detected...");
//         return false;
//     }

//     return true;
//     }

//     void Stop() {
//         // Releases all pylon resources.
//         PylonTerminate();
//     }
// }

#endif // SCANNER_H