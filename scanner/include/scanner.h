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
#include "point.h"

// Namespaces for using pylon objects
using namespace Pylon;
using namespace Pylon::DataProcessing;

typedef std::vector<Point> Path;

Path buildScanPath (double x_length, double y_length, double width) {
    Path path;
    int num_passes = std::ceil(x_length / width) + 1;
    for (int i = 0; i < num_passes*2; ++i) {
        double x_coordinate = (std::min)(width * (i/2), x_length);
        double y_coordinate = y_length * (((i+1)/2) % 2);
        path.push_back(Point(x_coordinate, y_coordinate));
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

#endif // SCANNER_H