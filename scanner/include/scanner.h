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

// Namespaces for using pylon objects
using namespace Pylon;
using namespace Pylon::DataProcessing;

typedef std::vector<std::vector<double>> Path;

Path buildScanPath (double x_length, double y_length, double width) {
    Path path;
    int num_passes = std::ceil(x_length / width) + 1;
    for (int i = 0; i < num_passes*2; ++i) {
        double x_coordinate = (std::min)(width * (i/2), x_length);
        double y_coordinate = y_length * (((i+1)/2) % 2);
        path.push_back({x_coordinate, y_coordinate});
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
    if (!resultCollector.GetWaitObject().Wait(75)) {// Blocks until image received, wait is ms
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

    Logger::info("px: (" + std::to_string(result.positions_px[0].X) + ", " + std::to_string(result.positions_px[0].Y) + ")");
    Logger::info("m : (" + std::to_string(result.positions_m[0].X) + ", " + std::to_string(result.positions_m[0].Y) + ")");
    return true;
}

// TODO: Enforce point typing
class MovingAverage {
public:    
    MovingAverage(size_t range = 10) { 
        this->range = range;
    }

    void Add(std::vector<double> measurement) {
        if (this->measurements.size() == this->range) {
            this->measurements.erase(this->measurements.begin());
        }
        this->measurements.push_back(measurement);
    }

    std::vector<double> Average() {
        auto num = this->measurements.size();
        if (num < 1) {
            throw std::runtime_error("Cannot get average, Moving Average has no measurements yet");
        }
        std::vector<double> average;
        for (auto measurement : this->measurements) {
            average[0] += measurement[0];
            average[1] += measurement[1];
        }
        average[0] /= num;
        average[1] /= num;
        return average;
    }

    std::vector<double> Latest() {
        if (this->measurements.size() < 1) {
            throw std::runtime_error("Cannot get latest, Moving Average has no measurements yet");
        }
        return this->measurements.back();
    }

private:
    size_t range;
    std::vector<std::vector<double>> measurements;
};

#endif // SCANNER_H