#ifndef SCANNER_H
#define SCANNER_H

#include <vector>

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

#endif // SCANNER_H