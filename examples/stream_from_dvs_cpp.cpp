#include <iostream>
#include <unistd.h>
#include <roboflex_core/core_nodes/core_nodes.h>
#include <roboflex_dvs/dvs.h>
#include <roboflex_visualization/visualization.h>

int main() {

    auto sensor = roboflex::dvs::DVSSensor("/dev/dvs0");
    auto encoder = roboflex::dvs::DVSEncoder();
    auto imager = roboflex::dvs::DVSEigenToGrayScale();
    auto visualizer = roboflex::visualization::BlackAndWhiteTV(24.0, 480, 320, "image", {-1,-1}, false, false, "Event Cameras are Cool");

    sensor > encoder > imager > visualizer;

    sensor.start();
    imager.start();
    visualizer.start();

    sleep(500.0);

    sensor.stop();
    imager.stop();
    visualizer.stop();

    return 0;
}