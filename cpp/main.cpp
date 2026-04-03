#include <iostream>
#include <stdexcept>
#include <string>

#include "simulator.h"

int main(int argc, char* argv[]) {
    Simulator simulator;
    const std::string mapPath = argc > 1 ? argv[1] : "../maps/map2.txt";
    const std::string metricsPath = "../results/metrics.txt";
    int frameDelayMs = 200;

    if (argc > 2) {
        try {
            frameDelayMs = std::stoi(argv[2]);
        } catch (const std::exception&) {
            std::cerr << "Invalid frame delay: " << argv[2] << '\n';
            std::cerr << "Usage: .\\robot_nav [map_path] [frame_delay_ms]\n";
            return 1;
        }
    }

    if (!simulator.initialize(mapPath)) {
        std::cerr << "Failed to load map: " << mapPath << '\n';
        std::cerr << "Usage: .\\robot_nav [map_path] [frame_delay_ms]\n";
        return 1;
    }

    Metrics metrics = simulator.run(frameDelayMs);

    if (!simulator.writeMetrics(metricsPath, metrics)) {
        std::cerr << "Failed to write metrics: " << metricsPath << '\n';
        return 1;
    }

    std::cout << "Simulation complete.\n";
    std::cout << "Reached goal: " << (metrics.reachedGoal ? "yes" : "no") << '\n';
    std::cout << "Steps: " << metrics.steps << '\n';
    std::cout << "Replans: " << metrics.replans << '\n';
    std::cout << "Observed cells: " << metrics.observedCells << '\n';
    return 0;
}
