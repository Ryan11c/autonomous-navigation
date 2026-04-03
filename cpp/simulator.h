#pragma once

#include <memory>
#include <string>
#include <vector>

#include "planner.h"
#include "robot.h"

struct Metrics {
    bool reachedGoal = false;
    int steps = 0;
    int replans = 0;
    int observedCells = 0;
    int finalPathLength = 0;
};

class Simulator {
public:
    bool initialize(const std::string& mapPath);
    Metrics run(int frameDelayMs = 200);
    bool writeMetrics(const std::string& outputPath, const Metrics& metrics) const;

private:
    std::vector<Prediction> classifyObservations(const std::vector<Observation>& observations) const;
    std::vector<Prediction> runPythonClassifier(const std::vector<Observation>& observations) const;
    static std::string pythonCommand();
    static std::string quoteForShell(const std::string& value);

    GridMap map_;
    Planner planner_;
    std::unique_ptr<Robot> robot_;
};
