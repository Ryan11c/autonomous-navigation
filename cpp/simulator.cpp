#include "simulator.h"

#include <cstdlib>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <thread>

bool Simulator::initialize(const std::string& mapPath) {
    if (!map_.loadFromFile(mapPath)) {
        return false;
    }
    robot_ = std::make_unique<Robot>(map_);
    return true;
}

std::vector<Prediction> Simulator::classifyObservations(const std::vector<Observation>& observations) const {
    std::vector<Prediction> predictions;
    predictions.reserve(observations.size());
    auto pythonPredictions = runPythonClassifier(observations);

    if (!pythonPredictions.empty()) {
        return pythonPredictions;
    }

    for (const auto& observation : observations) {
        const double obstacleScore =
            0.65 * observation.features[0] +
            0.25 * observation.features[1] +
            0.10 * observation.features[3];

        predictions.push_back({
            {observation.x, observation.y},
            obstacleScore >= 0.5 ? CellType::Obstacle : CellType::Free,
            std::max(0.55, std::min(0.95, std::abs(obstacleScore - 0.5) + 0.5)),
            "cpp_heuristic"
        });
    }

    return predictions;
}

std::vector<Prediction> Simulator::runPythonClassifier(const std::vector<Observation>& observations) const {
    std::vector<Prediction> predictions;
    if (observations.empty()) {
        return predictions;
    }

    const std::string inputPath = "../results/ml_requests.csv";
    const std::string outputPath = "../results/ml_predictions.csv";

    {
        std::ofstream requestFile(inputPath);
        if (!requestFile) {
            return predictions;
        }

        requestFile << "x,y,intensity,local_density,normalized_distance,boundary_flag\n";
        requestFile << std::fixed << std::setprecision(6);
        for (const auto& observation : observations) {
            requestFile
                << observation.x << ','
                << observation.y << ','
                << observation.features[0] << ','
                << observation.features[1] << ','
                << observation.features[2] << ','
                << observation.features[3] << '\n';
        }
    }

    std::ostringstream command;
    command
        << pythonCommand() << ' ' << quoteForShell("../ml/infer.py")
        << " --input " << quoteForShell(inputPath)
        << " --output " << quoteForShell(outputPath);

    const int exitCode = std::system(command.str().c_str());
    if (exitCode != 0) {
        return predictions;
    }

    std::ifstream predictionFile(outputPath);
    if (!predictionFile) {
        return predictions;
    }

    std::string line;
    std::getline(predictionFile, line);
    while (std::getline(predictionFile, line)) {
        if (line.empty()) {
            continue;
        }

        std::stringstream row(line);
        std::string token;
        std::vector<std::string> fields;
        while (std::getline(row, token, ',')) {
            fields.push_back(token);
        }
        if (fields.size() < 5) {
            continue;
        }

        CellType type = fields[2] == "obstacle" ? CellType::Obstacle : CellType::Free;
        predictions.push_back({
            {std::stoi(fields[0]), std::stoi(fields[1])},
            type,
            std::stod(fields[3]),
            fields[4]
        });
    }

    return predictions;
}

std::string Simulator::pythonCommand() {
    const char* configured = std::getenv("ROBOT_NAV_PYTHON");
    if (configured != nullptr && configured[0] != '\0') {
        return quoteForShell(configured);
    }
    return "python";
}

std::string Simulator::quoteForShell(const std::string& value) {
    return "\"" + value + "\"";
}

Metrics Simulator::run(int frameDelayMs) {
    Metrics metrics;
    if (!robot_) {
        return metrics;
    }

    const int maxSteps = 500;
    const auto frameDelay = std::chrono::milliseconds(std::max(0, frameDelayMs));
    while (!robot_->atGoal() && metrics.steps < maxSteps) {
        auto observations = robot_->sense(1);
        metrics.observedCells += static_cast<int>(observations.size());

        auto predictions = classifyObservations(observations);
        robot_->updateBeliefMap(predictions);

        if (robot_->pathEmpty() || robot_->pathBlocked()) {
            robot_->planPath(planner_);
            ++metrics.replans;
        }

        if (robot_->pathEmpty()) {
            break;
        }

        if (!robot_->moveOneStep()) {
            robot_->planPath(planner_);
            ++metrics.replans;
            continue;
        }

        ++metrics.steps;
        if (frameDelayMs > 0) {
            std::system("cls");
        }
        map_.printBelief(robot_->x(), robot_->y(), robot_->currentPath());
        if (frameDelayMs > 0) {
            std::this_thread::sleep_for(frameDelay);
        }
    }

    metrics.reachedGoal = robot_->atGoal();
    metrics.finalPathLength = static_cast<int>(robot_->currentPathLength());
    return metrics;
}

bool Simulator::writeMetrics(const std::string& outputPath, const Metrics& metrics) const {
    std::ofstream output(outputPath);
    if (!output) {
        return false;
    }

    output << "reached_goal=" << (metrics.reachedGoal ? 1 : 0) << '\n';
    output << "steps=" << metrics.steps << '\n';
    output << "replans=" << metrics.replans << '\n';
    output << "observed_cells=" << metrics.observedCells << '\n';
    output << "final_path_length=" << metrics.finalPathLength << '\n';
    return true;
}
