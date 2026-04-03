#include "robot.h"

#include <algorithm>
#include <cmath>

namespace {
double clampValue(double value, double low, double high) {
    return std::max(low, std::min(high, value));
}
}  // namespace

Robot::Robot(GridMap& map) : map_(map) {
    auto start = map_.start();
    goal_ = map_.goal();
    x_ = start.first;
    y_ = start.second;
    map_.updateBeliefCell(x_, y_, CellType::Free, true);
    map_.updateBeliefCell(goal_.first, goal_.second, CellType::Free, true);
}

std::vector<Observation> Robot::sense(int radius) const {
    std::vector<Observation> observations;
    for (int dy = -radius; dy <= radius; ++dy) {
        for (int dx = -radius; dx <= radius; ++dx) {
            int nx = x_ + dx;
            int ny = y_ + dy;
            if (!map_.inBounds(nx, ny)) {
                continue;
            }

            CellType truth = map_.trueCellType(nx, ny);
            const double baseReflectance = truth == CellType::Obstacle ? 0.82 : 0.22;
            const double deterministicNoise = ((nx * 17 + ny * 31 + x_ * 13 + y_ * 7) % 11 - 5) * 0.025;
            const double intensity = clampValue(baseReflectance + deterministicNoise, 0.0, 1.0);

            int obstacleCount = 0;
            int neighborCount = 0;
            for (int oy = -1; oy <= 1; ++oy) {
                for (int ox = -1; ox <= 1; ++ox) {
                    int px = nx + ox;
                    int py = ny + oy;
                    if (!map_.inBounds(px, py)) {
                        continue;
                    }
                    ++neighborCount;
                    if (map_.trueCellType(px, py) == CellType::Obstacle) {
                        ++obstacleCount;
                    }
                }
            }

            const double localDensity = neighborCount == 0 ? 0.0 : static_cast<double>(obstacleCount) / neighborCount;
            const double distance = static_cast<double>(std::abs(nx - x_) + std::abs(ny - y_));
            const double normalizedDistance = radius == 0 ? 0.0 : distance / (2.0 * radius);
            const double boundaryFlag =
                (nx == 0 || ny == 0 || nx == map_.width() - 1 || ny == map_.height() - 1) ? 1.0 : 0.0;

            observations.push_back({nx, ny, {intensity, localDensity, normalizedDistance, boundaryFlag}, truth});
        }
    }
    return observations;
}

void Robot::updateBeliefMap(const std::vector<Prediction>& predictions) {
    for (const auto& prediction : predictions) {
        map_.updateBeliefCell(
            prediction.point.first,
            prediction.point.second,
            prediction.type,
            true,
            prediction.confidence);
    }
}

void Robot::planPath(const Planner& planner) {
    path_ = planner.plan(map_, {x_, y_}, goal_);
}

bool Robot::pathEmpty() const {
    return path_.empty();
}

bool Robot::pathBlocked() const {
    for (const auto& step : path_) {
        if (map_.isKnownBlocked(step.first, step.second)) {
            return true;
        }
    }
    return false;
}

bool Robot::moveOneStep() {
    if (path_.empty()) {
        return false;
    }

    Point next = path_.front();
    path_.erase(path_.begin());

    if (map_.trueCellType(next.first, next.second) == CellType::Obstacle) {
        map_.updateBeliefCell(next.first, next.second, CellType::Obstacle, true);
        return false;
    }

    x_ = next.first;
    y_ = next.second;
    map_.updateBeliefCell(x_, y_, CellType::Free, true);
    return true;
}

bool Robot::atGoal() const {
    return x_ == goal_.first && y_ == goal_.second;
}

int Robot::x() const {
    return x_;
}

int Robot::y() const {
    return y_;
}

Robot::Point Robot::goal() const {
    return goal_;
}

const std::vector<Robot::Point>& Robot::currentPath() const {
    return path_;
}

std::size_t Robot::currentPathLength() const {
    return path_.size();
}
