#pragma once

#include <string>
#include <utility>
#include <vector>

#include "grid_map.h"
#include "planner.h"

struct Observation {
    int x;
    int y;
    std::vector<double> features;
    CellType truth;
};

struct Prediction {
    std::pair<int, int> point;
    CellType type;
    double confidence;
    std::string backend;
};

class Robot {
public:
    explicit Robot(GridMap& map);

    using Point = std::pair<int, int>;

    std::vector<Observation> sense(int radius = 1) const;
    void updateBeliefMap(const std::vector<Prediction>& predictions);
    void planPath(const Planner& planner);
    bool pathEmpty() const;
    bool pathBlocked() const;
    bool moveOneStep();
    bool atGoal() const;

    int x() const;
    int y() const;
    Point goal() const;
    const std::vector<Point>& currentPath() const;
    std::size_t currentPathLength() const;

private:
    GridMap& map_;
    int x_;
    int y_;
    Point goal_;
    std::vector<Point> path_;
};
