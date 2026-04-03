#pragma once

#include <utility>
#include <vector>

#include "grid_map.h"

class Planner {
public:
    using Point = std::pair<int, int>;

    std::vector<Point> plan(const GridMap& map, Point start, Point goal) const;

private:
    int heuristic(Point a, Point b) const;
    double movementCost(const GridMap& map, int x, int y) const;
};

