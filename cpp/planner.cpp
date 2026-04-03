#include "planner.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <unordered_map>

namespace {
struct PointHash {
    std::size_t operator()(const std::pair<int, int>& p) const {
        return static_cast<std::size_t>(p.first * 73856093u) ^ static_cast<std::size_t>(p.second * 19349663u);
    }
};

struct Node {
    std::pair<int, int> point;
    double fScore;

    bool operator<(const Node& other) const {
        return fScore > other.fScore;
    }
};
}  // namespace

int Planner::heuristic(Point a, Point b) const {
    return std::abs(a.first - b.first) + std::abs(a.second - b.second);
}

double Planner::movementCost(const GridMap& map, int x, int y) const {
    const Cell& cell = map.beliefCell(x, y);
    if (!cell.observed) {
        return 3.0;
    }
    if (cell.type == CellType::Obstacle) {
        return std::numeric_limits<double>::infinity();
    }
    return 1.0 + (1.0 - cell.confidence);
}

std::vector<Planner::Point> Planner::plan(const GridMap& map, Point start, Point goal) const {
    std::priority_queue<Node> open;
    std::unordered_map<Point, Point, PointHash> cameFrom;
    std::unordered_map<Point, double, PointHash> gScore;

    open.push({start, static_cast<double>(heuristic(start, goal))});
    gScore[start] = 0.0;

    const std::vector<Point> directions{{1, 0}, {-1, 0}, {0, 1}, {0, -1}};

    while (!open.empty()) {
        Point current = open.top().point;
        open.pop();

        if (current == goal) {
            std::vector<Point> path;
            Point step = goal;
            path.push_back(step);
            while (step != start) {
                step = cameFrom[step];
                path.push_back(step);
            }
            std::reverse(path.begin(), path.end());
            if (!path.empty()) {
                path.erase(path.begin());
            }
            return path;
        }

        for (const auto& dir : directions) {
            Point next{current.first + dir.first, current.second + dir.second};
            if (!map.inBounds(next.first, next.second) || map.isKnownBlocked(next.first, next.second)) {
                continue;
            }

            double tentative = gScore[current] + movementCost(map, next.first, next.second);
            auto it = gScore.find(next);
            if (it == gScore.end() || tentative < it->second) {
                cameFrom[next] = current;
                gScore[next] = tentative;
                double fScore = tentative + heuristic(next, goal);
                open.push({next, fScore});
            }
        }
    }

    return {};
}
