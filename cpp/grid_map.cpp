#include "grid_map.h"

#include <fstream>
#include <iostream>
#include <unordered_set>

namespace {
struct PointHash {
    std::size_t operator()(const std::pair<int, int>& point) const {
        return static_cast<std::size_t>(point.first * 73856093u) ^
               static_cast<std::size_t>(point.second * 19349663u);
    }
};
}  // namespace

bool GridMap::loadFromFile(const std::string& path) {
    std::ifstream input(path);
    if (!input) {
        return false;
    }

    trueMap_.clear();
    std::string line;
    while (std::getline(input, line)) {
        if (!line.empty()) {
            trueMap_.push_back(line);
        }
    }

    if (trueMap_.empty()) {
        return false;
    }

    for (int y = 0; y < height(); ++y) {
        for (int x = 0; x < width(); ++x) {
            if (trueMap_[y][x] == 'R') {
                start_ = {x, y};
                trueMap_[y][x] = '.';
            } else if (trueMap_[y][x] == 'G') {
                goal_ = {x, y};
                trueMap_[y][x] = '.';
            }
        }
    }

    initializeBelief();
    return true;
}

int GridMap::width() const {
    return trueMap_.empty() ? 0 : static_cast<int>(trueMap_[0].size());
}

int GridMap::height() const {
    return static_cast<int>(trueMap_.size());
}

bool GridMap::inBounds(int x, int y) const {
    return x >= 0 && y >= 0 && x < width() && y < height();
}

CellType GridMap::trueCellType(int x, int y) const {
    if (!inBounds(x, y)) {
        return CellType::Obstacle;
    }
    return trueMap_[y][x] == 'X' ? CellType::Obstacle : CellType::Free;
}

const Cell& GridMap::beliefCell(int x, int y) const {
    return beliefMap_[y][x];
}

Cell& GridMap::beliefCell(int x, int y) {
    return beliefMap_[y][x];
}

void GridMap::initializeBelief() {
    beliefMap_.assign(height(), std::vector<Cell>(width(), Cell{}));
}

void GridMap::updateBeliefCell(int x, int y, CellType type, bool observed, double confidence) {
    if (!inBounds(x, y)) {
        return;
    }
    beliefMap_[y][x].type = type;
    beliefMap_[y][x].observed = observed;
    beliefMap_[y][x].confidence = confidence;
}

bool GridMap::isKnownBlocked(int x, int y) const {
    if (!inBounds(x, y)) {
        return true;
    }
    const Cell& cell = beliefCell(x, y);
    return cell.observed && cell.type == CellType::Obstacle;
}

bool GridMap::isKnownFree(int x, int y) const {
    if (!inBounds(x, y)) {
        return false;
    }
    const Cell& cell = beliefCell(x, y);
    return cell.observed && cell.type == CellType::Free;
}

std::pair<int, int> GridMap::start() const {
    return start_;
}

std::pair<int, int> GridMap::goal() const {
    return goal_;
}

void GridMap::printBelief(int robotX, int robotY, const std::vector<std::pair<int, int>>& path) const {
    std::unordered_set<std::pair<int, int>, PointHash> pathCells(path.begin(), path.end());
    for (int y = 0; y < height(); ++y) {
        for (int x = 0; x < width(); ++x) {
            if (x == robotX && y == robotY) {
                std::cout << 'R' << ' ';
            } else if (x == goal_.first && y == goal_.second) {
                std::cout << 'G' << ' ';
            } else if (pathCells.count({x, y}) > 0) {
                std::cout << '*' << ' ';
            } else {
                const Cell& cell = beliefCell(x, y);
                char c = '?';
                if (cell.observed) {
                    c = (cell.type == CellType::Obstacle) ? 'X' : '.';
                }
                std::cout << c << ' ';
            }
        }
        std::cout << '\n';
    }
    std::cout << '\n';
}
