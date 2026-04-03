#pragma once

#include <string>
#include <utility>
#include <vector>

#include "cell.h"

class GridMap {
public:
    bool loadFromFile(const std::string& path);

    int width() const;
    int height() const;
    bool inBounds(int x, int y) const;

    CellType trueCellType(int x, int y) const;
    const Cell& beliefCell(int x, int y) const;
    Cell& beliefCell(int x, int y);

    void initializeBelief();
    void updateBeliefCell(int x, int y, CellType type, bool observed = true, double confidence = 1.0);
    bool isKnownBlocked(int x, int y) const;
    bool isKnownFree(int x, int y) const;

    std::pair<int, int> start() const;
    std::pair<int, int> goal() const;

    void printBelief(int robotX, int robotY, const std::vector<std::pair<int, int>>& path = {}) const;

private:
    std::vector<std::string> trueMap_;
    std::vector<std::vector<Cell>> beliefMap_;
    std::pair<int, int> start_{0, 0};
    std::pair<int, int> goal_{0, 0};
};
