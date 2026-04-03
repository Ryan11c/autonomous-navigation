#pragma once

enum class CellType {
    Unknown,
    Free,
    Obstacle
};

struct Cell {
    CellType type = CellType::Unknown;
    bool observed = false;
    double confidence = 0.0;
};
