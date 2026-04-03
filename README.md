# Robot Nav MVP

Minimal autonomous navigation MVP with:

- A true 2D world map
- A robot belief map with unknown cells
- Local sensing
- A* path planning on the belief map
- Replanning when new obstacles are discovered
- A more realistic Python ML path with a PyTorch classifier

## Structure

```text
robot_nav_mvp/
├── cpp/
├── ml/
├── maps/
└── results/
```

## Build

```powershell
cd C:\Users\ryan0\OneDrive\Documents\Playground\cpp-ml-python-project\robot_nav_mvp\cpp
g++ -std=c++17 -O2 main.cpp grid_map.cpp robot.cpp planner.cpp simulator.cpp -o robot_nav
```

Or with CMake:

```powershell
cd C:\Users\ryan0\OneDrive\Documents\Playground\cpp-ml-python-project\robot_nav_mvp\cpp
cmake -S . -B build
cmake --build build --config Release
```

## Run

```powershell
.\robot_nav
```

Pick a map explicitly:

```powershell
.\robot_nav ..\maps\map2.txt
```

Set animation delay in milliseconds:

```powershell
.\robot_nav ..\maps\map2.txt 500
.\robot_nav ..\maps\map2.txt 0
```

The default run loads `../maps/map1.txt`, executes the simulator, prints the belief map over time, and writes metrics to `../results/metrics.txt`.

## Notes

- Unknown cells are allowed in planning, but with a higher traversal cost.
- The robot now generates richer observation features: intensity, local obstacle density, normalized sensor distance, and boundary context.
- The C++ simulator writes sensed batches to CSV and calls the Python classifier at runtime.
- If `torch` and trained model files are available, Python uses a small PyTorch MLP.
- If `torch` is not installed yet, inference falls back to a heuristic classifier so the simulator still runs.
- `*` in the printed map marks the robot's current planned path overlay.

## Train The PyTorch Model

```powershell
cd C:\Users\ryan0\OneDrive\Documents\Playground\cpp-ml-python-project\robot_nav_mvp\ml
python train.py
```

This produces:

- `model.pt`
- `feature_stats.json`

If you want the C++ simulator to use a specific Python interpreter, set:

```powershell
$env:ROBOT_NAV_PYTHON="C:\Path\To\python.exe"
```
