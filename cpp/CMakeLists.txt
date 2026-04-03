cmake_minimum_required(VERSION 3.16)
project(robot_nav_mvp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(robot_nav
    main.cpp
    grid_map.cpp
    robot.cpp
    planner.cpp
    simulator.cpp
)
