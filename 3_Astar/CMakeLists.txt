cmake_minimum_required(VERSION 3.21)
project(3_Astar)

set(CMAKE_CXX_STANDARD 14)

#  OpenCV
FIND_PACKAGE(OpenCV REQUIRED)
INCLUDE_DIRECTORIES({OpenCV_INCLUDE_DIRS})

add_executable(3_Astar main.cpp
        readMap.cpp readMap.h AStar.cpp AStar.h)

TARGET_LINK_LIBRARIES(3_Astar ${OpenCV_LIBS})
