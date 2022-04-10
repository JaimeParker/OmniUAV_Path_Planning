cmake_minimum_required(VERSION 3.21)
project(5_final)

set(CMAKE_CXX_STANDARD 14)

#  OpenCV
FIND_PACKAGE(OpenCV REQUIRED)
INCLUDE_DIRECTORIES({OpenCV_INCLUDE_DIRS})

add_executable(5_final main.cpp AStarSearch.cpp AStarSearch.h readMap.cpp readMap.h testRadar.cpp testRadar.h Solution.cpp Solution.h)

# link
TARGET_LINK_LIBRARIES(5_final ${OpenCV_LIBS})