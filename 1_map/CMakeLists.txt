cmake_minimum_required(VERSION 3.20)
project(1_map)

set(CMAKE_CXX_STANDARD 14)

#  OpenCV
FIND_PACKAGE(OpenCV REQUIRED)
INCLUDE_DIRECTORIES({OpenCV_INCLUDE_DIRS})

add_executable(1_map main.cpp ReadMap.cpp ReadMap.h)

# link
TARGET_LINK_LIBRARIES(1_map ${OpenCV_LIBS})
