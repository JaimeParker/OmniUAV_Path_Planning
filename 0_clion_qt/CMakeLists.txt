cmake_minimum_required(VERSION 3.20)
project(0_clion_qt)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)


find_package(Qt5 COMPONENTS
        Core
        Gui
        Widgets
        REQUIRED)

add_executable(0_clion_qt main.cpp draw_grid.cpp draw_grid.h)
target_link_libraries(0_clion_qt
        Qt5::Core
        Qt5::Gui
        Qt5::Widgets
        )

