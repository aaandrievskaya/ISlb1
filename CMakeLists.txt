cmake_minimum_required(VERSION 3.22)
project(lb1)

set(CMAKE_CXX_STANDARD 14)
set(BUILD_SHARED_LIBS OFF)
set(CMAKE_EXE_LINKER_FLAGS "-static")

add_executable(lb1 main.cpp astar.cpp dijkstra.cpp input.cpp  structures.h astar.h dijkstra.h input.h PathPlanning.pro)
