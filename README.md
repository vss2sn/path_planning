**Path Planning**

This repository will contain various path planning algorithms in C++.
It currently includes an implementation of:
1. Dijkstra's algorithm for grid based search
2. A-star algorithm for grid based search
3. RRT algorithm for grid based search
4. RRT* (RRT star) algorithm for grid based search

Notes:
1. utils.cpp built as library and used in every separate file.
2. Setting the CMake option BUILD_INDIVIDUAL allows building of each .cpp separately (except main.cpp), allowing easy testing. Setting it to OFF allows use of all base classes and algorithms in main.cpp.
