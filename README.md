# Path Planning #

#### This repository contains path planning algorithms in C++. ####
##### It currently includes an implementation of: #####
1. Dijkstra's algorithm for grid based search
2. A* (A star) algorithm for grid based search
3. RRT algorithm for grid based search
4. RRT* (RRT star) algorithm for grid based search

##### Notes: #####
1. utils.cpp built as library and used in every separate file.
2. Setting the CMake option BUILD_INDIVIDUAL allows building of each .cpp separately (except main.cpp), allowing easy testing. Setting it to OFF allows use of all base classes and algorithms in main.cpp.

##### Notes on implementations: #####
1. RRT stops as soon as goal is found. It is connects new points to the nearest point, not accounting for total cost to reach that point. In contrast RRT\* chooses to connect to a new node to the node that allows the new node to have the minimum cost. RRT\* also rewires the preexisting nodes to the new node if that path allows for a lower cost for the preexisting node.
2. A* uses Manhattan distance as its heuristic.

##### TODOs: #####
1. Next algorithms to be implemented include Lifelong A*, D\* lite and a Voronoi cell based planner
2. Include unit test section for testing different conditions/grids
3. Alterations for moving node variables into private namespace
4. Prune merged branches

##### Consider: #####
1. Adding namespace to each header file
2. Inheriting node class into each file that requires a modified node (such as A* with heuristic cost and D* with rhs, etc)
