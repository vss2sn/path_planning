# Path Planning #

### This repository contains path planning algorithms in C++. ###

#### Algorithms: ####
1. Dijkstra's algorithm for grid based search.
2. AStar (A*) algorithm for grid based search.
3. RRT algorithm for grid based search.
4. RRTStar (RRT*) algorithm for grid based search.
5. DStarLite (D* Lite) algorithm for grid based search.

#### Notes: ####
1. utils.cpp built as library and used in every separate file.
2. Setting the CMake option BUILD_INDIVIDUAL allows building of each .cpp separately (except main.cpp), allowing easy testing. Setting it to OFF allows use of all base classes and algorithms in main.cpp.
3. D* Lite can be run live with random obstacle creation using the RunDStarLite function.
4. To create documentation using Doxygen run `doxygen` in the docs folder. Config files added to generate documentation with the READTHEDOCs theme. To do so, make sure you have the extensions `breathe`, `exhale`, `m2r` and  `sphinx_rtd_theme` installed (using pip). Run `make html` in the docs folder after running `doxygen`.

#### Notes on test: ####
1. Unit test framework set up to set algorithms under different grids. This section uses Google Test.
2. CMake option TEST allows building tests when set when BUILD_INDIVIDUAL is set OFF.
3. Tests set to run after main file built.
4. Files named grid#.cpp are specific grids to check for correctness under certain conditions. gridr.cpp generates a random grid and checks whether Dijkstra, A* and D* Lite (without new obstacles) generate the same cost from start to end.

#### Notes on implementations: ####
1. RRT stops as soon as goal is found. It is connects new points to the nearest point, not accounting for total cost to reach that point. In contrast RRT\* chooses to connect to a new node to the node that allows the new node to have the minimum cost. RRT\* also rewires the preexisting nodes to the new node if that path allows for a lower cost for the preexisting node.
2. Acceptable motions can be modified in the GetMotion function in utils.cpp.
3. A* and D* Lite use Manhattan distance (L1) as their heuristic (change to L2 if adding diagonal moves to the GetMotion function). D* also uses the same in its C function.
4. D* Lite implemented based on based on Sven Koenig's & Maxim Likhachev's paper.
5. For the live run of D* Lite, obstacles are detected on the current path of the bot with a probability  of 1/n, n being the number of rows/columns in the grid.
6. To specify your own grid, set n to number of rows, created the 2D vector, setting 1 for obstacles and 0 elsewhere, and comment out the MakeGrid function.

#### Notes on CMake Options: ####
1. To run each algorithm independently, set BUILD_INDIVIDUAL to ON (Executables created: `dijkstra`, `a_star`, etc). If you want to run all of them on the same grid, set it to OFF (Executable created: `main`).
2. To run tests, set BUILD_INDIVIDUAL to OFF and TEST to ON.

#### TODOs: ####
1. Next algorithm to be implemented: a Voronoi cell based planner.
2. Alterations for moving node variables into private namespace.
3. Prune merged branches.
4. Cleanup and refactor test section.
5. Use hash to check if node in list for D* Lite (can be applied to others if required)
6. Add overview, pseudo codes and references to documentation.

#### Consider: ####
1. Adding namespace to each header file.
2. Inheriting node class into each file that requires a modified node (such as A* with heuristic cost, etc).
3. Moving TODOs to another file
