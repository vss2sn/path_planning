# Path Planning #

### This repository contains path planning algorithms in C++. ###

[![Build Status](https://travis-ci.com/vss2sn/path_planning.svg?branch=feature%2Ftravis_ci)](https://travis-ci.com/vss2sn/path_planning)

<a name="algorithms"></a>
#### Algorithms: ####
1. Dijkstra's algorithm for grid based search.
2. AStar (A*) algorithm for grid based search.
3. Lifelong Planning AStar (LPA*) algorithm for grid based search.
4. DStarLite (D* Lite) algorithm for grid based search.
5. RRT algorithm for grid based search.
6. RRTStar (RRT*) algorithm for grid based search.
7. Ant Colony Optimization algorithm (ACO) for grid based search.
9. Genetic algorithm (GA) for grid based search.

<a name="instructions"></a>
#### To build and run: ####
     git clone https://github.com/vss2sn/path_planning.git  
     cd path_planning  
     mkdir build  
     cd build  
     cmake .. && make -j4  
     ./main  

<a name="toc"></a>
#### Table of contents ####
- [Algorithms](#algorithms)
- [Instructions](#instructions)
- [Table of contents](#toc)
- [Notes](#notes)
- [Notes on tests](#notes_on_tests)
- [Notes on implementations](#notes_on_implementations)
- [Notes on CMake Options](#notes_on_cmake_options)
- [Notes on Travis CI Integration](#notes_on_travis_ci_integration)
- [TODOs](#todos)
- [Consider](#consider)

<a name="notes"></a>
#### Notes: ####
1. `main` creates a grid of a given size n, with any point set as an obstacle with a probability of 1/n. It then runs all the algorithms in the repository on the given grid.
2. Setting the CMake option `BUILD_INDIVIDUAL` to `ON` allows building of each .cpp separately (except main.cpp), facilitating easy testing. Setting `BUILD_INDIVIDUAL` to `OFF` allows use of all base classes and algorithms in main.cpp.
3. utils.cpp built as library and used in every separate file.
4. [Documentation](https://vss2sn.github.io/path_planning/) moved to GitHub pages. It has been created using [Doxygen](http://www.doxygen.nl/), and pip3 packages [Sphinx](http://www.sphinx-doc.org/en/master/) (sphinx==1.8.3), [Breathe](https://github.com/michaeljones/breathe) (breathe==4.12.0), [Exhale](https://github.com/svenevs/exhale) (exhale==0.2.2) and [Read the Docs Sphinx Theme](https://github.com/rtfd/sphinx_rtd_theme) (sphinx_rtd_theme==0.4.3).

<a name="notes_on_tests"></a>
#### Notes on test: ####
1. Unit test framework set up to set algorithms under different grids. This section uses Google Test.
2. CMake option TEST allows building tests when set when `BUILD_INDIVIDUAL` is set `OFF`.
3. Tests set to run after main file built.
4. Files named grid#.cpp are specific grids to check for correctness under certain conditions. gridr.cpp generates a random grid and checks whether Dijkstra, A\* and D\* Lite (without new obstacles) generate the same cost from start to end.
5. Given the random nature of RRT, no set has been set up for it yet.
6. Due to the nature of Ant Colony Optimization and accounting for the hyper parameters, the tests are run with a 20% margin above the optimal solution.

<a name="notes_on_implementations"></a>
#### Notes on implementations: ####
1. RRT stops as soon as goal is found. It is connects new points to the nearest point, not accounting for total cost to reach that point. In contrast RRT\* chooses to connect to a new node to the node that allows the new node to have the minimum cost. RRT\* also rewires the preexisting nodes to the new node if that path allows for a lower cost for the preexisting node.
2. Acceptable motions can be modified in the GetMotion function in utils.cpp.
3. A\* and D\* Lite use Manhattan distance (L1) as their heuristic (change to L2 if adding diagonal moves to the GetMotion function). D* Lite also uses the same in its C function.
4. D* Lite can be run live with random obstacle creation using the RunDStarLite function. For the live run of D* Lite, obstacles are detected on the current path of the bot with a probability  of 1/n, n being the number of rows/columns in the grid. D* Lite is implemented based on Sven Koenig's & Maxim Likhachev's paper.
5. To specify your own grid, set n to number of rows, created the 2D vector, setting 1 for obstacles and 0 elsewhere, and comment out the MakeGrid function.
6. The LPA* algorithm is implemented to run `max_iter_` number of times with default value `n`. Obstacles are created on the current path of the bot with a probability  of 1/n, n being the number of rows/columns in the grid, at a random point along the path. It can be run with `max_iter_` set to `0` if continuous replanning is to be disabled.

<a name="notes_on_cmake_options"></a>
#### Notes on CMake Options: ####
1. To run each algorithm independently, set `BUILD_INDIVIDUAL` to `ON` (Executables created: `dijkstra`, `a_star`, etc). If you want to run all of them on the same grid, set `BUILD_INDIVIDUAL` to `OFF` (Executable created: `main`).
2. To run tests, set `BUILD_INDIVIDUAL` to `OFF` and TEST to `ON`.

<a name="notes_on_travis_ci_integration"></a>
#### Notes on Travis CI integration: ####
1. CI integrated in the feature/travis_ci branch, as integrating googletest with travis_ci, along with the test setup that runs automatically after every local build of `main` requires changes to CMakeLists.txt and use of google test as a submodule. Current travis_ci use is aimed more at future proofing than anything else.

<a name="todos"></a>
#### TODOs: ####
1. Alterations for moving node variables into private namespace.
2. Prune merged branches.
3. Cleanup and refactor test section.
4. Use hash to check if node in list for D* Lite (can be applied to others if required)
5. Add test of probabilistic completeness for RRT.
6. Formalize pseudocode and add references for ACO.
7. Add ACO to test section

<a name="consider"></a>
#### Consider: ####
1. Adding namespace to each header file.
2. Inheriting node class into each file that requires a modified node (such as A* with heuristic cost, etc).
3. Moving TODOs to another file
