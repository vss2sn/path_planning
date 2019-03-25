/**
* @file main.cpp
* @author vss2sn
* @brief Main file where all the algorithms can be used and tested.
*/

#include "dijkstra.h"
#include "a_star.h"
#include "rrt.h"
#include "rrt_star.h"
#include "d_star_lite.h"

/**
* @brief Script main function. Generates start and end nodes as well as grid, then creates the algorithm objects and calls the main algorithm functions.
* @return 0
*/
int main(){
  int n = 11;
  int num_points = n*n;

  int main_grid[n][n];
  int grid_space = n*n*sizeof(int);
  int grid[n][n];
  MakeGrid(grid, n);

  std::random_device rd; // obtain a random number from hardware
  std::mt19937 eng(rd()); // seed the generator
  std::uniform_int_distribution<int> distr(0,n-1); // define the range

  Node start(distr(eng),distr(eng),0,0,0,0);
  Node goal(distr(eng),distr(eng),0,0,0,0);

  start.id_ = start.x_ * n + start.y_;
  start.pid_ = start.x_ * n + start.y_;
  goal.id_ = goal.x_ * n + goal.y_;
  start.h_cost_ = abs(start.x_ - goal.x_) + abs(start.y_ - goal.y_);
  //Make sure start and goal are not obstacles and their ids are correctly assigned.
  grid[start.x_][start.y_] = 0;
  grid[goal.x_][goal.y_] = 0;
  PrintGrid(grid, n);

  // Store points after algorithm has run
  std::vector<Node> path_vector;
  memcpy(main_grid, grid, grid_space);
  double threshold = 2;
  int max_iter_x_factor = 20;

  // Resetting grid
  // Create object for the algorithm
  // Run algorithm
  // Print the final grid using the path_vector
  std::cout << "--------------------------------------------------------------" << std::endl;
  std::cout << "--------------------- ALGORITH: DIJKSTRA ---------------------" << std::endl;
  std::cout << "--------------------------------------------------------------" << std::endl;
  memcpy(grid, main_grid, grid_space);
  Dijkstra new_dijkstra;
  path_vector = new_dijkstra.dijkstra(grid, n, start, goal);
  PrintPath(path_vector, start, goal, grid, n);

  std::cout << "--------------------------------------------------------" << std::endl;
  std::cout << "--------------------- ALGORITH: A* ---------------------" << std::endl;
  std::cout << "--------------------------------------------------------" << std::endl;
  memcpy(grid, main_grid, grid_space);
  AStar new_a_star;
  path_vector = new_a_star.a_star(grid, n, start, goal);
  PrintPath(path_vector, start, goal, grid, n);

  std::cout << "---------------------------------------------------------" << std::endl;
  std::cout << "--------------------- ALGORITH: RRT ---------------------" << std::endl;
  std::cout << "---------------------------------------------------------" << std::endl;
  memcpy(grid, main_grid, grid_space);
  RRT new_rrt;
  path_vector = new_rrt.rrt(grid, n, start, goal, max_iter_x_factor, threshold);
  PrintPath(path_vector, start, goal, grid, n);

  std::cout << "----------------------------------------------------------" << std::endl;
  std::cout << "--------------------- ALGORITH: RRT* ---------------------" << std::endl;
  std::cout << "----------------------------------------------------------" << std::endl;
  memcpy(grid, main_grid, grid_space);
  RRTStar new_rrt_star;
  path_vector = new_rrt_star.rrt_star(grid, n, start, goal, max_iter_x_factor, threshold);
  PrintPath(path_vector, start, goal, grid, n);

  std::cout << "-------------------------------------------------------------" << std::endl;
  std::cout << "--------------------- ALGORITH: D* Lite ---------------------" << std::endl;
  std::cout << "-------------------------------------------------------------" << std::endl;
  memcpy(grid, main_grid, grid_space);
  DStarLite new_d_star_lite;
  path_vector = new_d_star_lite.d_star_lite(grid, n, start, goal);
  PrintPath(path_vector, start, goal, grid, n);

  std::cout << "----------------------------------------------------------------------" << std::endl;
  std::cout << "--------------------- ALGORITH: D* Lite Live Run ---------------------" << std::endl;
  std::cout << "----------------------------------------------------------------------" << std::endl;
  // NOTE: Make sure the function d_star_lite(grid, n, start, goal) is called
  // before calling RunDStarLite()
  new_d_star_lite.RunDStarLite(); // NOTE: Pass false to RunDStarLite if the
  // incremental updated position of the bot is not to be displayed as it moves
  // NOTE: D* Lite currently does not return a path vector as the
  // returned path vector changes every replan. However, the path traversed is
  // set as 3 in the grid and displayed. 4 displays current location of bot in
  // the live run

  return 0;
}
