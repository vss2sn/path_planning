/**
* @file main.cpp
* @author vss2sn
* @brief Main file where all the algorithms can be used and tested.
*/

#include "dijkstra.hpp"
#include "a_star.hpp"
#include "lpa_star.hpp"
#include "rrt.hpp"
#include "rrt_star.hpp"
#include "d_star_lite.hpp"
#include "ant_colony.hpp"
#include "genetic_algorithm.hpp"

int main(){
  int n = 21;

  std::vector<std::vector<int>> grid(n);
  std::vector<int> tmp(n);
  for (int i = 0; i < n; i++){
    grid[i] = tmp;
  }
  MakeGrid(grid);
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
  PrintGrid(grid);

  // Store points after algorithm has run
  std::vector<Node> path_vector;
  std::vector<std::vector<int>> main_grid = grid;

  // Variables for RRT and RRTStar
  double threshold = 2;
  int max_iter_x_factor = 20;

  // Variables for Ant Colony Optimization
  int n_ants = 10, iterations = 50;
  float alpha = 1, beta =0.7, evap_rate = 0.3, Q = 10;

  // Resetting grid
  // Create object for the algorithm
  // Run algorithm
  // Print the final grid using the path_vector
  std::cout << "--------------------------------------------------------------" << std::endl;
  std::cout << "--------------------- ALGORITH: DIJKSTRA ---------------------" << std::endl;
  std::cout << "--------------------------------------------------------------" << std::endl;
  grid = main_grid;
  Dijkstra new_dijkstra;
  path_vector = new_dijkstra.dijkstra(grid, start, goal);
  PrintPath(path_vector, start, goal, grid);

  std::cout << "--------------------------------------------------------" << std::endl;
  std::cout << "--------------------- ALGORITH: A* ---------------------" << std::endl;
  std::cout << "--------------------------------------------------------" << std::endl;
  grid = main_grid;
  AStar new_a_star;
  path_vector = new_a_star.a_star(grid, start, goal);
  PrintPath(path_vector, start, goal, grid);

  std::cout << "--------------------------------------------------------------------------" << std::endl;
  std::cout << "--------------------- ALGORITH: Lifeling Planning A* ---------------------" << std::endl;
  std::cout << "--------------------------------------------------------------------------" << std::endl;
  grid = main_grid;
  LPAStar new_lpa_star;
  path_vector.clear();
  path_vector = new_lpa_star.lpa_star(grid, start, goal, n);
  PrintPath(path_vector, start, goal, grid);

  std::cout << "---------------------------------------------------------" << std::endl;
  std::cout << "--------------------- ALGORITH: RRT ---------------------" << std::endl;
  std::cout << "---------------------------------------------------------" << std::endl;
  grid = main_grid;
  RRT new_rrt;
  path_vector = new_rrt.rrt(grid, start, goal, max_iter_x_factor, threshold);
  PrintPath(path_vector, start, goal, grid);

  std::cout << "----------------------------------------------------------" << std::endl;
  std::cout << "--------------------- ALGORITH: RRT* ---------------------" << std::endl;
  std::cout << "----------------------------------------------------------" << std::endl;
  grid = main_grid;
  RRTStar new_rrt_star;
  path_vector = new_rrt_star.rrt_star(grid, start, goal, max_iter_x_factor, threshold);
  PrintPath(path_vector, start, goal, grid);

  std::cout << "-------------------------------------------------------------" << std::endl;
  std::cout << "--------------------- ALGORITH: D* Lite ---------------------" << std::endl;
  std::cout << "-------------------------------------------------------------" << std::endl;
  grid = main_grid;
  DStarLite new_d_star_lite;
  path_vector = new_d_star_lite.d_star_lite(grid, start, goal);
  PrintPath(path_vector, start, goal, grid);

#ifdef DYNAMIC_ALGOS
  std::cout << "----------------------------------------------------------------------" << std::endl;
  std::cout << "--------------------- ALGORITH: D* Lite Live Run ---------------------" << std::endl;
  std::cout << "----------------------------------------------------------------------" << std::endl;
  // NOTE: Make sure the function d_star_lite(grid, start, goal) is called
  // before calling RunDStarLite()
  new_d_star_lite.RunDStarLite(); // NOTE: Pass false to RunDStarLite if the
  // incremental updated position of the bot is not to be displayed as it moves
  // NOTE: D* Lite currently does not return a path vector as the
  // returned path vector changes every replan. However, the path traversed is
  // set as 3 in the grid and displayed. 4 displays current location of bot in
  // the live run
#endif

  std::cout << "-----------------------------------------------------------------------------" << std::endl;
  std::cout << "--------------------- ALGORITH: Ant Colony Optimization ---------------------" << std::endl;
  std::cout << "-----------------------------------------------------------------------------" << std::endl;
  grid = main_grid;
  AntColony new_ant_colony(n_ants, alpha, beta, evap_rate, iterations, Q);
  path_vector = new_ant_colony.ant_colony(grid, start, goal);
  PrintPath(path_vector, start, goal, grid);

  std::cout << "-----------------------------------------------------------------------" << std::endl;
  std::cout << "--------------------- ALGORITH: Genetic Algorithm ---------------------" << std::endl;
  std::cout << "-----------------------------------------------------------------------" << std::endl;
  grid = main_grid;
  GeneticAlgorithm new_genetic_algorithm;
  path_vector = new_genetic_algorithm.genetic_algorithm(grid, start, goal, 2*start.h_cost_);
  PrintPathInOrder(path_vector, start, goal, grid);

  return 0;
}
