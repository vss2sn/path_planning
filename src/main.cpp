#include "dijkstra.h"
#include "a_star.h"
#include "rrt.h"
#include "rrt_star.h"
#include "d_star_lite.h"

int main(){
  int n = 8;
  int num_points = n*n;

  int main_grid[n][n];

  int grid_space = n*n*sizeof(int);
  int grid[n][n];
  MakeGrid(grid, n);
  PrintGrid(grid, n);

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
  RRT_Star new_rrt_star;
  path_vector = new_rrt_star.rrt_star(grid, n, start, goal, max_iter_x_factor, threshold);
  PrintPath(path_vector, start, goal, grid, n);

  std::cout << "-------------------------------------------------------------" << std::endl;
  std::cout << "--------------------- ALGORITH: D* Lite ---------------------" << std::endl;
  std::cout << "-------------------------------------------------------------" << std::endl;
  memcpy(grid, main_grid, grid_space);
  DStarLite new_d_star_lite;
  path_vector = new_d_star_lite.d_star_lite(grid, n, start, goal);
  PrintPath(path_vector, start, goal, grid, n);

  std::cout << "--------------------------------------------------------------------" << std::endl;
  std::cout << "--------------------- ALGORITH: D* Lite Replan ---------------------" << std::endl;
  std::cout << "--------------------------------------------------------------------" << std::endl;
  memcpy(grid, main_grid, grid_space);
  if(path_vector.size() > 3){
    Node new_obs(path_vector[2].x_,path_vector[2].y_);
    std::cout << "Obstacle created at: "<< std::endl;
    new_obs.PrintStatus();
    grid[new_obs.x_][new_obs.y_] = 1;
    grid[start.x_][start.y_] = 0;
    grid[goal.x_][goal.y_] = 0;
    path_vector = new_d_star_lite.replan(grid, new_obs);
  }
  else{
    std::cout << "Path size too small; no new obstacle created" << std::endl;
  }
  PrintPath(path_vector, start, goal, grid, n);

  return 0;
}
