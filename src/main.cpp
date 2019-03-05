#include "dijkstra.h"
#include "a_star.h"
#include "rrt.h"
#include "rrt_star.h"

int main(){
  int n = 3;
  int num_points = n*n;

  /*
  int grid[n][n] = {
                     { 0 , 1 , 1 },
                     { 1 , 0 , 1 },
                     { 1 , 1 , 0 }
                   };
  */

  n = 6;
  int grid[n][n] = {
                     { 0 , 0 , 0 , 0 , 0, 0 },
                     { 0 , 1 , 0 , 0 , 0, 0 },
                     { 1 , 0 , 1 , 1 , 1, 0 },
                     { 1 , 0 , 1 , 0 , 1, 0 },
                     { 0 , 0 , 1 , 1 , 1, 1 },
                     { 0 , 0 , 0 , 0 , 0, 0 }
                   } ;

  //int grid[n][n];
  //make_grid(grid, n);

  //NOTE:
  // x = row index, y = column index.

  std::cout << "Grid:" << std::endl;
  std::cout << "1. Points not considered ---> 0" << std::endl;
  std::cout << "2. Obstacles             ---> 1" << std::endl;
  print_grid(grid, n);

  //Make sure start and goal not obstacles and their ids are correctly assigned.
  Node start(0,0,0,0,0,0);
  start.id = start.x * n + start.y;
  start.pid = start.x * n + start.y;
  Node goal(n-1,n-1,0,0,0,0);
  goal.id = goal.x * n + goal.y;

  grid[start.x][start.y] = 0;
  grid[goal.x][goal.y] = 0;

  DIJKSTRA new_dijkstra;
  std::vector<Node> path_vector = new_dijkstra.dijkstra(grid, n, start, goal);
  print_path(path_vector, start, goal, grid, n);

  A_STAR new_a_star
  std::vector<Node> path_vector = new_a_star.a_star(grid, n, start, goal);
  print_path(path_vector, start, goal, grid, n);

  RRT new_rrt;
  std::vector<Node> path_vector = new_rrt.rrt(grid, n, start, goal, 5000, 3);
  print_path(path_vector, start, goal, grid, n);

  RRT_STAR new_rrt_star;
  std::vector<Node> path_vector = new_rrt_star.rrt_star(grid, n, start, goal, 5000, 3);
  print_path(path_vector, start, goal, grid, n);

  return 0;
}
