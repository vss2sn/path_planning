#include "dijkstra.h"
#include "a_star.h"
#include "rrt.h"
#include "rrt_star.h"
#include <gtest/gtest.h>

double run_test(void *grid, int n, std::string algo){
  int (*p_grid)[n][n] = (int (*)[n][n]) grid;
  Node start(0,0,0,0,0,0);
  start.id_ = start.x_ * n + start.y_;
  start.pid_ = start.x_ * n + start.y_;
  Node goal(n-1,n-1,0,0,0,0);
  goal.id_ = goal.x_ * n + goal.y_;
  start.h_cost_ = abs(start.x_ - goal.x_) + abs(start.y_ - goal.y_);
  //Make sure start and goal are not obstacles and their ids are correctly assigned.
  (*p_grid)[start.x_][start.y_] = 0;
  (*p_grid)[goal.x_][goal.y_] = 0;

  std::vector<Node> path_vector;
  double threshold = 2;
  int max_iter_x_factor = 20;

  if(algo =="dijkstra"){
    Dijkstra new_dijkstra;
    path_vector = new_dijkstra.dijkstra(*p_grid, n, start, goal);
  }
  else if(algo == "a_star"){
    AStar new_a_star;
    path_vector = new_a_star.a_star(*p_grid, n, start, goal);
  }
  else if(algo=="rrt"){
    RRT new_rrt;
    path_vector = new_rrt.rrt(*p_grid, n, start, goal, max_iter_x_factor, threshold);
  }
  else if(algo=="rrt_star"){
    RRT_Star new_rrt_star;
    path_vector = new_rrt_star.rrt_star(*p_grid, n, start, goal, max_iter_x_factor, threshold);
  }
  if(path_vector.size()==1) return (double)-1.0;
  int i;
  for(i = 0; i < path_vector.size(); i++){
    if(goal == path_vector[i]) break;
  }
  return path_vector[i].cost_;
}

TEST(PathPlanningTest,Test1) {
  int n = 6;
  int main_grid[n][n];
  int grid[n][n] = {
                     { 0 , 0 , 0 , 0 , 0, 0 },
                     { 0 , 1 , 1 , 1 , 1, 1 },
                     { 1 , 1 , 1 , 0 , 1, 0 },
                     { 1 , 0 , 0 , 0 , 0, 0 },
                     { 0 , 0 , 0 , 0 , 0, 0 },
                     { 0 , 0 , 0 , 0 , 0, 0 }
                   } ;
  int grid_space = n*n*sizeof(int);

  memcpy(main_grid, grid, grid_space);

  memcpy(grid, main_grid, grid_space);
  ASSERT_EQ(-1, run_test(grid, n, "dijkstra"));
  memcpy(grid, main_grid, grid_space);
  ASSERT_EQ(-1, run_test(grid, n, "a_star"));
  // NOTE: RRT currently does not store cost. Now becomes a TODO.
  // memcpy(grid, main_grid, grid_space);
  // ASSERT_EQ(floor(sqrt((double)8 )), floor(run_test(grid, n, "rrt")));
  memcpy(grid, main_grid, grid_space);
  ASSERT_EQ(-1, run_test(grid, n, "rrt_star"));
}

TEST(PathPlanningTest,Test2) {
  int n = 6;
  int main_grid[n][n];
  int grid[n][n] = {
                     { 0 , 0 , 0 , 0 , 0, 0 },
                     { 0 , 1 , 1 , 1 , 1, 1 },
                     { 1 , 1 , 1 , 0 , 1, 0 },
                     { 1 , 0 , 0 , 0 , 0, 0 },
                     { 0 , 0 , 0 , 0 , 0, 0 },
                     { 0 , 0 , 0 , 0 , 0, 0 }
                   } ;
  int grid_space = n*n*sizeof(int);
  int t_grid[n][n];
  for (int i = 0; i < n; ++i)
      for (int j = 0; j < n; ++j)
          t_grid[j][i]= grid[i][j];
  memcpy(main_grid, t_grid, grid_space);

  memcpy(grid, main_grid, grid_space);
  ASSERT_EQ(-1, run_test(grid, n, "dijkstra"));
  memcpy(grid, main_grid, grid_space);
  ASSERT_EQ(-1, run_test(grid, n, "a_star"));
  // NOTE: RRT currently does not store cost. Now becomes a TODO.
  // memcpy(grid, main_grid, grid_space);
  // ASSERT_EQ(floor(sqrt((double)8 )), floor(run_test(grid, n, "rrt")));
  memcpy(grid, main_grid, grid_space);
  ASSERT_EQ(-1, run_test(grid, n, "rrt_star"));
}


int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
