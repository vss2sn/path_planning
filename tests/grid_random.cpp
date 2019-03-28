#include "dijkstra.h"
#include "a_star.h"
#include "rrt.h"
#include "rrt_star.h"
#include "d_star_lite.h"
#include <gtest/gtest.h>

double run_test(std::vector<std::vector<int> > &grid, int n, std::string algo){

  Node start(0,n-1,0,0,0,0);
  start.id_ = start.x_ * n + start.y_;
  start.pid_ = start.x_ * n + start.y_;
  Node goal(n-1,0,0,0,0,0);
  goal.id_ = goal.x_ * n + goal.y_;
  start.h_cost_ = abs(start.x_ - goal.x_) + abs(start.y_ - goal.y_);
  //Make sure start and goal are not obstacles and their ids are correctly assigned.
  grid[start.x_][start.y_] = 0;
  grid[goal.x_][goal.y_] = 0;

  std::vector<Node> path_vector;
  double threshold = 2;
  int max_iter_x_factor = 20;

  if(algo =="dijkstra"){
    Dijkstra new_dijkstra;
    path_vector = new_dijkstra.dijkstra(grid, n, start, goal);
  }
  else if(algo == "a_star"){
    AStar new_a_star;
    path_vector = new_a_star.a_star(grid, n, start, goal);
  }
  else if(algo=="rrt"){
    RRT new_rrt;
    path_vector = new_rrt.rrt(grid, n, start, goal, max_iter_x_factor, threshold);
  }
  else if(algo=="rrtstar"){
    RRTStar new_rrt_star;
    path_vector = new_rrt_star.rrt_star(grid, n, start, goal, max_iter_x_factor, threshold);
  }
  else if(algo == "d_star_lite"){
    DStarLite new_d_star_lite;
    path_vector = new_d_star_lite.d_star_lite(grid, n, start, goal);
  }

  if(path_vector.size()==1) return -1;
  int i;
  for(i = 0; i < path_vector.size(); i++){
    if(goal == path_vector[i]) break;
  }
  return path_vector[i].cost_;
}

TEST(PathPlanningTest, Test1) {
  int n = 8;
  std::vector<std::vector<int>> grid_1(n);
  std::vector<int> tmp(n);
  for (int i = 0; i < n; i++){
    grid_1[i] = tmp;
  }
  MakeGrid(grid_1, n);
  std::vector<std::vector<int>> grid_2 = grid_1;
  ASSERT_EQ(run_test(grid_1, n, "dijkstra"), run_test(grid_2, n, "a_star"));
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
