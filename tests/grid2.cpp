#include "dijkstra.h"
#include "a_star.h"
#include "rrt.h"
#include "rrt_star.h"

#include <gtest/gtest.h>

double run_test(void *grid, int n){
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

  AStar new_a_star;
  std::vector<Node> path_vector = new_a_star.a_star(*p_grid, n, start, goal);
  std::cout <<path_vector[0].cost_ << std::endl;
  int i;
  for(i = 0; i < path_vector.size(); i++){
    if(goal == path_vector[i]) break;
  }
  return path_vector[i].cost_;
}


TEST(PathPlanningTest, Hello) {
  int n = 3;
  int grid[n][n] = {
                     { 0 , 0 , 0 },
                     { 0 , 0 , 0 },
                     { 0 , 0 , 0 }
                   };
  ASSERT_EQ(4, run_test(grid, n));
}

TEST(PathPlanningTest, HelloAgain) {
  int n = 3;
  int grid[n][n] = {
                     { 0 , 0 , 0 },
                     { 0 , 0 , 0 },
                     { 0 , 0 , 0 }
                   };
  ASSERT_EQ(4, run_test(grid, n));
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
