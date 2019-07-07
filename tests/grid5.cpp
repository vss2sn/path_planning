#include "dijkstra.hpp"
#include "a_star.hpp"
#include "lpa_star.hpp"
#include "rrt.hpp"
#include "rrt_star.hpp"
#include "d_star_lite.hpp"
#include "ant_colony.hpp"
#include <gtest/gtest.h>

double run_test(std::vector<std::vector<int> > &grid, std::string algo){
  int n = grid.size();
  Node start(0,0,0,0,0,0);
  start.id_ = start.x_ * n + start.y_;
  start.pid_ = start.x_ * n + start.y_;
  Node goal(n-1,n-1,0,0,0,0);
  goal.id_ = goal.x_ * n + goal.y_;
  start.h_cost_ = abs(start.x_ - goal.x_) + abs(start.y_ - goal.y_);
  //Make sure start and goal are not obstacles and their ids are correctly assigned.
  grid[start.x_][start.y_] = 0;
  grid[goal.x_][goal.y_] = 0;

  std::vector<Node> path_vector;
  // Variables for RRT and RRTStar
  double threshold = 2;
  int max_iter_x_factor = 20;

  // Variables for Ant Colony Optimization
  int n_ants = 10, iterations = 50;
  float alpha = 1, beta =0.7, evap_rate = 0.3, Q = 10;

  if(algo =="dijkstra"){
    Dijkstra new_dijkstra;
    path_vector = new_dijkstra.dijkstra(grid, start, goal);
  }
  else if(algo == "a_star"){
    AStar new_a_star;
    path_vector = new_a_star.a_star(grid, start, goal);
  }
  else if(algo == "lpa_star"){
    LPAStar new_lpa_star;
    path_vector = new_lpa_star.lpa_star(grid, start, goal, 0);
  }
  else if(algo=="rrt"){
    RRT new_rrt;
    path_vector = new_rrt.rrt(grid, start, goal, max_iter_x_factor, threshold);
  }
  else if(algo=="rrtstar"){
    RRTStar new_rrt_star;
    path_vector = new_rrt_star.rrt_star(grid, start, goal, max_iter_x_factor, threshold);
  }
  else if(algo == "d_star_lite"){
    DStarLite new_d_star_lite;
    path_vector = new_d_star_lite.d_star_lite(grid, start, goal);
  }
  else if(algo == "ant_colony"){
    AntColony new_ant_colony(n_ants, alpha, beta, evap_rate, iterations, Q);
    path_vector = new_ant_colony.ant_colony(grid, start, goal);
  }

  int i;
  if(path_vector[0].cost_==-1) return -1;
  for(i = 0; i < path_vector.size(); i++){
    if(goal == path_vector[i]) break;
  }
  return path_vector[i].cost_;
}

TEST(PathPlanningTest, Test1) {
  int n = 8;
  std::vector<std::vector<int>> grid{
    { 0 , 0 , 1 , 0 , 0 , 0 , 0 , 0 },
    { 0 , 0 , 0 , 0 , 0 , 0 , 1 , 1 },
    { 1 , 0 , 0 , 0 , 1 , 0 , 0 , 0 },
    { 0 , 0 , 0 , 0 , 0 , 0 , 1 , 1 },
    { 1 , 0 , 0 , 1 , 0 , 0 , 1 , 1 },
    { 0 , 0 , 0 , 1 , 1 , 0 , 1 , 0 },
    { 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 },
    { 0 , 0 , 1 , 0 , 0 , 0 , 0 , 0 }
                   } ;
  std::vector<std::vector<int>> main_grid = grid;
  grid = main_grid;
  ASSERT_EQ(14, run_test(grid, "dijkstra"));
  grid = main_grid;
  ASSERT_EQ(14, run_test(grid, "a_star"));
  grid = main_grid;
  ASSERT_EQ(14, run_test(grid, "lpa_star"));
  // NOTE: RRT currently does not store cost. Now becomes a TODO.
  // grid = main_grid;
  // ASSERT_EQ(floor(sqrt((double)8 )), floor(run_test(grid, "rrt")));
  grid = main_grid;
  ASSERT_EQ(trunc(7*sqrt(2)*100)/100, trunc(run_test(grid, "rrtstar")*100)/100);
  grid = main_grid;
  ASSERT_EQ(14, run_test(grid, "d_star_lite"));
  grid = main_grid;
  ASSERT_GE(17, run_test(grid, "ant_colony"));
}

TEST(PathPlanningTest, Test2) {
  int n = 8;

  std::vector<std::vector<int>> grid{
    { 0 , 0 , 1 , 0 , 0 , 0 , 0 , 0 },
    { 0 , 0 , 0 , 0 , 0 , 0 , 1 , 1 },
    { 1 , 0 , 0 , 0 , 1 , 0 , 0 , 0 },
    { 0 , 0 , 0 , 0 , 0 , 0 , 1 , 1 },
    { 1 , 0 , 0 , 1 , 0 , 0 , 1 , 1 },
    { 0 , 0 , 0 , 1 , 1 , 0 , 1 , 0 },
    { 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 },
    { 0 , 0 , 1 , 0 , 0 , 0 , 0 , 0 }
                   } ;

  std::vector<std::vector<int>> main_grid = grid;
  grid = main_grid;
  ASSERT_EQ(14, run_test(grid, "dijkstra"));
  grid = main_grid;
  ASSERT_EQ(14, run_test(grid, "a_star"));
  grid = main_grid;
  ASSERT_EQ(14, run_test(grid, "lpa_star"));
  // NOTE: RRT currently does not store cost. Now becomes a TODO.
  // grid = main_grid;
  // ASSERT_EQ(floor(sqrt((double)8 )), floor(run_test(grid, "rrt")));
  grid = main_grid;
  ASSERT_EQ(trunc(7*sqrt(2)*100)/100, trunc(run_test(grid, "rrtstar")*100)/100);
  grid = main_grid;
  ASSERT_EQ(14, run_test(grid, "d_star_lite"));
  grid = main_grid;
  ASSERT_GE(17, run_test(grid, "ant_colony"));
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
