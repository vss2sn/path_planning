#include "test_utils.cpp"

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
  ASSERT_EQ(14, run_test(grid, "jump_point_search"));
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
  grid = main_grid;
  ASSERT_GE(17, run_test(grid, "genetic_algorithm"));
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
