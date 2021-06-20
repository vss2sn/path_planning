#include "tests/test_utils.hpp"

TEST(PathPlanningTest, RandomlyGeneratedGrid) {
  constexpr int n = 8;
  std::vector<std::vector<int>> grid(n, std::vector<int>(n));
  MakeGrid(grid);
  ASSERT_EQ(run_test(grid, "dijkstra"), run_test(grid, "a_star"));
  ASSERT_EQ(run_test(grid, "jump_point_search"), run_test(grid, "a_star"));
  ASSERT_EQ(run_test(grid, "lpa_star"), run_test(grid, "a_star"));
  ASSERT_EQ(run_test(grid, "d_star_lite"), run_test(grid, "a_star"));
  ASSERT_GE(abs(run_test(grid, "a_star")*1.2), abs(run_test(grid, "ant_colony")));
  ASSERT_GE(abs(run_test(grid, "a_star")*1.2), abs(run_test(grid, "genetic_algorithm")));
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
