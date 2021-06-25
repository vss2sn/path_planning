#include <cmath>

#include "tests/test_utils.hpp"
#include <tuple>

using Grid = std::vector<std::vector<int>>;

Grid empty_grid{
 { 0 , 0 , 0 },
 { 0 , 0 , 0 },
 { 0 , 0 , 0 }
};

Grid no_path {
 { 0 , 0 , 0 , 0 , 0, 0 },
 { 0 , 1 , 1 , 1 , 1, 1 },
 { 1 , 1 , 1 , 0 , 1, 0 },
 { 1 , 0 , 0 , 0 , 0, 0 },
 { 0 , 0 , 0 , 0 , 0, 0 },
 { 0 , 0 , 0 , 0 , 0, 0 }
} ;

Grid path_exists_1 {
  { 0 , 0 , 0 , 0 , 0 , 0 , 0 , 1 },
  { 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 },
  { 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 },
  { 1 , 0 , 0 , 1 , 0 , 0 , 0 , 0 },
  { 0 , 0 , 0 , 1 , 0 , 0 , 1 , 0 },
  { 0 , 0 , 0 , 0 , 0 , 0 , 0 , 1 },
  { 0 , 0 , 1 , 0 , 1 , 0 , 0 , 0 },
  { 0 , 0 , 0 , 0 , 1 , 0 , 0 , 0 }
};

Grid path_exists_2 {
  { 0 , 0 , 1 , 0 , 0 , 0 , 0 , 0 },
  { 0 , 0 , 0 , 0 , 0 , 0 , 1 , 1 },
  { 1 , 0 , 0 , 0 , 1 , 0 , 0 , 0 },
  { 0 , 0 , 0 , 0 , 0 , 0 , 1 , 1 },
  { 1 , 0 , 0 , 1 , 0 , 0 , 1 , 1 },
  { 0 , 0 , 0 , 1 , 1 , 0 , 1 , 0 },
  { 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 },
  { 0 , 0 , 1 , 0 , 0 , 0 , 0 , 0 }
};

Grid no_path_grid {
  { 0 , 0 , 0 , 0 , 0, 0 },
  { 0 , 1 , 1 , 1 , 1, 1 },
  { 1 , 1 , 1 , 0 , 1, 0 },
  { 1 , 0 , 0 , 0 , 0, 0 },
  { 0 , 0 , 0 , 0 , 0, 0 },
  { 0 , 0 , 0 , 0 , 0, 0 }
} ;

class PlannerParamterizedTestsPathFound :
    public testing::TestWithParam<std::tuple<std::tuple<double, Grid>, PlannerEnum>> {
};

class PlannerParamterizedTestsNoPathFound :
    public testing::TestWithParam<std::tuple<Grid, PlannerEnum>> {
};


TEST_P (PlannerParamterizedTestsPathFound, PathFound) {
  const auto expected_value = std::get<0>(std::get<0>(GetParam()));
  auto grid = std::get<1>(std::get<0>(GetParam()));
  const auto planner_enum = std::get<1>(GetParam());

  auto planner = PlannerFactory(planner_enum, grid);

  int n = grid.size();
  Node start(0, 0, 0, 0, 0, 0);
  start.id_ = start.x_ * n + start.y_;
  start.pid_ = start.x_ * n + start.y_;
  Node goal(n-1, n-1, 0, 0, 0, 0);
  goal.id_ = goal.x_ * n +   goal.y_;
  start.h_cost_ = abs(start.x_ - goal.x_) + abs(start.y_ - goal.y_);
  grid[start.x_][start.y_] = 0;
  grid[goal.x_][goal.y_] = 0;

  const auto [path_found, path_cost] = run_test(planner.get(), start, goal);
  ASSERT_EQ(expected_value, path_cost);
}

TEST_P (PlannerParamterizedTestsNoPathFound, NoPathFound) {
  auto grid = std::get<0>(GetParam());
  const auto planner_enum = std::get<1>(GetParam());

  auto planner = PlannerFactory(planner_enum, grid);

  int n = grid.size();
  Node start(0, 0, 0, 0, 0, 0);
  start.id_ = start.x_ * n + start.y_;
  start.pid_ = start.x_ * n + start.y_;
  Node goal(n-1, n-1, 0, 0, 0, 0);
  goal.id_ = goal.x_ * n +   goal.y_;
  start.h_cost_ = abs(start.x_ - goal.x_) + abs(start.y_ - goal.y_);
  grid[start.x_][start.y_] = 0;
  grid[goal.x_][goal.y_] = 0;

  const auto [path_found, path_cost] = run_test(planner.get(), start, goal);
  ASSERT_EQ(false, path_found);
}

INSTANTIATE_TEST_SUITE_P(
  PlannerParamterizedTests,
  PlannerParamterizedTestsPathFound,
  ::testing::Combine(
    ::testing::Values(
      std::tuple<double, Grid>{4, empty_grid},
      std::tuple<double, Grid>{14, path_exists_1},
      std::tuple<double, Grid>{14, path_exists_2}
    ),
    ::testing::ValuesIn(getOptimalPlannerEnums())
  )
);

INSTANTIATE_TEST_SUITE_P(
  PlannerParamterizedTests,
  PlannerParamterizedTestsNoPathFound,
  ::testing::Combine(
    ::testing::Values(no_path_grid),
    ::testing::ValuesIn(getOptimalPlannerEnums())
  )
);


int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
