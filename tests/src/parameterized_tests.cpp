#include <cmath>

#include "tests/test_utils.hpp"

#include <random>
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

Grid dynamic_grid {
 { 0 , 0 , 0 , 0 , 0 , 0 },
 { 0 , 0 , 0 , 0 , 0 , 0 },
 { 0 , 0 , 0 , 0 , 0 , 0 },
 { 0 , 0 , 0 , 0 , 0 , 0 },
 { 0 , 0 , 0 , 0 , 0 , 0 },
 { 0 , 0 , 0 , 0 , 0 , 0 }
};

/****************************************************/
/* Tests for all planners where goal is reached */
/****************************************************/
class PathFound :
    public testing::TestWithParam<std::tuple<std::tuple<double, Grid>, PlannerEnum>> {
};

TEST_P (PathFound, Planners) {
  constexpr double margin_of_error = 0.00001;
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
  ASSERT_NEAR(expected_value, path_cost, margin_of_error);
}

INSTANTIATE_TEST_SUITE_P(
  OptimalPlannersPathFound,
  PathFound,
  ::testing::Combine(
    ::testing::Values(
      std::tuple<double, Grid>{4, empty_grid},
      std::tuple<double, Grid>{14, path_exists_1},
      std::tuple<double, Grid>{14, path_exists_2}
    ),
    ::testing::ValuesIn(getOptimalPlannerEnums())
  )
);

/************************************************************/
/* Tests for optimal planners where the goal is not reached */
/************************************************************/
class NoPathFound :
    public testing::TestWithParam<std::tuple<Grid, PlannerEnum>> {
};

TEST_P (NoPathFound, AllPlanners) {
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
  AllPlannersNoPathFound,
  NoPathFound,
  ::testing::Combine(
    ::testing::Values(no_path_grid),
    ::testing::ValuesIn(getAllPlannerEnums())
  )
);

/**********************************************************************/
/* Tests for asymtotically optimal planners where the goal is reached */
/**********************************************************************/
INSTANTIATE_TEST_SUITE_P(
  AsymtoticallyOptimalPlannersPathFound,
  PathFound,
  ::testing::Combine(
    ::testing::Values(
      std::tuple<double, Grid>{2*std::sqrt(2), empty_grid},
      std::tuple<double, Grid>{6*std::sqrt(2)+2, path_exists_1},
      std::tuple<double, Grid>{7*std::sqrt(2), path_exists_2}
    ),
    ::testing::ValuesIn(getAsymtoticallyOptimalPlannerEnums())
  )
);

/**********************************************************/
/* Tests for heuristic planners where the goal is reached */
/**********************************************************/

class HeuristicPathFound :
    public testing::TestWithParam<std::tuple<std::tuple<double, Grid>, PlannerEnum>> {
};

TEST_P (HeuristicPathFound, HeuristicPlanners) {
  constexpr double margin_of_error = 1.2;
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
  ASSERT_EQ(true, path_found);
  ASSERT_GE(expected_value * margin_of_error, path_cost);
}

INSTANTIATE_TEST_SUITE_P(
  HeuristicPlannersPathFound,
  HeuristicPathFound,
  ::testing::Combine(
    ::testing::Values(
      std::tuple<double, Grid>{4, empty_grid},
      std::tuple<double, Grid>{14, path_exists_1},
      std::tuple<double, Grid>{14, path_exists_2}
    ),
    ::testing::ValuesIn(getHeuristicPlannerEnums())
  )
);

/****************************************************/
/* Tests for all planners where goal is reached */
/****************************************************/
class SubOptimalPathFound :
    public testing::TestWithParam<std::tuple<std::tuple<double, Grid>, PlannerEnum>> {
};

TEST_P (SubOptimalPathFound, SubOptimalPlanners) {
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
  ASSERT_EQ(true, path_found);
  ASSERT_LE(expected_value, path_cost);
}

INSTANTIATE_TEST_SUITE_P(
  SubOptimalPlannersPathFound,
  SubOptimalPathFound,
  ::testing::Combine(
    ::testing::Values(
      std::tuple<double, Grid>{2*std::sqrt(2), empty_grid},
      std::tuple<double, Grid>{6*std::sqrt(2)+2, path_exists_1},
      std::tuple<double, Grid>{7*std::sqrt(2), path_exists_2}
    ),
    ::testing::ValuesIn(getSubOptimalPlannerEnums())
  )
);

/****************************************************/
/* Tests for dynamic planners where goal is reached */
/****************************************************/

class DynamicPathFound :
    public testing::TestWithParam<std::tuple<
      std::tuple<double, Grid>,
      PlannerEnum,
      bool,
      std::unordered_map<int, std::vector<Node>>
      >> {
};

TEST_P (DynamicPathFound, SubOptimalPlanners) {
  const auto expected_value = std::get<0>(std::get<0>(GetParam()));
  auto grid = std::get<1>(std::get<0>(GetParam()));
  const auto planner_enum = std::get<1>(GetParam());

  auto planner = PlannerFactory(planner_enum, grid);
  planner->SetDynamicObstacles(std::get<2>(GetParam()), std::get<3>(GetParam()));
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
  ASSERT_EQ(true, path_found);
  ASSERT_EQ(expected_value, path_cost);
}

INSTANTIATE_TEST_SUITE_P(
  DynamicPlannersPathFound,
  DynamicPathFound,
  ::testing::Combine (
    ::testing::Values (std::tuple<double, Grid>{20, dynamic_grid}),
    ::testing::ValuesIn (getDynamicPlannerEnums()),
    ::testing::Values (false),
    ::testing::Values (
      std::unordered_map<int, std::vector<Node>>{
        {1, {Node(0, 2), Node(1, 2), Node(2, 2), Node(3, 2), Node(4, 2)}},
        {2, {Node(1, 4), Node(2, 4), Node(3, 4), Node(4, 4), Node(5, 4)}}
      }
    )
  )
);

/***************************/
/* Tests with unknown grid */
/***************************/
class RandomGridOptimalPlanner:
    public testing::TestWithParam<std::tuple<PlannerEnum, PlannerEnum>> {
};

TEST_P (RandomGridOptimalPlanner, OptimalPlanners) {
  constexpr int n = 11;
  std::vector<std::vector<int>> grid(n, std::vector<int>(n, 0));
  MakeGrid(grid);

  std::random_device rd;   // obtain a random number from hardware
  std::mt19937 eng(rd());  // seed the generator
  std::uniform_int_distribution<int> distr(0, n - 1);  // define the range

  Node start(distr(eng), distr(eng), 0, 0, 0, 0);
  Node goal(distr(eng), distr(eng), 0, 0, 0, 0);

  start.id_ = start.x_ * n + start.y_;
  start.pid_ = start.x_ * n + start.y_;
  goal.id_ = goal.x_ * n + goal.y_;

  // Make sure start and goal are not obstacles and their ids are correctly
  // assigned.
  grid[start.x_][start.y_] = 0;
  grid[goal.x_][goal.y_] = 0;

  start.PrintStatus();
  goal.PrintStatus();

  PrintGrid(grid);

  const auto [path_found_1, path_cost_1] = run_test(PlannerFactory(std::get<0>(GetParam()), grid).get(), start, goal);
  const auto [path_found_2, path_cost_2] = run_test(PlannerFactory(std::get<1>(GetParam()), grid).get(), start, goal);
  ASSERT_EQ(path_found_1, path_found_2);
  if (path_found_1) {
    ASSERT_EQ(path_cost_1, path_cost_2);
  }
}

INSTANTIATE_TEST_SUITE_P(
  RandomGridOptimalPlanner,
  RandomGridOptimalPlanner,
  ::testing::Combine(
    ::testing::ValuesIn(getOptimalPlannerEnums()),
    ::testing::ValuesIn(getOptimalPlannerEnums())
  )
);

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
