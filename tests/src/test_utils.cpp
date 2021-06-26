/**
 * @file test_utils.cpp
 * @author vss2sn
 * @brief Contains the util functions for testing
 */

#include "tests/test_utils.hpp"

std::unique_ptr<Planner> PlannerFactory(PlannerEnum planner_enum, const std::vector<std::vector<int>>& grid) {
  if (planner_enum == PlannerEnum::DIJKSTRA) {
    return std::make_unique<Dijkstra>(grid);
  } else if (planner_enum == PlannerEnum::A_STAR) {
    return std::make_unique<AStar>(grid);
  } else if (planner_enum == PlannerEnum::D_STAR_LITE) {
    return std::make_unique<DStarLite>(grid);
  } else if (planner_enum == PlannerEnum::LPA_STAR) {
    return std::make_unique<LPAStar>(grid);
  } else if (planner_enum == PlannerEnum::RRT) {
    return std::make_unique<RRT>(grid);
  } else if (planner_enum == PlannerEnum::RRT_STAR) {
    return std::make_unique<RRTStar>(grid);
  } else if (planner_enum == PlannerEnum::ANT_COLONY) {
    return std::make_unique<AntColony>(grid);
  } else if (planner_enum == PlannerEnum::GENETIC) {
    return std::make_unique<GeneticAlgorithm>(grid);
  }
  std::cout << "Invalid planner. Using Dijkstra." << '\n';
  return std::make_unique<Dijkstra>(grid);
}

std::vector<PlannerEnum> getAllPlannerEnums() {
  return std::vector<PlannerEnum> {
    PlannerEnum::DIJKSTRA,
    PlannerEnum::A_STAR,
    PlannerEnum::D_STAR_LITE,
    PlannerEnum::LPA_STAR,
    PlannerEnum::RRT_STAR,
    PlannerEnum::ANT_COLONY,
    PlannerEnum::GENETIC,
    PlannerEnum::RRT
  };
}

std::vector<PlannerEnum> getOptimalPlannerEnums() {
  return std::vector<PlannerEnum> {
    PlannerEnum::DIJKSTRA,
    PlannerEnum::A_STAR,
    PlannerEnum::D_STAR_LITE,
    PlannerEnum::LPA_STAR
  };
}

std::vector<PlannerEnum> getAsymtoticallyOptimalPlannerEnums() {
  return std::vector<PlannerEnum> {
    PlannerEnum::RRT_STAR
  };
}

std::vector<PlannerEnum> getHeuristicPlannerEnums() {
  return std::vector<PlannerEnum> {
    PlannerEnum::ANT_COLONY,
    PlannerEnum::GENETIC
  };
}

std::vector<PlannerEnum> getSubOptimalPlannerEnums() {
  return std::vector<PlannerEnum>{
    PlannerEnum::RRT
  };
}

std::vector<PlannerEnum> getDynamicPlannerEnums() {
  return std::vector<PlannerEnum> {
    PlannerEnum::LPA_STAR,
    PlannerEnum::D_STAR_LITE,
  };
}


std::tuple<bool, double> run_test(Planner* planner, const Node& start, const Node& goal) {
  const auto [path_found, path] = planner->Plan(start, goal);
  if(path_found) {
    for(const auto& p : path) {
      if(CompareCoordinates(p, goal)) {
        return {path_found, p.cost_};
      }
    }
  }
  return {path_found, 0};
}
