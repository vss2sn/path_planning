/**
 * @file test_utils.hpp
 * @author vss2sn
 * @brief Contains the test utils
 */

#ifndef TEST_UTILS
#define TEST_UTILS

#include <gtest/gtest.h>

#include "path_planning/planner.hpp"
#include "path_planning/a_star.hpp"
#include "path_planning/ant_colony.hpp"
#include "path_planning/d_star_lite.hpp"
#include "path_planning/dijkstra.hpp"
#include "path_planning/genetic_algorithm.hpp"
#include "path_planning/jump_point_search.hpp"
#include "path_planning/lpa_star.hpp"
#include "path_planning/rrt.hpp"
#include "path_planning/rrt_star.hpp"

enum class PlannerEnum {
  DIJKSTRA,
  A_STAR,
  D_STAR_LITE,
  LPA_STAR,
  RRT_STAR,
  ANT_COLONY,
  GENETIC,
  RRT,

  First = DIJKSTRA,
  Last = RRT
};

template<typename T>
class Enum {
public:
  class Iterator {
    public:
      Iterator(const int value) :
         m_value(value) { }

      T operator*() const {
         return static_cast<T>(m_value);
      }

      void operator++() {
         ++m_value;
      }

      bool operator != (const Iterator rhs) const {
         return m_value != rhs.m_value;
      }
    private:
      int m_value;
  };
};

template< typename T >
typename Enum<T>::Iterator begin(Enum<T>) {
  return typename Enum<T>::Iterator(static_cast<int>(T::First));
}

template< typename T >
typename Enum<T>::Iterator end(Enum<T>) {
  return typename Enum<T>::Iterator(static_cast<int>(T::Last) + 1);
}

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
};

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

#endif  // TEST_UTILS
