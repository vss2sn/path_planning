/**
 * @file test_utils.hpp
 * @author vss2sn
 * @brief Contains the test utils
 */

#ifndef TEST_UTILS
#define TEST_UTILS

#include <gtest/gtest.h>

#include "utils/utils.hpp"
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

/**
* @brief Class to allow iteration over enum classes
*/
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

/**
* @brief creates a planner based on the enum passed in
* @return unique_ptr to the planner instance created
*/
std::unique_ptr<Planner> PlannerFactory(PlannerEnum planner_enum, const std::vector<std::vector<int>>& grid);

/**
* @brief creates and returns a vector of all planners listed in PlannerEnum
* @return vector of all planners listed in PlannerEnum
*/
std::vector<PlannerEnum> getAllPlannerEnums();

/**
* @brief creates and returns a vector of all optimal planners listed in PlannerEnum
* @return vector of all optimal planners listed in PlannerEnum
*/
std::vector<PlannerEnum> getOptimalPlannerEnums();

/**
* @brief creates and returns a vector of all asymtotically optimal planners listed in PlannerEnum
* @return vector of all asymtotically optimal planners listed in PlannerEnum
*/
std::vector<PlannerEnum> getAsymtoticallyOptimalPlannerEnums();

/**
* @brief creates and returns a vector of all heuristic planners listed in PlannerEnum
* @return vector of all heuristic planners listed in PlannerEnum
*/
std::vector<PlannerEnum> getHeuristicPlannerEnums();

/**
* @brief creates and returns a vector of all sub-optimal planners listed in PlannerEnum
* @return vector of all sub-optimal planners listed in PlannerEnum
*/
std::vector<PlannerEnum> getSubOptimalPlannerEnums();

/**
* @brief creates and returns a vector of all dynamic planners listed in PlannerEnum
* @return vector of all dynamic planners listed in PlannerEnum
*/
std::vector<PlannerEnum> getDynamicPlannerEnums();

/**
* @brief runs the test
* @param planner planner used to create the plan
* @param start start node
* @param goal goal node
* @return true when a path is found and the cost of the path
*/
std::tuple<bool, double> run_test(Planner* planner, const Node& start, const Node& goal);

#endif  // TEST_UTILS
