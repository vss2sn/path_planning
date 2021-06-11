/**
 * @file a_star.hpp
 * @author vss2sn
 * @brief Contains the AStar class
 */

#ifndef A_STAR_H
#define A_STAR_H

#include <queue>
#include "path_planning/planner.hpp"
#include "utils/utils.hpp"

/**
 * @brief Class for A Star objects
 */
class AStar : public Planner {
 public:

   AStar(std::vector<std::vector<int>> grid) : Planner (std::move(grid)) {}

  /**
   * @brief Main algorithm of A*
   * @param grid Main grid
   * @param start_in start node
   * @param goal_in goal node
   * @return vector of path
   */
  std::tuple<bool, std::vector<Node>> Plan(const Node& start_in, const Node& goal_in) override;

private:
  std::vector<Node> ConvertClosedListToPath(std::unordered_set<Node, NodeIdAsHash, compare_coordinates>& closed_list, const Node& start, const Node& goal);
};

#endif  // A_STAR_H
