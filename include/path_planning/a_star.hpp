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
 * @brief Class for objects that plan using the A* algorithm
 */
class AStar : public Planner {
 public:
  /**
   * @brief Constructor
   * @param grid the grid on which the planner is to plan
   * @return no return value
   */
  explicit AStar(std::vector<std::vector<int>> grid)
      : Planner(std::move(grid)) {}

  /**
   * @brief A* algorithm implementation
   * @param start start node
   * @param goal goal node
   * @return tuple contatining a bool as to whether a path was found, and the
   * path
   */
  std::tuple<bool, std::vector<Node>> Plan(const Node& start,
                                           const Node& goal) override;

 private:
  std::vector<Node> ConvertClosedListToPath(
      std::unordered_set<Node, NodeIdAsHash, compare_coordinates>& closed_list,
      const Node& start, const Node& goal);
};

#endif  // A_STAR_H
