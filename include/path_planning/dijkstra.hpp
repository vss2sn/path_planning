/**
 * @file dijkstra.hpp
 * @author vss2sn
 * @brief Contains the Dijkstra class
 */

#ifndef DIJKSTRA_H
#define DIJKSTRA_H

#include <queue>

#include "path_planning/planner.hpp"
#include "utils/utils.hpp"

/**
 * @brief Class for objects that plan using the Dijkstra algorithm
 */
class Dijkstra : public Planner {
 public:
  /**
   * @brief Constructor
   * @param grid the grid on which the planner is to plan
   * @return no return value
   */
  explicit Dijkstra(std::vector<std::vector<int>> grid)
      : Planner(std::move(grid)) {}

  /**
   * @brief Dijkstra algorithm implementation
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

#endif  // DIJKSTRA_H
