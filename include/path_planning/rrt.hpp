/**
 * @file rrt.hpp
 * @author vss2sn
 * @brief Contains the RRT class
 */

#ifndef RRT_H
#define RRT_H

#include <limits>
#include <tuple>
#include <unordered_map>
#include <vector>

#include "path_planning/planner.hpp"
#include "utils/utils.hpp"

/**
 * @brief Class for objects that plan using the RRT algorithm
 */
class RRT : public Planner {
 public:
  /**
   * @brief Constructor
   * @param grid the grid on which the planner is to plan
   * @return no return value
   */
  explicit RRT(std::vector<std::vector<int>> grid) : Planner(std::move(grid)) {}

  void SetParams(const int threshold = 2, const int max_iter_x_factor = 20);

  /**
   * @brief RRT algorithm implementation
   * @param start start node
   * @param goal goal node
   * @return tuple contatining a bool as to whether a path was found, and the
   * path
   */
  std::tuple<bool, std::vector<Node>> Plan(const Node& start,
                                           const Node& goal) override;

 private:
  /**
   * @brief Find the nearest Node that has been seen by the algorithm. This does
   * not consider cost to reach the node.
   * @param new_node Node to which the nearest node must be found
   * @return nearest node
   */
  std::tuple<bool, Node> FindNearestPoint(Node& new_node);

  /**
   * @brief Check if there is any obstacle between the 2 nodes. As this planner
   * is for grid maps, the obstacles are square.
   * @param n_1 Node 1
   * @param n_2 Node 2
   * @return bool value of whether obstacle exists between nodes
   */
  bool IsAnyObstacleInPath(const Node& n_1, const Node& n_2) const;

  /**
   * @brief Generates a random node
   * @return Generated node
   */
  Node GenerateRandomNode() const;

  /**
   * @brief Check if goal is reachable from current node
   * @param new_node Current node
   * @return bool value of whether goal is reachable from current node
   */
  bool CheckGoalVisible(const Node& new_node);

  /**
   * @brief Create the obstacle list from the input grid
   * @return void
   */
  void CreateObstacleList();

  std::vector<Node> CreatePath();

 private:
  Node start_, goal_;
  std::unordered_set<Node, NodeIdAsHash, compare_coordinates>
      point_list_;  // TODO: set up in cstor
  std::unordered_map<Node, std::vector<Node>> near_nodes_;
  std::vector<Node> obstacle_list_;
  double threshold_ = 1.5;       // TODO: set up in cstor
  int max_iter_x_factor_ = 500;  // TODO: set up in cstor
};

#endif  // RRT_H
