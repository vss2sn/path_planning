/**
* @file rrt.hpp
* @author vss2sn
* @brief Contains the RRT class
*/

#ifndef RRT_H
#define RRT_H

#include "utils.hpp"

/**
* @brief Class for RRT objects
*/
class RRT{
public:

  /**
  * @brief Find the nearest node that has been seen by the algorithm. This does not consider cost to reach the node.
  * @param new_node Node to which the nearest node must be found
  * @param n number of rows/columns
  * @return Nearest node
  */
  Node FindNearestPoint(Node& new_node, int n);

  /**
  * @brief Check if there is any obstacle between the 2 nodes. As this planner is for grid maps, the obstacles are square.
  * @param n_1 Node 1
  * @param n_2 Node 2
  * @return bool value of whether obstacle exists between nodes
  */
  bool CheckObstacle(Node& n_1, Node& n_2);

  /**
  * @brief Generates a random node
  * @param n Number of rows/columns
  * @return Generated node
  */
  Node GenerateRandomNode(int n);

  /**
  * @brief Main algorithm of RRT
  * @param grid Main grid
  * @param n number of rows/columns
  * @param start_in starting node
  * @param goal_in goal node
  * @param max_iter_x_factor Maximum number of allowable iterations before returning no path
  * @param threshold_in Maximum distance per move
  * @return path vector of nodes
  */
  std::vector<Node> rrt(std::vector<std::vector<int> > &grid, int n, Node start_in, Node goal_in, int max_iter_x_factor = 500, double threshold_in = std::numeric_limits<double>::infinity());

  /**
  * @brief Check if goal is reachable from current node
  * @param new_node Current node
  * @return bool value of whether goal is reachable from current node
  */
  bool CheckGoalVisible(Node new_node);

  /**
  * @brief Create the obstacle list from the input grid
  * @param grid input grid for algorithm
  * @param n Number of rows/columns
  * @return void
  */
  void CreateObstacleList(std::vector<std::vector<int> > &grid, int n);

private:
  std::vector<Node> point_list_;
  std::vector<Node> obstacle_list_;
  Node start_, goal_;
  double threshold_ = 1;
};

#endif RRT_H
