/**
* @file rrt.h
* @author vss2sn
* @brief Contains the RRT class
*/

#ifndef RRT_H
#define RRT_H

#include "utils.h"

/**
* @brief Class for RRT objects
*/
class RRT{
public:
  Node FindNearestPoint(Node& new_node, int n);
  bool CheckObstacle(Node& n_1, Node& n_2);
  Node GenerateRandomNode(int n);
  std::vector<Node> rrt(std::vector<std::vector<int> > &grid, int n, Node start_in, Node goal_in, int max_iter_x_factor = 500, double threshold_in = std::numeric_limits<double>::infinity());
  bool CheckGoalVisible(Node new_node);
  void CreateObstacleList(std::vector<std::vector<int> > &grid, int n);

private:
  std::vector<Node> point_list_;
  std::vector<Node> obstacle_list_;
  Node start_, goal_;
  double threshold_ = 1;
};

#endif RRT_H
