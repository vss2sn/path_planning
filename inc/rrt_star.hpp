/**
* @file rrt_star.h
* @author vss2sn
* @brief Contains the RRTStar class
*/

#ifndef RRT_STAR_H
#define RRT_STAR_H

#include "utils.hpp"

/**
* @brief Class for RRT Star objects
*/
class RRTStar{
public:
  Node FindNearestPoint(Node& new_node, int n);
  bool CheckObstacle(Node& n_1, Node& n_2);
  Node GenerateRandomNode(int n);
  void Rewire(Node new_node);
  std::vector<Node> rrt_star(std::vector<std::vector<int> > &grid, int n, Node start_in, Node goal_in, int max_iter_x_factor = 500, double threshold_in = std::numeric_limits<double>::infinity());
  bool CheckGoalVisible(Node new_node);
  void CreateObstacleList(std::vector<std::vector<int> > &grid, int n);

private:
  std::vector<Node> point_list_;
  std::vector<Node> obstacle_list_;
  std::vector<Node> near_nodes_;
  std::vector<double> near_nodes_dist_;
  Node start_, goal_;
  double threshold_ = 1;
  bool found_goal_ = false;
};

#endif RRT_STAR_H
