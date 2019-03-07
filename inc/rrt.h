#ifndef RRT_H
#define RRT_H

#include "utils.h"

class RRT{
public:
  Node FindNearestPoint(Node& new_node, int n);
  bool CheckObstacle(Node& n_1, Node& n_2);
  Node GenerateRandomNode(int n);
  std::vector<Node> rrt(void *grid, int n, Node start_in, Node goal_in, int max_iter_x_factor = 500, double threshold_in = std::numeric_limits<double>::infinity());
  bool Checkgoal_Visible(Node new_node);
  void CreateObstacleList(void *grid, int n);
  void PrintCost(void *grid, int n);

private:
  std::vector<Node> point_list_;
  std::vector<Node> obstacle_list_;
  Node start_, goal_;
  double threshold_ = 1;
};

#endif RRT_H
