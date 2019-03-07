#ifndef RRT_STAR_H
#define RRT_STAR_H

#include "utils.h"

class RRT_STAR{
public:
  Node FindNearestPoint(Node& new_node, int n);
  bool CheckObstacle(Node& n_1, Node& n_2);
  Node GenerateRandomNode(int n);
  void Rewire(Node new_node);
  std::vector<Node> rrt_star(void *grid, int n, Node start_in, Node goal_in, int max_iter_x_factor = 500, double threshold_in = std::numeric_limits<double>::infinity());
  bool CheckGoalVisible(Node new_node);
  void CreateObstacleList(void *grid, int n);
  void PrintCost(void *grid, int n);

private:
  std::vector<Node> point_list;
  std::vector<Node> obstacle_list;
  std::vector<Node> near_nodes;
  std::vector<double> near_nodes_dist;
  Node start, goal;
  double threshold = 1;
  int n;
  bool found_goal = false;
};

#endif RRT_STAR_H
