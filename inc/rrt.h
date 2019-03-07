#ifndef RRT_H
#define RRT_H

#include "utils.h"

class RRT{
public:
  Node find_nearest_point(Node& new_node, int n);
  bool check_obstacle(Node& n_1, Node& n_2);
  Node generate_random_node(int n);
  std::vector<Node> rrt(void *grid, int n, Node start_in, Node goal_in, int max_iter_x_factor = 500, double threshold_in = std::numeric_limits<double>::infinity());
  bool check_goal_visible(Node new_node);
  void create_obstacle_list(void *grid, int n);
  void print_cost(void *grid, int n);

private:
  std::vector<Node> point_list;
  std::vector<Node> obstacle_list;
  Node start, goal;
  double threshold = 1;
  int n;
};

#endif RRT_H
