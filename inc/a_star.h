#ifndef A_STAR_H
#define A_STAR_H

#include "utils.h"

class AStar{
public:
  std::vector<Node> a_star(void *grid, int n, Node start_, Node goal_);
private:
  std::priority_queue<Node, std::vector<Node>, compare_cost> point_list_;
  Node start_, goal_;
};

#endif A_STAR_H
