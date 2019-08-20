#ifndef JUMP_POINT_SEARCH_H
#define JUMP_POINT_SEARCH_H

#include "utils.hpp"

class JumpPointSearch{
public:
  std::vector<Node> jump_point_search(std::vector<std::vector<int>> &grid, Node start_in, Node goal_in);
  void InsertionSort(std::vector<Node>& v);
  bool has_forced_neighbours(Node& new_point, Node& next_point, Node& motion);
  Node jump(Node& new_point, Node& motion, int id);
private:
  std::priority_queue<Node, std::vector<Node>, compare_cost> open_list_;
  std::vector<std::vector<int>> grid;
  std::vector<Node> closed_list_;
  std::unordered_set<int> pruned;
  Node start_, goal_;
  int n;
};
#endif JUMP_POINT_SEARCH_H
