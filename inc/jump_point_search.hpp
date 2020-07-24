/**
 * @file jump_point_search.hpp
 * @author vss2sn
 * @brief Contains the JumpPointSearch class
 */

#ifndef JUMP_POINT_SEARCH_H
#define JUMP_POINT_SEARCH_H

#include <queue>
#include <unordered_set>

#include "utils.hpp"

class JumpPointSearch {
 public:
  std::vector<Node> jump_point_search(std::vector<std::vector<int>>& grid,
                                      const Node& start_in,
                                      const Node& goal_in);
  void InsertionSort(std::vector<Node>& v) const;
  bool has_forced_neighbours(const Node& new_point, const Node& next_point,
                             const Node& motion) const;
  Node jump(const Node& new_point, const Node& motion, const int id);

 private:
  std::priority_queue<Node, std::vector<Node>, compare_cost> open_list_;
  std::vector<std::vector<int>> grid;
  std::vector<Node> closed_list_;
  std::unordered_set<int> pruned;
  Node start_, goal_;
  int n;
};
#endif  // JUMP_POINT_SEARCH_H
