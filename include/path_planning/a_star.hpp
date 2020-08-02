/**
 * @file a_star.hpp
 * @author vss2sn
 * @brief Contains the AStar class
 */

#ifndef A_STAR_H
#define A_STAR_H

#include <queue>

#include "utils/utils.hpp"

/**
 * @brief Class for A Star objects
 */
class AStar {
 public:
  /**
   * @brief Main algorithm of A*
   * @param grid Main grid
   * @param start_in start node
   * @param goal_in goal node
   * @return vector of path
   */
  std::vector<Node> a_star(std::vector<std::vector<int>>& grid,
                           const Node& start_in, const Node& goal_in);

  /**
   * @brief Using insertion sort to sort the vector list that maintains the
   * priority queue. Good for a mostly sorted queue. Sort called afterevery
   * insertion to maintain queue. Not using standard queue as iterating over is
   * not allowed.
   * @param v Vector to be sorted
   * @return void
   */
  void InsertionSort(std::vector<Node>& v) const;

 private:
  std::priority_queue<Node, std::vector<Node>, compare_cost> open_list_;
  std::vector<Node> closed_list_;
  Node start_, goal_;
  int n;
};

#endif  // A_STAR_H
