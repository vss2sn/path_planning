/**
* @file dijkstra.hpp
* @author vss2sn
* @brief Contains the Dijkstra class
*/

#ifndef DIJKSTRA_H
#define DIJKSTRA_H

#include <queue>

#include "utils.hpp"

/**
* @brief Class for Dijkstra objects
*/
class Dijkstra{
public:

  /**
   * @brief Main algorithm of Dijstra.
   */
  std::vector<Node> dijkstra(std::vector<std::vector<int>>& grid, const Node& start_, const Node& goal_);
private:
  std::priority_queue<Node, std::vector<Node>, compare_cost> open_list_;
  std::vector<Node> closed_list_;
  Node start_, goal_;
  int n;
};

#endif  // DIJKSTRA_H
