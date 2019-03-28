/**
* @file dijkstra.h
* @author vss2sn
* @brief Contains the Dijkstra class
*/

#ifndef DIJKSTRA_H
#define DIJKSTRA_H

#include "utils.h"

/**
* @brief Class for Dijkstra objects
*/
class Dijkstra{
public:
  std::vector<Node> dijkstra(std::vector<std::vector<int> > &grid, int n, Node start_, Node goal_);
private:
  std::priority_queue<Node, std::vector<Node>, compare_cost> open_list_;
  std::vector<Node> closed_list_;
  Node start_, goal_;
};

#endif DIJKSTRA_H
