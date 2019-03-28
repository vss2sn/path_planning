/**
* @file a_star.h
* @author vss2sn
* @brief Contains the AStar class
*/

#ifndef A_STAR_H
#define A_STAR_H

#include "utils.h"

/**
* @brief Class for A Star objects
*/
class AStar{
public:
  std::vector<Node> a_star(std::vector<std::vector<int>> &grid, int n, Node start_, Node goal_);
private:
  std::vector<Node> open_list_;
  std::vector<Node> closed_list_;
  Node start_, goal_;
};

#endif A_STAR_H
