/**
* @file utils.h
* @author vss2sn  
* @brief Contains common/commonly used funtions and classes
*/

#ifndef UTILS_H
#define UTILS_H

#include "main.hpp"

#define BLACK "\x1b[1;30m"
#define RED "\x1b[1;31m"
#define GREEN "\x1b[1;32m"
#define YELLOW "\x1b[1;33m"
#define BLUE "\x1b[1;34m"
#define MAGENTA "\x1b[1;35m"
#define CYAN "\x1b[1;36m"
#define WHITE "\x1b[1;37m"
#define RESET "\x1b[1;0m"

/**
* @brief Node class
* @param x_ X value
* @param y_ Y value
* @param cost_ Cost to get to this node
* @param h_cost_ Heuritic cost of this node
* @param id_ Node's id
* @param pid_ Node's parent's id
*/
class Node{
// Variables used here are constantly accessed and checked; leaving public for now.
public:
    /** \brief x coordinate */
  int x_;
    /** \brief y coordinate */
  int y_;
  /** \brief Node id */
  int id_;
  /** \brief Node's parent's id */
  int pid_;
  /** \brief cost to reach this node */
  double cost_;
  /** \brief heuristic cost to reach the goal */
  double h_cost_;
  Node(int x = 0, int y = 0, double cost = 0, double h_cost = 0, int id = 0, int pid = 0);
  void PrintStatus();
  Node operator+(Node p);
  Node operator-(Node p);
  void operator=(Node p);
  bool operator==(Node p);
  bool operator!=(Node p);
};

struct compare_cost{
  bool operator()(Node& p1, Node& p2);
};

std::vector<Node> GetMotion();
void PrintGrid(std::vector<std::vector<int>> &grid, int n);
void PrintPath(std::vector<Node> path_vector, Node start_, Node goal_, std::vector<std::vector<int>> &grid, int n);
void PrintCost(std::vector<std::vector<int>> &grid, int n, std::vector<Node> point_list);
void MakeGrid(std::vector<std::vector<int>> &grid, int n);
#endif UTILS_H
