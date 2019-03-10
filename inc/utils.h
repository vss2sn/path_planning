#ifndef UTILS_H
#define UTILS_H

#include "main.h"

#define BLACK "\x1b[1;30m"
#define RED "\x1b[1;31m"
#define GREEN "\x1b[1;32m"
#define YELLOW "\x1b[1;33m"
#define BLUE "\x1b[1;34m"
#define MAGENTA "\x1b[1;35m"
#define CYAN "\x1b[1;36m"
#define WHITE "\x1b[1;37m"
#define RESET "\x1b[1;0m"

class Node{
public:
  int x_, y_, id_, pid_;
  double cost_, h_cost_;
  Node(int x = 0, int y = 0, double cost = 0, double h_cost = 0, int id = 0, int pid = 0);
  void PrintStatus();
  Node operator+(Node p);
  Node operator=(Node p);
  bool operator==(Node p);
  bool operator!=(Node p);
};

struct compare_cost{
  bool operator()(Node& p1, Node& p2);
};

struct compare_points{
  bool operator()(const Node p1, const Node p2);
};

struct compare_id{
  bool operator()(const Node p1, const Node p2);
};

std::vector<Node> GetMotion(int n);
void MakeGrid(void *grid, int n);
void PrintGrid(void *grid, int n);
void PrintPath(std::vector<Node> path_vector, Node start_, Node goal_, void *grid, int n);
void PrintCost(void *grid, int n, std::vector<Node> point_list);
#endif UTILS_H
