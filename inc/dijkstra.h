#ifndef DIJKSTRA_H
#define DIJKSTRA_H

#include "utils.h"

class DIJKSTRA{
public:
  std::vector<Node> dijkstra(void *grid, int n, Node start, Node goal);
private:
  std::priority_queue<Node, std::vector<Node>, compare_cost> point_list;
};

#endif DIJKSTRA_H
