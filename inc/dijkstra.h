#ifndef DIJKSTRA_H
#define DIJKSTRA_H

#include "utils.h"

class DIJKSTRA{
public:
  std::vector<Node> dijkstra(void *grid, int n, Node start, Node goal);
};

#endif DIJKSTRA_H
