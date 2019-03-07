#ifndef A_STAR_H
#define A_STAR_H

#include "utils.h"

class A_STAR{
public:
  std::vector<Node> a_star(void *grid, int n, Node start, Node goal);
};

#endif A_STAR_H
