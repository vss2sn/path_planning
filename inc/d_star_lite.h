/*

A* grid based planning

*/
#ifndef D_STAR_LITE_H
#define D_STAR_LITE_H

#include "utils.h"

class DStarLite{
public:
  double get_heuristic(Node s1, Node s2);
  void my_print();
  std::pair<double,double> CalculateKey(const Node& s);
  std::vector<Node> GetPred(Node u);
  std::vector<Node> GetSucc(Node u);
  void insertionSort();
  double c(Node s1, Node s2);
  void Init();
  void UpdateVertex(Node u);
  int ComputeShortestPath();
  int d_star_lite(void *grid_in, int n_in, Node start_in, Node goal_in);
  int replan(Node u);
  std::vector<Node> generate_path_vector();
private:
  Node start_, main_start_, goal_, last_;
  int grid[100][100]; // Do not let grid size exceed 100
  std::pair<double,double> S_[100][100] = {}; // Do not let grid size exceed 100
  std::vector<std::pair<Node,std::pair<double,double>>> U_;
  std::pair<double,double> km_;
  std::pair<double,double> k_old_;
  int n;
  std::vector<Node> path_vector_;
  std::vector<Node> motions;
};

#endif D_STAR_LITE_H
