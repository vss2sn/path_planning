/*

A* grid based planning

*/
#ifndef D_STAR_LITE_H
#define D_STAR_LITE_H

#include "utils.h"

class DStarLite{
public:
  double GetHeuristic(Node s1, Node s2);
  void MyPrint();
  std::pair<double,double> CalculateKey(const Node& s);
  std::vector<Node> GetPred(Node u);
  std::vector<Node> GetSucc(Node u);
  void InsertionSort();
  double C(Node s1, Node s2);
  void Init();
  void UpdateVertex(Node& u);
  int ComputeShortestPath();
  std::vector<Node> d_star_lite(void *grid_in, int n_in, Node start_in, Node goal_in);
  std::vector<Node> Replan(Node u);
  std::vector<Node> SetObs(Node u);
  void GeneratePathVector();
  // void CopyGrid(void *grid_in);
  std::vector<Node> UpdateStart(Node start_in);
  std::vector<Node> ReturnInvertedVector();
  bool CompareKey(std::pair<double,double>& pair_in, Node& u);
  void RunDStarLite(bool disp_inc_in = true);
  Node NextPoint();
  void DisplayGrid();
private:
  bool disp_inc = true; // Display incremental movements during D* Lite live run
  useconds_t disp_p = 500000; // Pause for displaying grid in microseconds
  Node start_, main_start_, goal_, last_;
  int grid[100][100]; // Do not let grid size exceed 100
  std::pair<double,double> S_[100][100] = {}; // Do not let grid size exceed 100
  std::vector<std::pair<Node,std::pair<double,double>>> U_;
  std::pair<double,double> km_;
  std::pair<double,double> k_old_;
  std::vector<Node> motions;
  int n, iter_ = 0, max_iter_ = 0;
  std::vector<Node> path_vector_;
  std::pair<double,double> large_num;
};

#endif D_STAR_LITE_H
