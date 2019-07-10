#ifndef GENETIC_ALGORITHM_H
#define GENETIC_ALGORITHM_H

#include "utils.hpp"

class GeneticAlgorithm{
public:
  GeneticAlgorithm(int generations = 10000, int popsize = 100, float c = 1.05);

  std::vector<Node> genetic_algorithm(std::vector<std::vector<int>>& grid, Node start, Node goal, int path_length = 30);

  std::vector<Node> ReturnLastPath(); // given the way a genetic algorithm decreases fitness values, the last path is likely ot be the best. Can reorder ased on actual fitness values if required.

  void PrintChromosome(std::vector<Node>& path);

  void PrintPathOfChromosome(std::vector<Node>& path);

  void InitialSetup(std::vector<Node>& path);

  int CalculateFitness(std::vector<Node>& path);

  void CrossoverMutation();

  bool CheckPath(std::vector<Node>& path);

private:
  std::vector<std::vector<int>> grid_;
  std::vector<Node> motions_;
  Node start_, goal_;
  int path_length_, n_, f_val, generation_, generations_, popsize_;
  float c_;
  std::vector<std::vector<Node>> paths_, truepaths_;
  bool found_;
};

#endif GENETIC_ALGORITHM_H
