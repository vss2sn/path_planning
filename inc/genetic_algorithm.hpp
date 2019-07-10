#ifndef GENETIC_ALGORITHM_H
#define GENETIC_ALGORITHM_H

#include "utils.hpp"

/**
* @brief Class for Genetic Algorithm ojects
*/
class GeneticAlgorithm{
public:
  /**
  * @brief Constructr to set up Genetic Algorithm object
  * @param generations number of generations of genetic algorithms
  * @param popsize size of population in each generation
  * @param c multiplicative value for fitness criteria for path selection/elimination
  * @return no return value
  */
  GeneticAlgorithm(int generations = 10000, int popsize = 30, float c = 1.05, bool shorten_chromosome = false);

  /**
  * @brief Main algorithm of genertic algorithm
  * @param grid Main grid
  * @param start start node
  * @param goal goal node
  * @return best path within last iteration of the algorithm
  */
  std::vector<Node> genetic_algorithm(std::vector<std::vector<int>>& grid, Node start, Node goal, int path_length = 30);

  /**
  * @brief Returns the last valid path within an iteration
  * @return last valid path
  * @details <details>
  */
  std::vector<Node> ReturnLastPath(); // given the way a genetic algorithm decreases fitness values, the last path is likely ot be the best. Can reorder ased on actual fitness values if required.

  /**
  * @brief Prints a chromosome, ie the sequence of moves made.
  * @param path Each chromosome represents a path by a sequence of moves.
  * @return void
  */
  void PrintChromosome(std::vector<Node>& path);

  /**
  * @brief Prints the path represented by the chomosome
  * @param path Each chromosome represents a path by a sequence of moves.
  * @return void
  */
  void PrintPathOfChromosome(std::vector<Node>& path);

  /**
  * @brief Creates the first path. Set to create the path by moving along a diagonal initially if possible
  * @param path Each chromosome represents a path by a sequence of moves.
  * @return void
  */
  void InitialSetup(std::vector<Node>& path);

  /**
  * @brief Calculates fitness value of a path
  * @param path Each chromosome represents a path by a sequence of moves.
  * @return value of fitness
  */
  int CalculateFitness(std::vector<Node>& path);

  /**
  * @brief Mutation function to create random combinations of parents as well as some random changes. Applied to every chromosome in paths.
  */
  void CrossoverMutation();

  /**
  * @brief Checks whether path is valid or not.
  * @param path Each chromosome represents a path by a sequence of moves.
  * @return bool value of validity of path
  */
  bool CheckPath(std::vector<Node>& path);

private:
  std::vector<std::vector<int>> grid_;
  std::vector<Node> motions_;
  Node start_, goal_;
  int path_length_, n_, f_val, generation_, generations_, popsize_;
  float c_;
  std::vector<std::vector<Node>> paths_, truepaths_;
  bool found_, shorten_chromosome_;
};

#endif GENETIC_ALGORITHM_H
