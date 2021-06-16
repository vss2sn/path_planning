/**
 * @file genetic_algorithm.hpp
 * @author vss2sn
 * @brief Contains the GeneticAlgorithm class
 */

#ifndef GENETIC_ALGORITHM_H
#define GENETIC_ALGORITHM_H

#include <limits>

#include "path_planning/planner.hpp"
#include "utils/utils.hpp"

/**
 * @brief Class for Genetic Algorithm ojects
 */
class GeneticAlgorithm : public Planner {
 public:
  /**
   * @brief Constructr to set up Genetic Algorithm object
   * @param generations number of generations of genetic algorithms
   * @param popsize size of population in each generation
   * @param c multiplicative value for fitness criteria for path
   * selection/elimination
   * @param shorten_chromosome reduce size of chomosome based on best path
   * @return no return value
   */
  GeneticAlgorithm(const std::vector<std::vector<int>>& grid) : Planner (grid) {};

  void SetParams(const int generations = 10000, const int popsize = 30,
                 const float c = 1.05, const bool shorten_chromosome = false,
                 const int path_length = 30);

  /**
   * @brief Main algorithm of genertic algorithm
   * @param grid Main grid
   * @param start start node
   * @param goal goal node
   * @param path_length length of path (chromosome size)
   * @return best path within last iteration of the algorithm
   */
  std::tuple<bool, std::vector<Node>> Plan(const Node& start, const Node& goal) override;

  /**
   * @brief Returns the last valid path within an iteration
   * @return last valid path
   */
  std::vector<Node> ReturnLastPath() const;
  // given the way a genetic algorithm decreases fitness values, the last path
  // is likely ot be the best. Can reorder ased on actual fitness values if
  // required.

  /**
   * @brief Prints a chromosome, ie the sequence of moves made.
   * @param path Each chromosome represents a path by a sequence of moves.
   * @return void
   */
  void PrintChromosome(const std::vector<Node>& path) const;

  /**
   * @brief Prints the path represented by the chomosome
   * @param path Each chromosome represents a path by a sequence of moves.
   * @return void
   */
  void PrintPathOfChromosome(const std::vector<Node>& path) const;

  /**
   * @brief Creates a chromosome using the x and y distances between
   * the start and the goal
   * @return chromosome
   */
  std::vector<Node> GenerateSimplePath() const;

  /**
   * @brief Creates a random chromosome
   * @return chromosome
   */
  std::vector<Node> GenerateRandomPath() const;
  /**
   * @brief Calculates fitness value of a path
   * @param path Each chromosome represents a path by a sequence of moves.
   * @return value of fitness
   */
  int CalculateFitness(const std::vector<Node>& path) const;

  /**
   * @brief Mutation function to create random combinations of parents as well
   * as some random changes. Applied to every chromosome in paths.
   * @return new potential path
   */
  std::vector<Node> Crossover() const;

  /**
   * @brief Mutation function to create that alters one of the paths found to
   * create a new path
   * @return new potential path
   */
  std::vector<Node> Mutate() const;

  /**
   * @brief Checks whether path is valid or not.
   * @param path Each chromosome represents a path by a sequence of moves.
   * @return bool value of validity of path
   */
  bool CheckPath(const std::vector<Node>& path) const;

  /**
   * @brief Checks whether every node in the path is an acceptable motion
   * primitive
   * @param path Each chromosome represents a path by a sequence of moves.
   * @return void
   */
  void CheckIfNodesInPathAreAcceptable(const std::vector<Node>& path) const;

 private:
  std::vector<Node> motions_;
  Node start_, goal_;
  size_t path_length_ = 30;
  int f_val = std::numeric_limits<int>::max();
  int generations_ = 10000;
  int popsize_ = 30;
  float c_ = 1.05;
  std::vector<std::vector<Node>> paths_;
  std::vector<std::vector<Node>> truepaths_;
  bool found_ = false;
  bool shorten_chromosome_ = false;
};

#endif  // GENETIC_ALGORITHM_H
