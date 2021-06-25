/**
 * @file ant_colony.hpp
 * @author vss2sn
 * @brief Contains the AntColony class
 */

#ifndef ANT_COLONY_H
#define ANT_COLONY_H

#include <tuple>
#include <unordered_map>

#include "path_planning/planner.hpp"
#include "utils/utils.hpp"

/**
 * @brief Class for Ant objects
 */
struct Ant {
  /**
   * @brief Constructor to create a new ant
   * @param start Start node of ant
   * @param id Ant id
   * @return no return value
   */
  Ant(const Node& start = Node(), const int id = 0)
      : id_(id), current_node_(start), previous_node_(Node(-1, -1)) {}

  bool found_goal_ = false;
  int id_;
  int steps_ = 0;
  Node current_node_;
  Node previous_node_;
  std::vector<Node> path_;
};

/**
 * @brief Class for objects that plan using the Ant Colony algorithm
 */
class AntColony : public Planner {
 public:
  /**
   * @brief Constructor
   * @param grid the grid on which the planner is to plan
   * @return no return value
   */
  explicit AntColony(const std::vector<std::vector<int>>& grid)
      : Planner(grid) {}

  /**
   * @brief Constructor for set up of Ant Colony class
   * @param n_ants number of ants yo be created in every iterations
   * @param alpha value of exponent for the use of the pheromone trails in
   * assigning probability to possible next positions
   * @param beta value of exponent for the use of the heuristic in assigning
   * probability to possible next positions
   * @param evap_rate evaporation rate of the pheromone trail as a value between
   * 0 and 1
   * @param iterations number of iterations of the simulation
   * @param Q Constant multiplication factor for the cost/reward function
   * @return no return value
   */
  void SetParams(const int n_ants = 10, const double alpha = 1,
                 const double beta = 0.2, const double evap_rate = 0.5,
                 const double Q = 10, const int iterations = 50);

  /**
   * @brief Prints the path taken by the ant.
   * @param ant ant for which the path is to be printed.
   * @return void
   * @details Prints the path taken by the ant on the grid, using the PrintPath
   * function in utils.cpp. It can be used to print incomplete paths as well, as
   * long as the end point and start point are specified.
   */
  void PrintAntPath(Ant& ant) const;

  /**
   * @brief Removes loops in path
   * @param ant Ant in whose path the loops are to be removed
   * @return void
   * @details Removes loops in path of an ant only when a point is revisted.
   */
  static void RemoveLoop(Ant& ant);

  /**
   * @brief Ant Colony algorithm implementation
   * @param start start node
   * @param goal goal node
   * @return tuple contatining a bool as to whether a path was found, and the
   * path
   */
  std::tuple<bool, std::vector<Node>> Plan(const Node& start,
                                           const Node& goal) override;

 private:
  int n_ants_ = 10;
  double alpha_ = 1;
  double beta_ = 0.2;
  double evap_rate_ = 0.5;
  double Q_ = 10;
  int iterations_ = 50;
  Node start_, goal_;
  std::vector<Ant> ants_;
  std::vector<Node> motions_;
  std::unordered_map<std::pair<int, int>, double, pair_hash> pheromone_edges_;
};

#endif  // ANT_COLONY_H
