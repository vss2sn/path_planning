/**
 * @file d_star_lite.hpp
 * @author vss2sn
 * @brief Contains the DStarLite class
 */

#ifndef D_STAR_LITE_H
#define D_STAR_LITE_H

#include <iostream>
#include <unordered_map>
#include <unordered_set>

#include "path_planning/planner.hpp"
#include "utils/utils.hpp"

/**
 * @brief Class for objects that plan using the D* Lite algorithm
 */
class DStarLite : public Planner {
 public:
  /**
   * @brief Constructor
   * @param grid the grid on which the planner is to plan
   * @return no return value
   */
  explicit DStarLite(std::vector<std::vector<int>> grid)
      : Planner(std::move(grid)) {}

  /**
   * @brief Sets the time discovered obstacles and the option that allows
            the creation of random obstacles
   * @param create_random_obstacles should random obstacles be created during
    *       execution
   * @param time_discovered_obstacles obstacles to be discovered at specific
   *        times
   * @return void
   * @details Set separately from the plan function to allow this to persist
              between calls to Plan()
   */
  void SetDynamicObstacles(const bool create_random_obstacles = false,
                           const std::unordered_map<int, std::vector<Node>>&
                               time_discovered_obstacles = {}) override;

  /**
   * @brief D* Lite algorithm implementation
   * @param start start node
   * @param goal goal node
   * @return tuple contatining a bool as to whether a path was found, and the
   path
   * @details Creates a path to be followed
              Steps through the path in increments of 1 time step
              Checks for discovered obstacles
              Replans as necessary
   */
  std::tuple<bool, std::vector<Node>> Plan(const Node& start,
                                           const Node& goal) override;

 private:
  /**
   * @brief Calculate the values of the key for a given node
   * @param s Node for whicht eh key values are to be calculated
   * @return Key containing values for s
   */
  Key CalculateKey(const Node& s) const;

  /**
   * @brief Get the predecessors of a node
   * @param u Node for which the predecessors are to be found
   * @return predecessors of node u
   * @details implemented for a grid with undirected edges,
              hence the predecessors are the neighbours
   */
  std::vector<Node> GetPred(const Node& u) const;

  /**
   * @brief Get the successors of a node
   * @param u Node for which the successors are to be found
   * @return predecessors of node u
   * @details implemented for a grid with undirected edges,
              hence the successors are the neighbours
   */
  std::vector<Node> GetSucc(const Node& u) const;

  /**
   * @brief Returns all the neighours for a given node
   * @param u Node for which neighbours are to be found
   * @return neighbours
   * @details all nodes that can be reached by use of a single motion primitive
   */
  std::vector<Node> GetNeighbours(const Node& u) const;

  /**
   * @brief Cost of traveresing the edge between 2 nodes
   * @param s1 Node 1
   * @param s2 Node 2
   * @return Edge cost
   * @details Implemented for an undirected grid, so the cost is same as the
              cost for the motion primitive, unless one of the nodes is an
              obstacle, in which case it is std::numeric_limits<double>::max()
   */
  double C(const Node& s1, const Node& s2) const;

  /**
   * @brief Initialize function based on D* Lite algorithm
   * @return void
   * @details Please see the algorithm implementation page
   *          for details about how the algorithm works
   */
  void Initialize();

  /**
   * @brief Update vertex function based on D* Lite algorithm
   * @param u The node who's values are to be updated
   * @return void
   * @details Please see the algorithm implementation page
   *          for details about how the algorithm works
   */
  void UpdateVertex(const Node& u);

  /**
   * @brief Compare key function based on D* Lite algorithm
   * @param p1 Key 1
   * @param p2 Key 2
   * @return result of comparison
   * @details Please see the algorithm implementation page
   *          for details about how the algorithm works
   */
  bool CompareKey(const Key& p1, const Key& p2) const;

  /**
   * @brief Compute shortest path function based on D* Lite algorithm
   * @return void
   * @details Please see the algorithm implementation page
   *          for details about how the algorithm works
   */
  void ComputeShortestPath();

  /**
   * @brief Detects wherether there are any changes in the grid.
   *        Generates obstacles if random obstacle generation is set to true.
   *        Creates obstacles based on the time discovered obstacles map.
   * @return vector of all nodes that have the changed in the graph
   *         at the given time step
   */
  std::vector<Node> DetectChanges();

  /**
   * @brief Checks whether the given node is an obstacle
   * @param n node
   * @return bool whether the given node is an obstacle
   */
  bool IsObstacle(const Node& n) const;

  /**
   * @brief Heuristic function based on D* Lite algorithm
   * @param n1 node 1
   * @param n2 node 2
   * @return the heuristic cost of travelling from one node 1 to node 2
   */
  static double H(const Node& n1, const Node& n2);

  /**
   * @brief Create a square grid of size n and set each value to
   *        std::numeric_limits<double>::max()
   * @return grid created
   */
  std::vector<std::vector<double>> CreateGrid();

  std::vector<std::vector<double>> rhs_;
  std::vector<std::vector<double>> g_;
  std::unordered_map<int, std::vector<Node>> time_discovered_obstacles_{};
  std::vector<Node> motions_{};
  LazyPQ U_;
  Node start_, goal_, last_;
  double k_m_ = 0;
  Key k_old_{0, 0};
  int time_step_ = 0;
  bool create_random_obstacles_ = false;
};

#endif  // D_STAR_LITE_H
