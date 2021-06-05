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

#include "utils/utils.hpp"

/**
 * @brief Stcut to hold key values for nodes used in D* Lite
 */
struct Key {
  double first;
  double second;

  /**
   * @brief Overload < operator for comparison
   * @param k key to be compared
   * @return result of comparison
   */
  bool operator<(const Key& k) const {
    return first < k.first || (first == k.first && second < k.second);
  }

  /**
   * @brief Overload < operator for comparison
   * @param k key to be compared
   * @return result of comparison
   */
  bool operator>(const Key& k) const {
    return first > k.first || (first == k.first && second > k.second);
  }

  /**
   * @brief Overload == operator for comparison
   * @param k key to be compared
   * @return result of comparison
   */
  bool operator==(const Key& k) const {
    return first == k.first && second == k.second;
  }

};

/**
 * @brief Struct to contain the Node Key pairs used by the priority queue in
 *        the D* Lite algorithm
 */
struct NodeKeyPair {
  Node node;
  Key key;
  /**
   * @brief Overload operator == for unordered_set
   * @param node key pair to be compared wth current node key pair
   * @return <return_description>
   * @details Compares whether the node in the struct has the same coordinates
   *          as the node from the one passed in as well as whether it has the
   *          same key
   */
  bool operator==(const NodeKeyPair& nkp) const { return node == nkp.node && key == nkp.key; }
};

template <>
class std::greater<Key> {
 public:
  /**
   * @brief Overload () operator for std::greater to use for comparison by
   *        priority queue
   * @param nk1 node key pair 1
   * @param nk1 node key pair 2
   * @return result of comparison
   * @details Compares the key values
   */
  bool operator()(const NodeKeyPair& nk1, const NodeKeyPair& nk2) const {
    return nk1.key > nk2.key;
  }
};

template <>
class std::greater<NodeKeyPair> {
 public:
   /**
    * @brief Overload () operator for std::greater to use for comparison by
    *        priority queue
    * @param nk1 node key pair 1
    * @param nk1 node key pair 2
    * @return result of comparison
    * @details Compares the key values
    */
  bool operator()(const NodeKeyPair& nk1, const NodeKeyPair& nk2) const {
    return nk1.key > nk2.key;
  }
};

template <>
class std::hash<NodeKeyPair> {
 public:
  /**
   * @brief Hash function for the node key pair
   * @param nkp node key pair who's jas is to be calculated
   * @return hash value
   */
  size_t operator()(const NodeKeyPair& nkp) const {
    return std::hash<Node>()(nkp.node);
  }
};

class DStarLite {
 public:
  /**
   * @brief Main function running the D* lite algorithm.
            Creates a path to be followed
            Steps trought the path in increments of 1 step along the path,
            rerouting as necessary
   * @param grid Grid on which the plan is to be made
   * @param start Start node
   * @param goal Goal node
   * @param create_random_obstacles should random obstacles be created during
    *       execution
   * @param time_discovered_obstacles obstacles to be discovered at specific
   *        times
   * @return The path taken
   */
  std::vector<Node> Plan(const std::vector<std::vector<int>>& grid,
                         const Node& start, const Node& goal,
                         const bool create_random_obstacles = false,
                         const std::unordered_map<int, std::vector<Node>>
                             time_discovered_obstacles = {});

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
   * @param Node for which neighbours are to be found
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
   * @param 21 node 2
   * @return the heuristic cost of travelling from one node 1 to node 2
   */
  double H(const Node& n1, const Node& n2) const;


  /**
   * @brief Create a square grid of size n and set each value to
   *        std::numeric_limits<double>::max()
   * @param n length of side
   * @return grid created
   */
  std::vector<std::vector<double>> CreateGrid(const int n);

  std::vector<std::vector<int>> grid_;
  std::vector<std::vector<double>> rhs_;
  std::vector<std::vector<double>> g_;
  std::unordered_map<int, std::vector<Node>> time_discovered_obstacles_;
  std::vector<Node> motions_;
  LazyPQ<NodeKeyPair> U_;
  Node start_, goal_, last_;
  double k_m_;
  Key k_old_;
  int n_;
  int time_step_ = 0;
  bool create_random_obstacles_;
};

#endif  // D_STAR_LITE_H
