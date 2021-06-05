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

struct Key {
  double first;
  double second;

  bool operator<(const Key& k) const {
    return first < k.first || (first == k.first && second < k.second);
  }

  bool operator>(const Key& k) const {
    return first > k.first || (first == k.first && second > k.second);
  }
};

struct NodeKeyPair {
  Node node;
  Key key;

  bool operator==(const NodeKeyPair& nkp) const { return node == nkp.node; }
};

template <>
class std::greater<Key> {
 public:
  bool operator()(const NodeKeyPair& nk1, const NodeKeyPair& nk2) const {
    return nk1.key > nk2.key;
  }
};

template <>
class std::greater<NodeKeyPair> {
 public:
  bool operator()(const NodeKeyPair& nk1, const NodeKeyPair& nk2) const {
    return nk1.key > nk2.key;
  }
};

template <>
class std::hash<NodeKeyPair> {
 public:
  size_t operator()(const NodeKeyPair& nkp) const {
    return std::hash<Node>()(nkp.node);
  }
};

class DStarLite {
 public:
  std::vector<Node> Plan(const std::vector<std::vector<int>>& grid,
                         const Node& start, const Node& goal,
                         const bool create_random_obstacles = false,
                         const std::unordered_map<int, std::vector<Node>>
                             time_discovered_obstacles = {});

 private:
  Key CalculateKey(const Node& s) const;
  std::vector<Node> GetPred(const Node& u) const;
  std::vector<Node> GetSucc(const Node& u) const;
  std::vector<Node> GetNeighbours(const Node& u) const;
  double C(const Node& s1, const Node& s2) const;
  void Initialize();
  void UpdateVertex(const Node& u);
  bool CompareKey(const Key& p1, const Key& p2) const;
  void ComputeShortestPath();
  std::vector<Node> DetectChanges();
  bool IsObstacle(const Node& n) const;
  double H(const Node& n1, const Node& n2) const;
  std::vector<std::vector<double>> CreateGrid(const int n);

  std::vector<std::vector<int>> grid_;
  std::vector<std::vector<double>> rhs_;
  std::vector<std::vector<double>> g_;
  std::unordered_map<int, std::vector<Node>> time_discovered_obstacles_;
  // std::unordered_map<int, std::vector<Node>> discovered_obstacles;
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
