/**
* @file ant_colony.hpp
* @author vss2sn
* @brief Contains the AntColony class
*/

#ifndef ANT_COLONY_H
#define ANT_COLONY_H

#include "utils.hpp"


struct pair_hash
{
	template <class T1, class T2>
	std::size_t operator() (const std::pair<T1, T2> &pair) const
	{
		return std::hash<T1>()(pair.first) ^ std::hash<T2>()(pair.second);
	}
};

class Ant{
public:
  bool found_goal = false;
  int steps = 0, id;
  std::vector<Node> path;
  Node previous_node;
  Node current_node;
  Ant(Node start = Node(0,0), int id = 0);
};

class AntColony{
  std::vector<std::vector<int>> grid;
  std::unordered_map<std::pair<int,int>,double, pair_hash> pheromone_edges;
  int n_ants, iterations, max_steps, grid_size;
  double alpha, beta, evap_rate, Q;
  Node start, goal;
  std::vector<Ant> ants;
	std::vector<Node> motions;
public:
  AntColony(std::vector<std::vector<int>>& grid, Node start, Node goal, int n_ants = 10, double alpha = 0.5, double beta = 0.5, double evap_rate = 0.5, int iterations = 10, double Q = 10.0);

	void PrintAntPath(Ant& ant);

	void RemoveLoop(Ant& ant);

  std::vector<Node> ant_colony();
};

#endif ANT_COLONY_H
