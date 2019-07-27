/**
* @file ant_colony.hpp
* @author vss2sn
* @brief Contains the AntColony class
*/

#ifndef ANT_COLONY_H
#define ANT_COLONY_H

#include "utils.hpp"

/**
* @brief Structure to generate a hash for std::pair
* @details This allows the use of pairs in data structures that use a hash, such as unordered_map/set
*/
struct pair_hash
{
	/**
	* @brief Function used to generate hash for keys
	* @param pair pair of values
	* @return generated hash value
	*/
	template <class T1, class T2>
	std::size_t operator() (const std::pair<T1, T2> &pair) const
	{
		return std::hash<T1>()(pair.first) ^ std::hash<T2>()(pair.second);
	}
};

/**
* @brief Class for Ant objects
*/
class Ant{
public:
	/**
	* @brief Constructor to create a new ant
	* @param start Start node of ant
	* @param id Ant id
	* @return no return value
	*/
  Ant(Node start = Node(0,0), int id = 0);
private:
	bool found_goal_ = false;
	int steps_ = 0, id_;
	std::vector<Node> path_;
	Node previous_node_;
	Node current_node_;
};

/**
* @brief Class for Ant Colony objects
*/
class AntColony{
public:
	/**
	* @brief Constructor for set up of Ant Colony class
	* @param n_ants number of ants yo be created in every iterations
	* @param alpha value of exponent for the use of the pheromone trails in assigning probability to possible next positions
	* @param beta value of exponent for the use of the heuristic in assigning probability to possible next positions
	* @param evap_rate evaporation rate of the pheromone trail as a value between 0 and 1
	* @param iterations number of iterations of the simulation
	* @param Q Constant multiplication factor for the cost/reward function
	* @return no return value
	*/
  AntColony(int n_ants = 10, double alpha = 0.5, double beta = 0.5, double evap_rate = 0.5, int iterations = 10, double Q = 10.0);

	/**
	* @brief Prints the path taken by the ant.
	* @param ant ant for which the path is to be printed.
	* @return void
	* @details Prints the path taken by the ant on the grid, using the PrintPath function in utils.cpp. It can be used to print incomplete paths as well, as long as the end point and start point are specified.
	*/
	void PrintAntPath(Ant& ant);

	/**
	* @brief Removes loops in path
	* @param ant Ant in whose path the loops are to be removed
	* @return void
	* @details Removes loops in path of an ant only when a point is revisted.
	*/
	void RemoveLoop(Ant& ant);

	/**
	* @brief Main algorithm of ant colony optimization
	* @param grid Main grid
	* @param start start node
	* @param goal goal node
	* @return best path within last iteration of the ant colony
	*/
  std::vector<Node> ant_colony(std::vector<std::vector<int>>& grid, Node start, Node goal);
private:
	std::vector<std::vector<int>> grid_;
	std::unordered_map<std::pair<int,int>,double, pair_hash> pheromone_edges_;
	int n_ants_, iterations_, max_steps_, grid_size_;
	double alpha_, beta_, evap_rate_, Q_;
	Node start_, goal_;
	std::vector<Ant> ants_;
	std::vector<Node> motions_;
};

#endif ANT_COLONY_H
