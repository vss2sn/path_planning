/**
* @file utils.hpp
* @author vss2sn
* @brief Contains common/commonly used funtions and classes
*/

#ifndef UTILS_H
#define UTILS_H

#include "main.hpp"

#define BLACK "\x1b[1;30m"
#define RED "\x1b[1;31m"
#define GREEN "\x1b[1;32m"
#define YELLOW "\x1b[1;33m"
#define BLUE "\x1b[1;34m"
#define MAGENTA "\x1b[1;35m"
#define CYAN "\x1b[1;36m"
#define WHITE "\x1b[1;37m"
#define RESET "\x1b[1;0m"

/**
* @brief Node class
* @param x_ X value
* @param y_ Y value
* @param cost_ Cost to get to this node
* @param h_cost_ Heuritic cost of this node
* @param id_ Node's id
* @param pid_ Node's parent's id
*/
class Node{
// Variables used here are constantly accessed and checked; leaving public for now.
public:
    /** \brief x coordinate */
  int x_;
    /** \brief y coordinate */
  int y_;
  /** \brief Node id */
  int id_;
  /** \brief Node's parent's id */
  int pid_;
  /** \brief cost to reach this node */
  double cost_;
  /** \brief heuristic cost to reach the goal */
  double h_cost_;

  /**
  * @brief Constructor for Node class
  * @param x X value
  * @param y Y value
  * @param cost Cost to get to this node
  * @param h_cost Heuritic cost of this node
  * @param id Node's id
  * @param pid Node's parent's id
  */
  Node(int x = 0, int y = 0, double cost = 0, double h_cost = 0, int id = 0, int pid = 0);

  /**
  * @brief Prints the values of the variables in the node
  * @return void
  */
  void PrintStatus();

  /**
  * @brief Overloading operator + for Node class
  * @param p node
  * @return Node with current node's and input node p's values added
  */
  Node operator+(Node p);

  /**
  * @brief Overloading operator - for Node class
  * @param p node
  * @return Node with current node's and input node p's values subtracted
  */
  Node operator-(Node p);

  /**
  * @brief Overloading operator = for Node class
  * @param p node
  * @return void
  */
  void operator=(Node p);

  /**
  * @brief Overloading operator == for Node class
  * @param p node
  * @return bool whether current node equals input node
  */
  bool operator==(Node p);

  /**
  * @brief Overloading operator != for Node class
  * @param p node
  * @return bool whether current node is not equal to input node
  */
  bool operator!=(Node p);
};

/**
* @brief Struct created to encapsulate function compare cost between 2 nodes. Used in with multiple algorithms and classes
*/
struct compare_cost{

  /**
  * @brief Compare cost between 2 nodes
  * @param p1 Node 1
  * @param p2 Node 2
  * @return Returns whether cost to get to node 1 is greater than the cost to get to node 2
  */
  bool operator()(Node& p1, Node& p2);
};

/**
* @brief Get permissible motion primatives for the bot
* @return vector of permissible motions
*/
std::vector<Node> GetMotion();

/**
* @brief Prints the grid passed
* @param grid Modify this grid
* @param n number of rows/columns
* @return void
*/
void PrintGrid(std::vector<std::vector<int>> &grid);

/**
* @brief Prints the grid passed
* @param path_vector the path vector
* @param start_ start node
* @param goal_ goal node
* @param grid Modify this grid
* @param n number of rows/columns
* @return void
*/
void PrintPath(std::vector<Node> path_vector, Node start_, Node goal_, std::vector<std::vector<int>> &grid);

/**
* @brief Prints out the cost for reaching points on the grid in the grid shape
* @param grid Grid on which algorithm is running
* @param n Number of rows/columns
* @param point_list Vector of all points that have been considered. Nodes in vector contain cost.
* @return void
*/
void PrintCost(std::vector<std::vector<int>> &grid, std::vector<Node> point_list);

/**
* @brief Creates a random grid of a given size
* @param grid Modify this grid
* @param n number of rows/columns
* @return void
*/
void MakeGrid(std::vector<std::vector<int>> &grid);
#endif UTILS_H
