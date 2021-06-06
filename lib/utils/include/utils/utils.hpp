/**
 * @file utils.hpp
 * @author vss2sn
 * @brief Contains common/commonly used funtions and classes
 */

#ifndef UTILS_H
#define UTILS_H

#include <iostream>
#include <queue>
#include <unordered_set>
#include <vector>

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
class Node {
  // Variables used here are constantly accessed and checked; leaving public for
  // now.
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
  Node(const int x = 0, const int y = 0, const double cost = 0,
       const double h_cost = 0, const int id = 0, const int pid = 0);

  /**
   * @brief Prints the values of the variables in the node
   * @return void
   */
  void PrintStatus() const;

  /**
   * @brief Overloading operator + for Node class
   * @param p node
   * @return Node with current node's and input node p's values added
   */
  Node operator+(const Node& p) const;

  /**
   * @brief Overloading operator - for Node class
   * @param p node
   * @return Node with current node's and input node p's values subtracted
   */
  Node operator-(const Node& p) const;

  /**
   * @brief Overloading operator == for Node class
   * @param p node
   * @return bool whether current node equals input node
   */
  bool operator==(const Node& p) const;
};

/**
 * @brief Hash for node struct
 * @param n node for which the hash is to be calculated
 * @return hash value calculated
 */
template<>
class std::hash<Node> {
public:
  size_t operator () (const Node& n) const {
    return std::hash<int>()(n.x_) ^ std::hash<int>()(n.y_);
  }
};

/**
 * @brief Struct created to encapsulate function compare cost between 2 nodes.
 * Used in with multiple algorithms and classes
 */
struct compare_cost {
  /**
   * @brief Compare cost between 2 nodes
   * @param p1 Node 1
   * @param p2 Node 2
   * @return Returns whether cost to get to node 1 is greater than the cost to
   * get to node 2
   */
  bool operator()(const Node& p1, const Node& p2) const;
};

/**
 * @brief Get permissible motion primatives for the bot
 * @return vector of permissible motions
 */
std::vector<Node> GetMotion();

/**
 * @brief Prints the grid passed, highlighting the path taken
 * @param path_vector the path vector
 * @param start_ start node
 * @param goal_ goal node
 * @param grid Modify this grid
 * @return void
 */
void PrintPath(const std::vector<Node>& path_vector, const Node& start_,
               const Node& goal_, std::vector<std::vector<int>>& grid);

/**
 * @brief Prints out the cost for reaching points on the grid in the grid shape
 * @param grid Grid on which algorithm is running
 * @param point_list Vector of all points that have been considered. Nodes in
 * vector contain cost.
 * @return void
 */
void PrintCost(const std::vector<std::vector<int>>& grid,
               const std::vector<Node>& point_list);

/**
 * @brief Creates a random grid of a given size
 * @param grid Modify this grid
 * @return void
 */
void MakeGrid(std::vector<std::vector<int>>& grid);
/**
 * @brief Prints the grid passed, highlighting the path taken, when the vector
 * is the path taken in order
 * @param path_vector the path vector
 * @param start start node
 * @param goal goal node
 * @param grid Modify this grid
 * @return void
 */
void PrintPathInOrder(const std::vector<Node>& path_vector, const Node& start,
                      const Node& goal, std::vector<std::vector<int>>& grid);

/**
 * @brief compare coordinates between 2 nodes
 * @param p1 node 1
 * @param p2 node 2
 * @return whether the two nodes are for the same coordinates
 */
bool CompareCoordinates(const Node& p1, const Node& p2);

/**
 * @brief checks whether the node is outside the boundary of the grid
 * @param node node who's coordinates are to be checked
 * @param n size of the grid
 * @return whether the node is outside the boundary of the grid
 */
bool checkOutsideBoundary(const Node& node, const int n);

template<typename T,
         typename = typename std::enable_if<std::is_arithmetic<T>::value, T>::type
        >
void PrintGrid(const std::vector<std::vector<T>>& grid) {
  int n = grid.size();
  std::cout << "Grid: " << '\n'
            << "1. Points not considered ---> 0" << '\n'
            << "2. Obstacles             ---> 1" << '\n'
            << "3. Points considered     ---> 2" << '\n'
            << "4. Points in final path  ---> 3" << '\n';
  for (int j = 0; j < n; j++) {
    std::cout << "---";
  }
  std::cout << '\n';
  for (const auto& row : grid) {
    for (const auto& ele : row) {
      if (ele == 3) {
        std::cout << GREEN << ele << RESET << " , ";
      } else if (ele == 1) {
        std::cout << RED << ele << RESET << " , ";
      } else if (ele == 2) {
        std::cout << BLUE << ele << RESET << " , ";
      } else if( ele == std::numeric_limits<double>::max()) {
        std::cout << YELLOW << "I" << RESET << " , ";
      } else {
        std::cout << ele << " , ";
      }
    }
    std::cout << '\n' << '\n';
  }

  for (int j = 0; j < n; j++) {
    std::cout << "---";
  }
  std::cout << '\n';
}


#endif  // UTILS_H
