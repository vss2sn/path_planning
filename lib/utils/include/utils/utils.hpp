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
  /** \brief cost to reach this node */
  double cost_;
  /** \brief heuristic cost to reach the goal */
  double h_cost_;
  /** \brief Node id */
  int id_;
  /** \brief Node's parent's id */
  int pid_;

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
       const double h_cost = 0, const int id = 0, const int pid = 0) :
       x_(x), y_(y), cost_(cost), h_cost_(h_cost), id_(id), pid_(pid) {}

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
 */
template<>
class std::hash<Node> {
public:

  /**
   * @brief Overlaod () operator to calculate the hash of a Node
   * @param n Node for which the hash is to be calculated
   * @return hash value
   */
  size_t operator () (const Node& n) const {
    return std::hash<int>()(n.x_) ^ std::hash<int>()(n.y_);
  }
};

/**
 * @brief Hash for node struct that returns node id
 */
class NodeIdAsHash {
public:

  /**
   * @brief Overlaod () operator to calculate the hash of a Node
   * @param n Node for which the hash is to be calculated
   * @return hash value
   * @details the hash returned is the node id
   */
  size_t operator () (const Node& n) const {
    return n.id_;
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
 * @brief Struct created to encapsulate function compare cost between 2 nodes.
 * Used in with multiple algorithms and classes
 */
struct compare_coordinates {
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
      if (ele == 1) {
        std::cout << RED << ele << RESET << " , ";
      } else if (ele == 2) {
        std::cout << BLUE << ele << RESET << " , ";
      } else if (ele == 3) {
        std::cout << GREEN << ele << RESET << " , ";
      } else if (ele == 4) {
        std::cout << YELLOW << ele << RESET << " , ";
      } else if( ele == std::numeric_limits<double>::max()) {
        std::cout << CYAN << "I" << RESET << " , ";
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

/**
 * @brief Structure to generate a hash for std::pair
 * @details This allows the use of pairs in data structures that use a hash,
 * such as unordered_map/set
 */
struct pair_hash {
  /**
   * @brief Function used to generate hash for keys
   * @param pair pair of values
   * @return generated hash value
   */
  template <class T1, class T2>
  std::size_t operator()(const std::pair<T1, T2>& pair) const {
    return std::hash<T1>()(pair.first) ^ std::hash<T2>()(pair.second);
  }
};

/**
 * @brief Struct to hold key values for nodes used in D* Lite
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

  /**
   * @brief Overload != operator for comparison
   * @param k key to be compared
   * @return result of comparison
   */
  bool operator!=(const Key& k) const {
    return !(first == k.first && second == k.second);
  }
};

/**
 * @brief Struct to contain the Node Key pairs used by the priority queue in
 *        the D* Lite algorithm
 */
struct NodeKeyPair {
  Node node;
  Key key;
};

struct CompareNodeKeyPairKeys {
  bool operator()(const NodeKeyPair& nkp1, const NodeKeyPair& nkp2) const {
    return nkp1.key == nkp2.key;
  }
};

struct CompareNodeKeyPairCoordinates {
  bool operator()(const NodeKeyPair& nkp1, const NodeKeyPair& nkp2) const {
    return CompareCoordinates(nkp1.node, nkp2.node);
  }
};

struct CompareNodeKeyPairCoordinatesAndKeys {
  bool operator()(const NodeKeyPair& nkp1, const NodeKeyPair& nkp2) const {
    return CompareCoordinates(nkp1.node, nkp2.node) && nkp1.key == nkp2.key;
  }
};

template <>
class std::greater<NodeKeyPair> {
 public:
   /**
    * @brief Overload () operator for std::greater to use for comparison by
    *        priority queue
    * @param nk1 node key pair 1
    * @param nk2 node key pair 2
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

/**
 * @brief The idea behind this class is to create a structure similar to a
 * priority queue that allows elements to be removed from the middle of the
 * queue as well, rather than just the top
 */
class LazyPQ {
public:

  /**
   * @brief Clear the LazyPQ
   * @return void
   */
  void clear();

  /**
   * @brief Insert into the LazyPQ
   * @return void
   */
  void insert(const NodeKeyPair& t);

  /**
   * @brief pop the top element from the LazyPQ
   * @return void
   */
  void pop();

  /**
   * @brief Returns the top element of the LazyPQ
   * @return reference to the top value in the LazyPQ
   */
  const NodeKeyPair& top() const;

  /**
   * @brief Number of elements in the LazyPQ
   * @return void
   */
  size_t size() const;

  /**
   * @brief Checks whether the LazyPQ is empty
   * @return bool whether the LazyPQ is empty
   */
  bool empty() const;

  /**
   * @brief Checks whether the element is in the LazyPQ
   * @return bool whether the element is in the LazyPQ
   */
  bool isElementInStruct(const NodeKeyPair& t) const;

  /**
   * @brief Remove an element from the LazyPQ if it exists
   * @return void
   */
  void remove(const NodeKeyPair& t);

private:
  std::priority_queue<NodeKeyPair, std::vector<NodeKeyPair>, std::greater<NodeKeyPair>> pq;
  std::unordered_set<NodeKeyPair, std::hash<NodeKeyPair>, CompareNodeKeyPairCoordinates> s; // Needs to just compare the coordinates and
};

#endif  // UTILS_H
