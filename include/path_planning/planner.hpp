#include <utils/utils.hpp>

#include <vector>
#include <tuple>

/**
 *  Abstract class that is inherited by concerete implementaions of planner
 *  classes. The Plan function is a pure virtual funciton that is overloaded
 */
class Planner {
public:
  /**
   * @brief Constructor
   * @param grid the grid on which the planner is to plan
   * @return no return value
   */
  Planner (std::vector<std::vector<int>> grid) : original_grid_(std::move(grid)), n_(original_grid_.size()) {};

  /**
   * @brief Copy constructor
   * @return No return value
   * @details default
   */
  Planner(const Planner&) = default;

  /**
   * @brief Move constructor
   * @return No return value
   * @details default
   */
  Planner(Planner&&) = default;

  /**
   * @brief Copy assignment
   * @return No return value
   * @details default
   */
  Planner& operator=(const Planner&) = default;

  /**
   * @brief Move assignment
   * @return No return value
   * @details default
   */
  Planner& operator=(Planner&&) = default;

  /**
   * @brief Virtual destructor
   * @return No return value
   * @details default
   */

  virtual ~Planner() = default;

  /**
   * @brief Pure virtual function that is overloadde by planner implementations
   * @param start start node
   * @param goal goal node
   * @return tuple contatining a bool as to whether a path was found, and the path
   */
  virtual std::tuple<bool, std::vector<Node>> Plan(const Node& start, const Node& goal) = 0;
protected:
  std::vector<std::vector<int>> grid_ = {};
  const std::vector<std::vector<int>> original_grid_;
  const int n_;
};
