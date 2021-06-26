/**
 * @file planner.hpp
 * @author vss2sn
 * @brief Contains the abstract planner class
 */

#ifndef PLANNER_HPP
#define PLANNER_HPP

#include <iostream>
#include <tuple>
#include <unordered_map>
#include <vector>

#include "utils/utils.hpp"

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
  Planner(std::vector<std::vector<int>> grid)
      : original_grid_(std::move(grid)), n_(original_grid_.size()){};

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
   * @return tuple contatining a bool as to whether a path was found, and the
   * path
   */
  virtual std::tuple<bool, std::vector<Node>> Plan(const Node& start,
                                                   const Node& goal) = 0;

  /**
   * @brief Sets the time discovered obstacles and the option that allows
            the creation of random obstacles
   * @param create_random_obstacles should random obstacles be created during
   *        execution
   * @param time_discovered_obstacles obstacles to be discovered at specific
   *        times
   * @return void
   * @details Set separately from the plan function to allow this to persist
              between calls to Plan()
   */
  virtual void SetDynamicObstacles(
      const bool create_random_obstacles = false,
      const std::unordered_map<int, std::vector<Node>>&
          time_discovered_obstacles = {}) {
    std::cout << "Please implement this function for the planner" << '\n';
    std::cout << "Value attempted to be set: " << '\n';
    std::cout << "Create random obstacles: " << create_random_obstacles << '\n';
    std::cout << "Number of time discovered obstacles: "
              << time_discovered_obstacles.size() << '\n';
  };

 protected:
  std::vector<std::vector<int>> grid_ = {};
  const std::vector<std::vector<int>> original_grid_;
  const int n_;
};

#endif  // PLANNER_HPP
