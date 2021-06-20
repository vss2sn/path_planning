/**
 * @file test_utils.hpp
 * @author vss2sn
 * @brief Contains the test utils
 */

#ifndef TEST_UTILS
#define TEST_UTILS

#include <gtest/gtest.h>

#include "path_planning/a_star.hpp"
#include "path_planning/ant_colony.hpp"
#include "path_planning/d_star_lite.hpp"
#include "path_planning/dijkstra.hpp"
#include "path_planning/genetic_algorithm.hpp"
#include "path_planning/jump_point_search.hpp"
#include "path_planning/lpa_star.hpp"
#include "path_planning/rrt.hpp"
#include "path_planning/rrt_star.hpp"

double run_test(std::vector<std::vector<int>>& grid, const std::string& algo){
  PrintGrid(grid);
  int n = grid.size();
  Node start(0,0,0,0,0,0);
  start.id_ = start.x_ * n + start.y_;
  start.pid_ = start.x_ * n + start.y_;
  Node goal(n-1,n-1,0,0,0,0);
  goal.id_ = goal.x_ * n +   goal.y_;
  start.h_cost_ = abs(start.x_ - goal.x_) + abs(start.y_ - goal.y_);
  //Make sure start and goal are not obstacles and their ids are correctly assigned.
  grid[start.x_][start.y_] = 0;
  grid[goal.x_][goal.y_] = 0;

  std::vector<Node> path_vector;

  if(algo =="dijkstra"){
    Dijkstra dijkstra(grid);
    const auto [path_found, path] = dijkstra.Plan(start, goal);
    path_vector = path;
  }
  else if(algo == "a_star"){
    AStar a_star(grid);
    const auto [path_found, path] = a_star.Plan(start, goal);
    path_vector = path;
  }
  else if(algo == "jump_point_search"){
    JumpPointSearch jump_point_search(grid);
    const auto [path_found, path] = jump_point_search.Plan(start, goal);
    path_vector = path;
  }
  else if(algo == "lpa_star"){
    LPAStar lpa_star(grid);
    const auto [path_found, path] = lpa_star.Plan(start, goal);
    path_vector = path;
  }
  else if(algo=="rrt"){
    RRT rrt(grid);
    const auto [path_found, path] = rrt.Plan(start, goal);
    path_vector = path;
  }
  else if(algo=="rrtstar"){
    RRTStar rrt_star(grid);
    const auto [path_found, path] = rrt_star.Plan(start, goal);
    path_vector = path;
  }
  else if(algo == "d_star_lite"){
    DStarLite d_star_lite(grid);
    const auto [path_found, path] = d_star_lite.Plan(start, goal);
    path_vector = path;
  }
  else if(algo == "ant_colony"){
    AntColony ant_colony(grid);
    const auto [path_found, path] = ant_colony.Plan(start, goal);
    path_vector = path;
  }
  else if(algo == "genetic_algorithm"){
    GeneticAlgorithm genetic_algorithm(grid);
    const auto [path_found, path] = genetic_algorithm.Plan(start, goal);
    path_vector = path;
    if(path_vector.empty()) {
      return -1;
    }
    for(size_t i = 0; i < path_vector.size(); i++) {
      if(CompareCoordinates(path_vector[i],goal)) {
        return i;
      }
    }
  }

  if(path_vector.empty()) {
    return -1;
  }

  for(const auto& p : path_vector) {
    if(CompareCoordinates(p, goal)) {
      return p.cost_;
    }
  }
  return -1;
}

#endif  // TEST_UTILS
