/**
 * @file test_utils.hpp
 * @author vss2sn
 * @brief Contains the test utils
 */

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

  // Variables for RRT and RRTStar
  constexpr double threshold = 2;
  constexpr int max_iter_x_factor = 20;

  // Variables for Ant Colony Optimization
  constexpr int n_ants = 10;
  constexpr int iterations = 50;
  constexpr float alpha = 1;
  constexpr float beta =0.7;
  constexpr float evap_rate = 0.3;
  constexpr float Q = 10;

  // Variables for genetic algorithm
  constexpr int generations = 10000;
  constexpr int popsize = 30;
  constexpr float c = 1.05;
  constexpr bool shorten_chromosome = true;

  if(algo =="dijkstra"){
    Dijkstra new_dijkstra;
    path_vector = new_dijkstra.dijkstra(grid, start, goal);
  }
  else if(algo == "a_star"){
    AStar new_a_star;
    path_vector = new_a_star.a_star(grid, start, goal);
  }
  else if(algo == "jump_point_search"){
    JumpPointSearch new_jump_point_search;
    path_vector = new_jump_point_search.jump_point_search(grid, start, goal);
  }
  else if(algo == "lpa_star"){
    LPAStar new_lpa_star;
    path_vector = new_lpa_star.lpa_star(grid, start, goal, 1, false);
  }
  else if(algo=="rrt"){
    RRT new_rrt;
    path_vector = new_rrt.rrt(grid, start, goal, max_iter_x_factor, threshold);
  }
  else if(algo=="rrtstar"){
    RRTStar new_rrt_star;
    path_vector = new_rrt_star.rrt_star(grid, start, goal, max_iter_x_factor, threshold);
  }
  else if(algo == "d_star_lite"){
    DStarLite new_d_star_lite;
    path_vector = new_d_star_lite.d_star_lite(grid, start, goal);
  }
  else if(algo == "ant_colony"){
    AntColony new_ant_colony(n_ants, alpha, beta, evap_rate, iterations, Q);
    path_vector = new_ant_colony.ant_colony(grid, start, goal);
  }
  else if(algo == "genetic_algorithm"){
    GeneticAlgorithm new_genetic_algorithm(generations, popsize, c, shorten_chromosome);
    path_vector = new_genetic_algorithm.genetic_algorithm(grid, start, goal, 2*static_cast<int>(start.h_cost_));
    if(path_vector[0].id_==-1) {
      return -1;
    }
    for(size_t i = 0; i < path_vector.size(); i++) {
      if(compareCoordinates(path_vector[i],goal)) {
        return i;
      }
    }
  }

  if(path_vector[0].cost_==-1) {
    return -1;
  }

  for(const auto& p : path_vector) {
    if(compareCoordinates(p, goal)) {
      return p.cost_;
    }
  }
  return -1;
}
