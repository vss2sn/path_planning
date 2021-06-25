/**
 * @file ant_colony.cpp
 * @author vss2sn
 * @brief Contains the Ant and Ant Colony class
 */

#include "path_planning/ant_colony.hpp"

#include <algorithm>
#include <chrono>
#include <climits>
#include <cmath>
#include <iostream>
#include <random>
#include <thread>
#include <tuple>

#ifdef CUSTOM_DEBUG_HELPER_FUNCION
void AntColony::PrintAntPath(Ant& ant) const {
  for (size_t k = 1; k < ant.path_.size(); k++)
    ant.path_[k].pid_ = ant.path_[k - 1].id_;
  ant.path_.back().id_ = ant.path_.back().x_ * n_ + ant.path_.back().y_;
  auto grid_2 = grid_;
  PrintPath(ant.path_, start_, ant.path_.back(), grid_2);
#ifndef RUN_TESTS
  std::this_thread::sleep_for(std::chrono::seconds(1));
#endif  // RUN_TESTS
}
#endif

void AntColony::RemoveLoop(Ant& ant) {
  for (auto it = ant.path_.begin(); it != ant.path_.end(); ++it) {
    if (CompareCoordinates(*it, ant.current_node_)) {
      ant.steps_ = std::distance(ant.path_.begin(), std::prev(it, 1));
      ant.path_.erase(it, ant.path_.end());
      break;
    }
  }
}

void AntColony::SetParams(const int n_ants, const double alpha,
                          const double beta, const double evap_rate,
                          const double Q, const int iterations) {
  n_ants_ = n_ants;
  alpha_ = alpha;
  beta_ = beta;
  evap_rate_ = evap_rate;
  Q_ = Q;
  iterations_ = iterations;
}

std::tuple<bool, std::vector<Node>> AntColony::Plan(const Node& start,
                                                    const Node& goal) {
  grid_ = original_grid_;
  start_ = start;  // Make sure start has id
  goal_ = goal;
  motions_ = GetMotion();
  ants_ = std::vector<Ant>(n_ants_);

  for (int i = 0; i < n_; i++) {
    for (int j = 0; j < n_; j++) {
      const int cur_id = i * n_ + j;
      const Node cur_node = Node(i, j);
      for (const auto& motion : motions_) {
        const Node c = cur_node + motion;
        if (!checkOutsideBoundary(c, n_)) {
          pheromone_edges_.insert(
              {std::make_pair(cur_id, c.x_ * n_ + c.y_), 1.0});
        }
      }
    }
  }

  // heuristically set max steps
  const int max_steps = n_ * n_ / 2 + n_;

  std::random_device device;
  std::mt19937 engine(device());

  // saves best path of last iteration. Not over all.
  std::vector<Node> last_best_path;
  Node possible_position;

  for (int i = 0; i < iterations_; i++) {
    for (int j = 0; j < n_ants_; j++) {
      // Can assign a thread to each ant if parallel required
      Ant ant(start_, j);
      while (!CompareCoordinates(ant.current_node_, goal_) &&
             ant.steps_ < max_steps) {
        ant.path_.push_back(ant.current_node_);

        // Get next position
        std::vector<Node> possible_positions;
        std::vector<double> possible_probabilities;
        double prob_sum = 0;
        for (const auto& m : motions_) {
          possible_position = ant.current_node_ + m;
          possible_position.id_ =
              possible_position.x_ * n_ + possible_position.y_;

          if (checkOutsideBoundary(possible_position, n_)) {
            continue;
          }

          if (CompareCoordinates(possible_position, ant.previous_node_) ||
              grid_[possible_position.x_][possible_position.y_] == 1) {
            continue;
          }

          if (CompareCoordinates(possible_position, goal_)) {
            ant.path_.push_back(goal_);
            ant.found_goal_ = true;
            break;
          }

          possible_positions.push_back(possible_position);
          double new_prob =
              std::pow(pheromone_edges_[std::make_pair(possible_position.id_,
                                                       ant.current_node_.id_)],
                       alpha_) *
              std::pow(
                  1.0 /
                      std::sqrt(std::pow((possible_position.x_ - goal_.x_), 2) +
                                std::pow((possible_position.y_ - goal_.y_), 2)),
                  beta_);
          possible_probabilities.push_back(new_prob);
          prob_sum += new_prob;
        }
        if (prob_sum == 0 || ant.found_goal_) {
          // Ant in a cul-de-sac or
          // no next node (ie start surrounded by obstacles)
          break;
        }

        std::for_each(possible_probabilities.begin(),
                      possible_probabilities.end(),
                      [&](double& p) { p /= prob_sum; });
        std::discrete_distribution<> dist(possible_probabilities.begin(),
                                          possible_probabilities.end());
        ant.previous_node_ = ant.current_node_;
        ant.current_node_ = possible_positions[dist(engine)];

        // Removing any loops if reached previously reached point
        // TODO(vss): add check to count number of loops removed and stop if
        // going into inf. Should be only 1? use hash of visited?
        RemoveLoop(ant);

        ant.steps_++;
      }

      ants_[j] = ant;
    }

    // Pheromone deterioration
    for (auto& pheromone_edge : pheromone_edges_) {
      pheromone_edge.second *= (1 - evap_rate_);
    }

    int bpl = std::numeric_limits<int>::max();
    std::vector<Node> bp;

    // Pheromone update based on successful ants
    for (const Ant& ant : ants_) {
      if (ant.found_goal_) {
        // Use iff goal reached
        if (static_cast<int>(ant.path_.size()) < bpl) {
          // Save best path yet in this iteration
          bpl = ant.path_.size();
          bp = ant.path_;
        }

        // c = cost / reward. Reward here, increased pheromone
        double c = Q_ / static_cast<double>(ant.path_.size() - 1);

        // start_.PrintStatus();
        // goal_.PrintStatus();
        for (size_t i_ant = 1; i_ant < ant.path_.size(); i_ant++) {
          // Assuming ant can tell which way the food was based on
          // how the phermones detereorate. Good for path planning as
          // prevents moving in the opposite direction to path and
          // improves convergence
          // std::cout << ant.path_[i_ant].id_ << ' ' <<  ant.path_[i_ant -
          // 1].id_ << '\n';
          auto it = pheromone_edges_.find(
              std::make_pair(ant.path_[i_ant].id_, ant.path_[i_ant - 1].id_));
          it->second += c;
        }
      }
    }
    if (i + 1 == iterations_) {
      last_best_path = bp;
    }

  }  // for every iteration loop ends here

  if (last_best_path.empty()) {
    return {false, {}};
  }
  for (size_t i = 1; i < last_best_path.size(); i++) {
    last_best_path[i].pid_ = last_best_path[i - 1].id_;
  }

  for (const auto& node : last_best_path) {
    node.PrintStatus();
  }
  return {true, last_best_path};
}

#ifdef BUILD_INDIVIDUAL
/**
 * @brief Script main function. Generates start and end nodes as well as grid,
 * then creates the algorithm object and calls the main algorithm function.
 * @return 0
 */
int main() {
  constexpr int n = 11;
  std::vector<std::vector<int>> grid(n, std::vector<int>(n, 0));
  MakeGrid(grid);

  std::random_device rd;   // obtain a random number from hardware
  std::mt19937 eng(rd());  // seed the generator
  std::uniform_int_distribution<int> distr(0, n - 1);  // define the range

  Node start(distr(eng), distr(eng), 0, 0, 0, 0);
  Node goal(distr(eng), distr(eng), 0, 0, 0, 0);

  start.id_ = start.x_ * n + start.y_;
  start.pid_ = start.x_ * n + start.y_;
  goal.id_ = goal.x_ * n + goal.y_;
  start.h_cost_ = abs(start.x_ - goal.x_) + abs(start.y_ - goal.y_);

  // Make sure start and goal are not obstacles and their ids are correctly
  // assigned.
  grid[start.x_][start.y_] = 0;
  grid[goal.x_][goal.y_] = 0;

  start.PrintStatus();
  goal.PrintStatus();

  PrintGrid(grid);

  // Normally as beta increases the solution becomes greedier. However, as the
  // heuristic is < 1 here, reducing beta increases the value placed on the
  // heuristic

  AntColony new_ant_colony(grid);
  // new_ant_colony.SetParams(10, 1, 0.2, 0.5, 10, 50);
  const auto [found_path, path_vector] = new_ant_colony.Plan(start, goal);
  PrintPath(path_vector, start, goal, grid);
  return 0;
}
#endif  // BUILD_INDIVIDUAL
