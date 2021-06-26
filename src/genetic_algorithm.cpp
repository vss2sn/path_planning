/**
 * @file genetic_algorithm.cpp
 * @author vss2sn
 * @brief Contains the GeneticAlgorithm class
 */

#include "path_planning/genetic_algorithm.hpp"

#include <algorithm>
#include <iomanip>  // TODO(vss): replace setw
#include <iostream>
#include <limits>
#include <random>

constexpr int random_range_max = 100;
constexpr int inverted_probabilty_mutate = 10;

void GeneticAlgorithm::SetParams(const int generations, const int popsize,
                                 const double c, const bool shorten_chromosome,
                                 const int path_length) {
  generations_ = generations;
  popsize_ = popsize;
  c_ = c;
  shorten_chromosome_ = shorten_chromosome;
  path_length_ = path_length;
}

std::tuple<bool, std::vector<Node>> GeneticAlgorithm::Plan(const Node &start,
                                                           const Node &goal) {
  int generation = 0;
  start_ = start;
  goal_ = goal;
  grid_ = original_grid_;
  motions_ = GetMotion();

  // Create initial chromosome
  std::vector<Node> initial_path = GenerateSimplePath();
  paths_.push_back(initial_path);

  // Check first path to goal
  if (CheckPath(initial_path)) {
    truepaths_.push_back(initial_path);
  }

  // Allow while loop to continue beyond path found to find optimum path
  while (generation <= generations_) {
    size_t paths_size = paths_.size();
    std::vector<int> f_vals(paths_size);
    std::transform(
        paths_.begin(), paths_.end(), f_vals.begin(),
        [&](const std::vector<Node> &path) { return CalculateFitness(path); });

    f_val = *std::min_element(f_vals.begin(), f_vals.end());
    std::vector<std::vector<Node>> new_paths_;

    for (int i = 0; i < paths_.size(); ++i) {
      // c provides a margin of error from best path
      if (f_vals[i] <= f_val * c_) {
        if (shorten_chromosome_) {
          const int len = std::min(paths_[i].size(), path_length_);
          new_paths_.emplace_back(std::vector<Node>(
              paths_[i].begin(), std::next(paths_[i].begin(), len)));
        } else {
          new_paths_.push_back(paths_[i]);
        }
      }
    }

    if (!new_paths_.empty()) {
      std::swap(paths_, new_paths_);
    }

    if (paths_.size() < popsize_) {
      while (paths_.size() < popsize_) {
        if (rand() % inverted_probabilty_mutate == 0) {
          paths_.emplace_back(Mutate());
        } else {
          paths_.emplace_back(Crossover());
        }
      }
    } else {
      paths_[rand() % paths_.size()] = Mutate();
    }

    // TODO(vss): Consider dynamically modifying path length to move towards
    // optimality
    for (const auto &path_seen : paths_) {
      int tmp_length = std::numeric_limits<int>::max();
      if (CheckPath(path_seen)) {
        found_ = true;
        if (truepaths_.size() > (n_ * n_)) {
          truepaths_[rand() % (n_ * n_)] = path_seen;
        } else {
          truepaths_.push_back(path_seen);
        }
        if (shorten_chromosome_) {
          auto tmp = start_;
          if (path_seen.size() < tmp_length) {
            tmp_length = path_seen.size();
          }
          if (CompareCoordinates(tmp, goal_)) {
            tmp_length = 0;
          }
          for (size_t i = 0; i < path_seen.size(); i++) {
            tmp = tmp + path_seen[i];
            if (CompareCoordinates(tmp, goal_)) {
              tmp_length = i;
              break;
            }
          }
          path_length_ = tmp_length;
        }
      }
      // PrintChromosome(path_seen);
    }
    generation++;
  }

  // std::cout << "True paths: " << truepaths_.size() << '\n';
  // if(!truepaths_.empty())std::cout << "True paths: " << '\n';
  // for(int i=0;i<truepaths_.size();i++) PrintPathOfChromosome(truepaths_[i]);

  const auto node_path = ReturnLastPath();
  return {!node_path.empty(), node_path};
}

std::vector<Node> GeneticAlgorithm::ReturnLastPath() const {
  if (truepaths_.empty()) {
    return {};
  }

  std::vector<int> f_vals(truepaths_.size());
  std::transform(truepaths_.begin(), truepaths_.end(), f_vals.begin(),
                 f_vals.begin(),
                 [&](const std::vector<Node> &path, const int /* f_vals_v */) {
                   return CalculateFitness(path);
                 });
  const int best_path_index = std::distance(
      f_vals.begin(), std::min_element(f_vals.begin(), f_vals.end()));
  std::vector<Node> node_path;
  node_path.push_back(start_);
  node_path.back().pid_ = node_path.back().id_;
  Node current = start_;
  for (const auto &motion : truepaths_[best_path_index]) {
    current = current + motion;
    current.pid_ = node_path.back().id_;
    current.id_ = n_ * current.x_ + current.y_;
    node_path.push_back(current);
    if (CompareCoordinates(current, goal_)) {
      break;
    }
  }
  return node_path;
}

#ifdef CUSTOM_DEBUG_HELPER_FUNCION
void GeneticAlgorithm::CheckIfNodesInPathAreAcceptable(
    const std::vector<Node> &path) const {
  for (const auto &motion : path) {
    bool found = false;
    for (const auto &m : motions_) {
      if (CompareCoordinates(m, motion)) {
        found = true;
        break;
      }
    }
    if (!found) {
      motion.PrintStatus();
      exit(0);
    }
  }
}

void GeneticAlgorithm::PrintChromosome(const std::vector<Node> &path) const {
  std::cout << "Chromosome: ";
  for (const auto &v : path) {
    for (size_t i = 0; i < motions_.size(); i++)
      if (CompareCoordinates(v, motions_[i])) {
        std::cout << i << " ";
      }
  }
  std::cout << "Fitness value: " << CalculateFitness(path) << '\n';
}

void GeneticAlgorithm::PrintPathOfChromosome(
    const std::vector<Node> &path) const {
  std::cout << "Path: ";
  Node tmp = start_;
  std::cout << "(" << tmp.x_ << "," << tmp.y_ << ")" << std::setw(3);
  for (auto v : path) {
    tmp = tmp + v;
    std::cout << "(" << tmp.x_ << "," << tmp.y_ << ")" << std::setw(3);
  }
  std::cout << '\n';
}
#endif  // CUSTOM_DEBUG_HELPER_FUNCION

std::vector<Node> GeneticAlgorithm::GenerateSimplePath() const {
  int d_x = goal_.x_ - start_.x_;
  int d_y = goal_.y_ - start_.y_;
  Node dx(d_x >= 0 ? 1 : -1, 0);
  Node dy(0, d_y >= 0 ? 1 : -1);
  std::vector<Node> path(path_length_);
  auto it = path.begin();
  std::fill_n(it, std::abs(d_x), dx);
  std::advance(it, std::abs(d_x));
  std::fill_n(it, std::abs(d_y), dy);
  std::advance(it, std::abs(d_y));
  std::generate_n(it, std::distance(it, path.end()),
                  [&]() { return motions_[rand() % 4]; });
  return path;
}

int GeneticAlgorithm::CalculateFitness(const std::vector<Node> &path) const {
  int cost = 0;
  Node i = start_;
  for (const auto &p : path) {
    i = i + p;
    if (i.x_ < 0 || i.x_ >= n_ || i.y_ < 0 || i.y_ >= n_) {
      return std::numeric_limits<int>::max();
    }
    if (CompareCoordinates(i, goal_)) {
      break;
    }
    if (grid_[i.x_][i.y_] == 1) {
      cost += n_ * abs(goal_.x_ - i.x_) + n_ * abs(goal_.y_ - i.y_);
    } else {
      // Can add a scaling factor here
      cost += abs(goal_.x_ - i.x_) + abs(goal_.y_ - i.y_);
    }
  }
  return cost;
}

std::vector<Node> GeneticAlgorithm::GenerateRandomPath() const {
  std::random_device rd;
  std::mt19937 eng(rd());
  std::vector<Node> path(path_length_);
  if (path_length_ > 0) {
    std::uniform_int_distribution<int> distr(
        0, motions_.size() - 1);  // define the range
    std::generate_n(path.begin(), path_length_,
                    [&]() { return motions_[distr(eng)]; });
  }
  return path;
}

std::vector<Node> GeneticAlgorithm::Mutate() const {
  std::random_device rd;
  std::mt19937 eng(rd());
  std::vector<Node> path;
  if (truepaths_.size() > 3 && rand() % 4 == 0) {
    path = truepaths_[rand() % truepaths_.size()];
  } else {
    path = GenerateRandomPath();
  }
  if (path.empty()) {
    return path;
  }
  int index_1 = static_cast<int>(rand() % path.size());
  int index_2 = static_cast<int>(rand() % path.size());
  if (index_2 < index_1) {
    std::swap(index_1, index_2);
  }
  std::shuffle(path.begin() + index_1, path.begin() + index_2, eng);
  return path;
}

std::vector<Node> GeneticAlgorithm::Crossover() const {
  int p1 = static_cast<int>(rand() % paths_.size());
  int p2 = static_cast<int>(rand() % paths_.size());

  const size_t len =
      std::max(std::max(paths_[p1].size(), paths_[p2].size()), path_length_);
  std::vector<Node> child;
  child.reserve(len);

  Node current = start_;

  int index = 0;
  int attempt_count = 0;

  while (index < len) {
    int random_int = rand() % random_range_max;
    Node motion;
    if (random_int < random_range_max / 4 && paths_[p1].size() > index) {
      motion = paths_[p1][index];
    } else if (random_int < random_range_max / 2 && paths_[p2].size() > index) {
      motion = paths_[p2][index];
    } else {
      motion = motions_[rand() % motions_.size()];
    }
    // Prevents the new chromosome from going beyond the grid
    // Added as a very large percentage (majority) of these are out of bounds,
    // and a waste of resources Chromosomes that contain paths travelling over
    // obstacles are still allowed after a point
    if (auto new_node = current + motion; !checkOutsideBoundary(new_node, n_)) {
      if (grid_[new_node.x_][new_node.y_] != 1 || attempt_count > n_ * n_) {
        current = new_node;
        child.push_back(motion);
        ++index;
      }
    }
    ++attempt_count;
  }
  return child;
}

// NOTE: Consider storing the point where an obstacle is encountereed and forcig
// that gene/motion to randomly mutate for a quicker convergence to a solution
// while maintaining randomness
bool GeneticAlgorithm::CheckPath(const std::vector<Node> &path) const {
  Node current = start_;
  if (CompareCoordinates(current, goal_)) {
    return true;
  }
  for (const auto &node : path) {
    current = current + node;
    if (CompareCoordinates(current, goal_)) {
      return true;
    }
    if (checkOutsideBoundary(current, n_)) {
      return false;
    }
    if (grid_[current.x_][current.y_] == 1) {
      return false;
    }
  }
  return CompareCoordinates(current, goal_);
}

#ifdef BUILD_INDIVIDUAL
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

  constexpr int generations = 10000;
  constexpr int popsize = 30;
  constexpr double c = 1.05;
  constexpr bool shorten_chromosome = true;
  constexpr int path_length_x_factor = 4;

  GeneticAlgorithm new_genetic_algorithm(grid);
  new_genetic_algorithm.SetParams(
      generations, popsize, c, shorten_chromosome,
      static_cast<int>(path_length_x_factor * start.h_cost_));
  const auto [path_found, path_vector] =
      new_genetic_algorithm.Plan(start, goal);
  PrintPathInOrder(path_vector, start, goal, grid);
}
#endif  // BUILD_INDIVIDUAL
