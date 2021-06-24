#include "path_planning/d_star_lite.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>

#ifdef BUILD_INDIVIDUAL
#include <random>
#endif  // BUILD_INDIVIDUAL

constexpr int pause_time = 500;  // milliseconds

bool DStarLite::IsObstacle(const Node& n) const {
  return grid_[n.x_][n.y_] == 1;
}

double DStarLite::H(const Node& n1, const Node& n2) {
  return std::sqrt(std::pow(n1.x_ - n2.x_, 2) + std::pow(n1.y_ - n2.y_, 2));
}

std::vector<Node> DStarLite::GetNeighbours(const Node& u) const {
  std::vector<Node> neighbours;
  for (const auto& m : motions_) {
    if (const auto neighbour = u + m;
        !checkOutsideBoundary(neighbour, grid_.size())) {
      neighbours.push_back(neighbour);
    }
  }
  return neighbours;
}

std::vector<Node> DStarLite::GetPred(const Node& u) const {
  return GetNeighbours(u);
}

std::vector<Node> DStarLite::GetSucc(const Node& u) const {
  return GetNeighbours(u);
}

double DStarLite::C(const Node& s1, const Node& s2) const {
  if (IsObstacle(s1) || IsObstacle(s2)) {
    return std::numeric_limits<double>::max();
  }
  const Node delta{s2.x_ - s1.x_, s2.y_ - s1.y_};
  return std::find_if(std::begin(motions_), std::end(motions_),
                      [&delta](const Node& motion) {
                        return CompareCoordinates(motion, delta);
                      })
      ->cost_;
}

Key DStarLite::CalculateKey(const Node& s) const {
  return Key{std::min(g_[s.x_][s.y_], rhs_[s.x_][s.y_]) + H(start_, s) + k_m_,
             std::min(g_[s.x_][s.y_], rhs_[s.x_][s.y_])};
}

std::vector<std::vector<double>> DStarLite::CreateGrid() {
  return std::vector<std::vector<double>>(
      n_, std::vector<double>(n_, std::numeric_limits<double>::max()));
}

void DStarLite::Initialize() {
  motions_ = GetMotion();
  time_step_ = 0;
  U_.clear();
  k_m_ = 0;
  rhs_ = CreateGrid();
  g_ = CreateGrid();
  rhs_[goal_.x_][goal_.y_] = 0;
  U_.insert(NodeKeyPair{goal_, CalculateKey(goal_)});
}

void DStarLite::UpdateVertex(const Node& u) {
  if (grid_[u.x_][u.y_] == 0) {
    grid_[u.x_][u.y_] = 2;
  }
  if (!CompareCoordinates(u, goal_)) {
    rhs_[u.x_][u.y_] = std::numeric_limits<double>::max();
    const auto successors = GetSucc(u);
    for (const auto& sprime : successors) {
      rhs_[u.x_][u.y_] =
          std::min(rhs_[u.x_][u.y_], C(u, sprime) + g_[sprime.x_][sprime.y_]);
    }
  }
  if (U_.isElementInStruct({u, {}})) {
    U_.remove(NodeKeyPair{u, Key()});
  }
  if (rhs_[u.x_][u.y_] != g_[u.x_][u.y_]) {
    U_.insert(NodeKeyPair{u, CalculateKey(u)});
  }
}

void DStarLite::ComputeShortestPath() {
  while ((!U_.empty() && U_.top().key < CalculateKey(start_)) ||
         (rhs_[start_.x_][start_.y_] != g_[start_.x_][start_.y_])) {
    k_old_ = U_.top().key;
    const Node u = U_.top().node;
    U_.pop();
    if (const Key u_key = CalculateKey(u); k_old_ < u_key) {
      U_.insert(NodeKeyPair{u, u_key});
    } else if (g_[u.x_][u.y_] > rhs_[u.x_][u.y_]) {
      g_[u.x_][u.y_] = rhs_[u.x_][u.y_];
      for (const auto& s : GetPred(u)) {
        UpdateVertex(s);
      }
    } else {
      g_[u.x_][u.y_] = std::numeric_limits<double>::max();
      for (const auto& s : GetPred(u)) {
        UpdateVertex(s);
      }
      UpdateVertex(u);
    }
  }
}

std::vector<Node> DStarLite::DetectChanges() {
  std::vector<Node> obstacles;
  if (time_discovered_obstacles_.find(time_step_) !=
      time_discovered_obstacles_.end()) {
    const auto discovered_obstacles_at_time =
        time_discovered_obstacles_[time_step_];
    for (const auto& discovered_obstacle_at_time :
         discovered_obstacles_at_time) {
      if (!((start_.x_ == discovered_obstacle_at_time.x_ &&
             start_.y_ == discovered_obstacle_at_time.y_) ||
            (goal_.x_ == discovered_obstacle_at_time.x_ &&
             goal_.y_ == discovered_obstacle_at_time.y_))) {
        grid_[discovered_obstacle_at_time.x_][discovered_obstacle_at_time.y_] =
            1;
        obstacles.push_back(discovered_obstacle_at_time);
      }
    }
  }
  if (create_random_obstacles_ && rand() > 1.0 / static_cast<double>(n_)) {
    const int x = rand() % n_;
    const int y = rand() % n_;
    if (!((start_.x_ == x && start_.y_ == y) ||
          (goal_.x_ == x && goal_.y_ == y))) {
      grid_[x][y] = 1;
      obstacles.emplace_back(Node(x, y));
    }
  }
  return obstacles;
}

void DStarLite::SetDynamicObstacles(
    const bool create_random_obstacles,
    const std::unordered_map<int, std::vector<Node>>&
        time_discovered_obstacles) {
  create_random_obstacles_ = create_random_obstacles;
  time_discovered_obstacles_ = time_discovered_obstacles;
}

std::tuple<bool, std::vector<Node>> DStarLite::Plan(const Node& start,
                                                    const Node& goal) {
  grid_ = original_grid_;
  start_ = start;
  goal_ = goal;
  std::vector<Node> path;
  path.push_back(start_);
  grid_[start_.x_][start_.y_] = 4;
  PrintGrid(grid_);
  auto last = start_;
  Initialize();
  ComputeShortestPath();
  while (!CompareCoordinates(start_, goal_)) {
    time_step_++;
    if (g_[start_.x_][start_.y_] == std::numeric_limits<double>::max()) {
      path.clear();
      path.push_back(start);
      path.back().cost_ = -1;
      std::cout << "The path has been blocked" << '\n';
      return {false, path};
    }
    const auto successors = GetSucc(start_);
    // double calculation, to optimize
    grid_[start_.x_][start_.y_] = 3;
    start_ = *std::min_element(std::begin(successors), std::end(successors),
                               [this](const auto& n1, const auto& n2) {
                                 return C(start_, n1) + g_[n1.x_][n1.y_] <
                                        C(start_, n2) + g_[n2.x_][n2.y_];
                               });
    path.push_back(start_);
    grid_[start_.x_][start_.y_] = 4;

#ifndef RUN_TESTS
    std::this_thread::sleep_for(std::chrono::milliseconds(pause_time));
#endif  // RUN_TESTS

    if (const auto changed_nodes = DetectChanges(); !changed_nodes.empty()) {
      k_m_ += H(last, start_);
      last = start;
      for (const auto node : changed_nodes) {
        UpdateVertex(node);
      }
      ComputeShortestPath();
    }
    start_.PrintStatus();
    PrintGrid(grid_);
  }
  path[0].id_ = path[0].x_ * n_ + path[0].y_;
  path[0].pid_ = path[0].id_;
  path[0].cost_ = 0;
  for (int i = 1; i < path.size(); i++) {
    path[i].id_ = path[i].x_ * n_ + path[i].y_;
    const auto delta =
        Node(path[i].x_ - path[i - 1].x_, path[i].y_ - path[i - 1].y_);
    path[i].cost_ = path[i - 1].cost_ +
                    std::find_if(std::begin(motions_), std::end(motions_),
                                 [&delta](const Node& motion) {
                                   return CompareCoordinates(motion, delta);
                                 })
                        ->cost_;
    path[i].pid_ = path[i - 1].id_;
  }
  return {true, path};
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

  const bool create_random_obstacles = false;
  const std::unordered_map<int, std::vector<Node>> time_discovered_obstacles{
      {1, {Node(1, 1)}},
      {2, {Node(2, 2)}},
      {3, {Node(5, 5)}},
      {4,
       {Node(6, 6), Node(7, 7), Node(8, 8), Node(9, 9), Node(10, 10),
        Node(7, 6)}}};

  DStarLite d_star_lite(grid);
  d_star_lite.SetDynamicObstacles(create_random_obstacles,
                                  time_discovered_obstacles);
  const auto [found_path, path_vector] = d_star_lite.Plan(start, goal);
  PrintPath(path_vector, start, goal, grid);
  return 0;
}
#endif  // BUILD_INDIVIDUAL
