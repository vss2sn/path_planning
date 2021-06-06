#include "path_planning/d_star_lite.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>

#ifdef BUILD_INDIVIDUAL
#include <random>
#endif  // BUILD_INDIVIDUAL

constexpr int pause_time = 200;  // milliseconds

void LazyPQ::clear() {
  s.clear();
  while(!pq.empty()) {
    pq.pop();
  }
}

void LazyPQ::insert(const NodeKeyPair& t) {
  if(auto p = s.insert(t); !p.second) {
    s.erase(t);
    s.insert(t);
  }
  pq.push(t);
}

void LazyPQ::pop() {
  while(!pq.empty()) {
    if (const auto it = s.find(pq.top()); it == s.end()) { // Element been removed from set
      pq.pop();
    } else if (it != s.end() && pq.top().key != it->key) { // Element has been updated in set with new key, and inserted already into pq with new value
      pq.pop();
    } else if (it != s.end() && pq.top().key == it->key) { // Found an elelment that is in set and priority queue
      break;
    }
  }
  if (s.empty()) {
    return;
  }
  s.erase(pq.top());
  pq.pop();
  // The loop below allows top() to be const without making the
  // std::priority_queue mutable
  while(!pq.empty()) {
    if (const auto it = s.find(pq.top()); it == s.end()) { // Element been removed from set
      pq.pop();
    } else if (it != s.end() && pq.top().key != it->key) { // Element has been updated in set with new key, and inserted already into pq with new value
      pq.pop();
    } else if (it != s.end() && pq.top().key == it->key) { // Found an elelment that is in set and priority queue
      break;
    }
  }
}

const NodeKeyPair& LazyPQ::top() const {
  return pq.top();
}

size_t LazyPQ::size() const {
  return s.size();
}

bool LazyPQ::empty() const {
  return s.empty();
}

bool LazyPQ::isElementInStruct(const NodeKeyPair& t) const {
  return s.find(t) != s.end();
}

void LazyPQ::remove(const NodeKeyPair& t) {
  if (s.find(t) != s.end()) {
    s.erase(t);
  }
  // Ensure top() is const
  while(!pq.empty()) {
    if (const auto it = s.find(pq.top()); it == s.end()) { // Element been removed from set
      pq.pop();
    } else if (it != s.end() && pq.top().key != it->key) { // Element has been updated in set with new key, and inserted already into pq with new value
      pq.pop();
    } else if (it != s.end() && pq.top().key == it->key) { // Found an elelment that is in set and priority queue
      break;
    }
  }
}

bool DStarLite::IsObstacle(const Node& n) const {
  return grid_[n.x_][n.y_] == 1;
}

double DStarLite::H(const Node& n1, const Node& n2) const {
  return 1;  // std::sqrt(std::pow(n1.x_ - n2.x_, 2) + std::pow(n1.y_ - n2.y_,
             // 2));
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
  return Key{std::min(g_[s.x_][s.y_], rhs_[s.x_][s.y_] + H(start_, s) + k_m_),
             std::min(g_[s.x_][s.y_], rhs_[s.x_][s.y_])};
}

std::vector<std::vector<double>> DStarLite::CreateGrid(const int n) {
  return std::vector<std::vector<double>>(
      n, std::vector<double>(n, std::numeric_limits<double>::max()));
}

void DStarLite::Initialize() {
  motions_ = GetMotion();
  time_step_ = 0;
  U_.clear();
  k_m_ = 0;
  rhs_ = CreateGrid(n_);
  g_ = CreateGrid(n_);
  rhs_[goal_.x_][goal_.y_] = 0;
  U_.insert(NodeKeyPair{goal_, CalculateKey(goal_)});
}

void DStarLite::UpdateVertex(const Node& u) {
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
  while (!U_.empty() && U_.top().key < CalculateKey(start_) ||
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
  if (create_random_obstacles_) {
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

std::vector<Node> DStarLite::Plan(
    const std::vector<std::vector<int>>& grid, const Node& start,
    const Node& goal, const bool create_random_obstacles,
    const std::unordered_map<int, std::vector<Node>>
        time_discovered_obstacles) {
  grid_ = grid;
  n_ = grid_.size();
  start_ = start;
  goal_ = goal;
  create_random_obstacles_ = create_random_obstacles;
  time_discovered_obstacles_ = time_discovered_obstacles;
  std::vector<Node> path;
  path.push_back(start_);
  grid_[start_.x_][start_.y_] = 3;
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
      return path;
    }
    const auto successors = GetSucc(start_);
    // double calculateion, to optimize
    start_ = *std::min_element(std::begin(successors), std::end(successors),
                               [this](const auto& n1, const auto& n2) {
                                 return C(start_, n1) + g_[n1.x_][n1.y_] <
                                        C(start_, n2) + g_[n2.x_][n2.y_];
                               });
    path.push_back(start_);
    grid_[start_.x_][start_.y_] = 3;
    std::this_thread::sleep_for(std::chrono::milliseconds(pause_time));
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
  return path;
}

#ifdef BUILD_INDIVIDUAL
/**
 * @brief Script main function. Generates start and end nodes as well as grid,
 * then creates the algorithm object and calls the main algorithm function.
 * @return 0
 */
int main() {
  int n = 11;
  std::vector<std::vector<int>> grid(n, std::vector<int>(n));
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

  const bool create_random_obstacles = true;
  const std::unordered_map<int, std::vector<Node>> time_discovered_obstacles{
      {1, {Node(1, 1)}},
      {2, {Node(2, 2)}},
      {3, {Node(5, 5)}},
      {4, {Node(6, 6), Node(7, 7), Node(8, 8), Node(9, 9), Node(10, 10), Node(7, 6)}}};

  DStarLite d_star_lite;
  const std::vector<Node> path_vector = d_star_lite.Plan(
      grid, start, goal, create_random_obstacles, time_discovered_obstacles);
  start.PrintStatus();
  goal.PrintStatus();
  return 0;
}
#endif  // BUILD_INDIVIDUAL
