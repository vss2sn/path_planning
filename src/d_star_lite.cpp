#include "path_planning/d_star_lite.hpp"
#include <cmath>


#ifdef BUILD_INDIVIDUAL
#include <random>
#endif  // BUILD_INDIVIDUAL


bool DStarLiteRefactor::IsObstacle(const Node& n) const {
  return grid_[n.x_][n.y_] != 0;
}

double DStarLiteRefactor::H(const Node& n1, const Node& n2) const {
  return 1; //std::sqrt(std::pow(n1.x_ - n2.x_, 2) + std::pow(n1.y_ - n2.y_, 2));
}

std::vector<Node> DStarLiteRefactor::GetNeighbours(const Node& u) const {
  std::vector<Node> neighbours;
  for (const auto& m : motions_) {
    if (const auto neighbour = u + m; !checkOutsideBoundary(neighbour, grid_.size())) {
      neighbours.push_back(neighbour);
    }
  }
  return neighbours;
}

std::vector<Node> DStarLiteRefactor::GetPred(const Node& u) const {
  return GetNeighbours(u);
}

std::vector<Node> DStarLiteRefactor::GetSucc(const Node& u) const {
  return GetNeighbours(u);
}

double DStarLiteRefactor::C(const Node& s1, const Node& s2) const {
  if (IsObstacle(s1) || IsObstacle(s2)) {
    return std::numeric_limits<double>::max();
  }
  const Node delta{s2.x_-s1.x_, s2.y_-s1.y_};
  return std::find_if(std::begin(motions_), std::end(motions_), [&delta](const Node& motion) { return CompareCoordinates(motion, delta); })->cost_;
}

Key DStarLiteRefactor::CalculateKey(const Node& s) const {
  return Key {
    std::min(g_[s.x_][s.y_], rhs_[s.x_][s.y_] + H(start_, s) + k_m_),
    std::min(g_[s.x_][s.y_], rhs_[s.x_][s.y_])
  };
}

std::vector<std::vector<double>> DStarLiteRefactor::CreateGrid(const int n) {
  return std::vector<std::vector<double>>
    (n, std::vector<double>(n, std::numeric_limits<double>::max()));
}

void DStarLiteRefactor::Initialize() {
  motions_ =  GetMotion();
  U_.clear();
  k_m_ = 0;
  rhs_ = CreateGrid(n_);
  g_ = CreateGrid(n_);
  rhs_[goal_.x_][goal_.y_] = 0;
  U_.insert(NodeKeyPair{goal_, CalculateKey(goal_)});
}

void DStarLiteRefactor::UpdateVertex(const Node& u) {
  if (!CompareCoordinates(u, goal_)) {
    rhs_[u.x_][u.y_] = std::numeric_limits<double>::max();
    const auto successors = GetSucc(u);
    for (const auto& sprime : successors) {
      rhs_[u.x_][u.y_] = std::min(rhs_[u.x_][u.y_], C(u, sprime) + g_[sprime.x_][sprime.y_]);
    }
  }
  if (U_.isElementInStruct({u, {}})) {
    U_.remove(NodeKeyPair{u, Key()});
  }
  if (rhs_[u.x_][u.y_] != g_[u.x_][u.y_]) {
    U_.insert(NodeKeyPair{u, CalculateKey(u)});
  }
}

void DStarLiteRefactor::ComputeShortestPath() {
  while (!U_.empty() && U_.top().key < CalculateKey(start_) || (rhs_[start_.x_][start_.y_] != g_[start_.x_][start_.y_])) {
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

std::vector<Node> DStarLiteRefactor::DetectChanges() {
  const int x = rand() % n_;
  const int y = rand() % n_;
  if (!((start_.x_ == x && start_.y_ == y) || (goal_.x_ == x && goal_.y_ == y))) {
    grid_[x][y] = 1;
    return {Node(x, y)};
  }
  return {};
}

std::tuple<bool, std::vector<Node>> DStarLiteRefactor::Plan(
  const std::vector<std::vector<int>>& grid,
  const Node& start,
  const Node& goal) {
  grid_ = grid;
  n_ = grid_.size();
  start_ = start;
  goal_ = goal;
  std::vector<Node> path;
  path.push_back(start_);
  start_.PrintStatus();
  auto last = start_;
  Initialize();
  ComputeShortestPath();
  while(!CompareCoordinates(start_, goal_)) {
    if (g_[start_.x_][start_.y_] == std::numeric_limits<double>::max()) {
      return {false, path};
    }
    const auto successors = GetSucc(start_);
    // double calculateion, to optimize
    start_ = *std::min_element(std::begin(successors), std::end(successors), [this](const auto& n1, const auto& n2) { return C(start_, n1) + g_[n1.x_][n1.y_] < C(start_, n2) + g_[n2.x_][n2.y_] ; });
    path.push_back(start_);
    start_.PrintStatus();
    if (const auto changed_nodes = DetectChanges(); !changed_nodes.empty()) {
      k_m_ += H(last, start_);
      last = start;
      for (const auto node : changed_nodes) {
        UpdateVertex(node);
      }
      ComputeShortestPath();
    }

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
  PrintGrid(grid);

  DStarLiteRefactor d_star_lite;
  const auto [path_found, path_vector] = d_star_lite.Plan(grid, start, goal);
  for (const auto& ele : path_vector) {
    // ele.PrintStatus();
    grid[ele.x_][ele.y_] = 3;
  }
  PrintGrid(grid);
  // PrintPath(path_vector, start, goal, grid);
  return 0;
}
#endif  // BUILD_INDIVIDUAL
