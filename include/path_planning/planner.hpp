#include <utils/utils.hpp>

#include <vector>
#include <tuple>

class Planner {
public:
  Planner (std::vector<std::vector<int>> grid) : original_grid_(std::move(grid)), n_(original_grid_.size()) {};
  Planner(const Planner&) = default;
  Planner(Planner&&) = default;
  Planner& operator=(const Planner&) = default;
  Planner& operator=(Planner&&) = default;
  virtual ~Planner() = default;

  virtual std::tuple<bool, std::vector<Node>> Plan(const Node& start, const Node& send) = 0;
protected:
  std::vector<std::vector<int>> grid_ = {};
  const std::vector<std::vector<int>> original_grid_;
  const int n_;
};
