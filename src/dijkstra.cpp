/**
* @file dijkstra.cpp
* @author vss2sn
* @brief Contains the Dijkstra class
*/

#ifdef BUILD_INDIVIDUAL
#include <random>
#endif  // BUILD_INDIVIDUAL

#include "dijkstra.hpp"

std::vector<Node> Dijkstra::dijkstra(std::vector<std::vector<int> > &grid, const Node& start_in, const Node& goal_in){
  start_ = start_in;
  goal_ = goal_in;
  n = grid.size();
  // Get possible motions
  std::vector<Node> motion = GetMotion();
  open_list_.push(start_);

  // Main loop
  Node temp;
  while(!open_list_.empty()){
    Node current = open_list_.top();
    open_list_.pop();
    current.id_ = current.x_ * n + current.y_;
    if(current.x_ == goal_.x_ && current.y_ == goal_.y_){
      closed_list_.push_back(current);
      grid[current.x_][current.y_] = 2;
      return closed_list_;
    }
    grid[current.x_][current.y_] = 2; // Point opened
    for(const auto& m : motion){
      Node new_point;
      new_point = current + m;
      new_point.id_ = n*new_point.x_+new_point.y_;
      new_point.pid_ = current.id_;

      if(new_point == goal_){
        open_list_.push(new_point);
        break;
      }
      if(new_point.x_ < 0 || new_point.y_ < 0 || new_point.x_ >= n || new_point.y_ >= n) {
        continue; // Check boundaries
      }
      if(grid[new_point.x_][new_point.y_]!=0){
        continue; //obstacle or visited
      }
      open_list_.push(new_point);
    }
    closed_list_.push_back(current);
  }
  closed_list_.clear();
  Node no_path_node(-1,-1,-1,-1,-1,-1);
  closed_list_.push_back(no_path_node);
  return closed_list_;
}

#ifdef BUILD_INDIVIDUAL
/**
* @brief Script main function. Generates start and end nodes as well as grid, then creates the algorithm object and calls the main algorithm function.
* @return 0
*/
int main(){
  int n = 11;

  std::vector<std::vector<int>> grid(n, std::vector<int>(n));
  MakeGrid(grid);
  std::random_device rd; // obtain a random number from hardware
  std::mt19937 eng(rd()); // seed the generator
  std::uniform_int_distribution<int> distr(0,n-1); // define the range

  Node start(distr(eng),distr(eng),0,0,0,0);
  Node goal(distr(eng),distr(eng),0,0,0,0);

  start.id_ = start.x_ * n + start.y_;
  start.pid_ = start.x_ * n + start.y_;
  goal.id_ = goal.x_ * n + goal.y_;
  start.h_cost_ = abs(start.x_ - goal.x_) + abs(start.y_ - goal.y_);
  //Make sure start and goal are not obstacles and their ids are correctly assigned.
  grid[start.x_][start.y_] = 0;
  grid[goal.x_][goal.y_] = 0;
  PrintGrid(grid);

  Dijkstra new_dijkstra;
  std::vector<Node> path_vector = new_dijkstra.dijkstra(grid, start, goal);
  PrintPath(path_vector, start, goal, grid);

  return 0;
}
#endif  // BUILD_INDIVIDUAL
