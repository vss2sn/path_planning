/**
* @file dijkstra.cpp
* @author vss2sn
* @brief Contains the Dijkstra class
*/

#include "dijkstra.hpp"

/**
 * @brief Main algorithm of Dijstra.
 */
std::vector<Node> Dijkstra::dijkstra(std::vector<std::vector<int> > &grid, int n, Node start_in, Node goal_in){
  start_ = start_in;
  goal_ = goal_in;

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
    int current_cost = current.cost_;
    for(auto it = motion.begin(); it!=motion.end(); ++it){
      Node new_point;
      new_point = current + *it;
      new_point.id_ = n*new_point.x_+new_point.y_;
      new_point.pid_ = current.id_;

      if(new_point == goal_){
        open_list_.push(new_point);
        break;
      }
      if(new_point.x_ < 0 || new_point.y_ < 0 || new_point.x_ >= n || new_point.y_ >= n) continue; // Check boundaries
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
  int n = 4;
  int num_points = n*n;

  std::vector<std::vector<int>> grid(n);
  std::vector<int> tmp(n);
  for (int i = 0; i < n; i++){
    grid[i] = tmp;
  }
  MakeGrid(grid, n);
  Node start(0,0,0,0,0,0);
  start.id_ = start.x_ * n + start.y_;
  start.pid_ = start.x_ * n + start.y_;
  Node goal(n-1,n-1,0,0,0,0);
  goal.id_ = goal.x_ * n + goal.y_;
  //Make sure start and goal are not obstacles and their ids are correctly assigned.
  grid[start.x_][start.y_] = 0;
  grid[goal.x_][goal.y_] = 0;
  PrintGrid(grid, n);

  Dijkstra new_dijkstra;
  std::vector<Node> path_vector = new_dijkstra.dijkstra(grid, n, start, goal);
  PrintPath(path_vector, start, goal, grid, n);

  return 0;
}
#endif
