/*

A* grid based planning

*/

#include "a_star.h"

std::vector<Node> AStar::a_star(void *grid, int n, Node start_in, Node goal_in){
  start_ = start_in;
  goal_ = goal_in;
  int (*p_grid)[n][n] = (int (*)[n][n]) grid;
  // Get possible motions
  std::vector<Node> motion = GetMotion();
  point_list_.push(start_);
  std::vector<Node> path_vector;
  path_vector.push_back(start_);

  // Main loop
  while(!point_list_.empty()){
    Node current = point_list_.top();
    current.id_ = current.x_ * n + current.y_;
    point_list_.pop();
    if(current == goal_) return path_vector;

    if((*p_grid)[current.x_][current.y_]!=0){
      continue; // Point already opened and
                // points around it added to points list
    }
    (*p_grid)[current.x_][current.y_] = 2; // Point opened
    int current_cost = current.cost_;
    for(auto it = motion.begin(); it!=motion.end(); ++it){
      Node new_point;
      new_point = current + *it;
      new_point.h_cost_ = abs(new_point.x_ - goal_.x_) + abs(new_point.y_ - goal_.y_);
      if(new_point.x_ < 0 || new_point.y_ < 0
          || new_point.x_ >= n || new_point.y_ >= n) continue; // Check boundaries
      if((*p_grid)[new_point.x_][new_point.y_]!= 2){
        new_point.pid_ = current.id_;
        new_point.id_ = new_point.x_ * n + new_point.y_;
        point_list_.push(new_point);
        std::vector<Node>::iterator it_v = find (path_vector.begin(), path_vector.end(), new_point);
        if (it_v != path_vector.end()) *it_v = new_point; //update point in list
        else path_vector.push_back(new_point); // add new point to list
      }
    }
  }
  path_vector.clear();
  Node no_path_node(-1,-1,-1,-1,-1,-1);
  path_vector.push_back(no_path_node);
  return path_vector;
}

#ifdef BUILD_INDIVIDUAL
int main(){
  int n = 8;
  int num_points = n*n;

  int grid[n][n];
  MakeGrid(grid, n);
  PrintGrid(grid, n);

  Node start(0,0,0,0,0,0);
  start.id_ = start.x_ * n + start.y_;
  start.pid_ = start.x_ * n + start.y_;
  Node goal(n-1,n-1,0,0,0,0);
  goal.id_ = goal.x_ * n + goal.y_;
  start.h_cost_ = abs(start.x_ - goal.x_) + abs(start.y_ - goal.y_);
  //Make sure start and goal are not obstacles and their ids are correctly assigned.
  grid[start.x_][start.y_] = 0;
  grid[goal.x_][goal.y_] = 0;

  AStar new_a_star;
  std::vector<Node> path_vector = new_a_star.a_star(grid, n, start, goal);
  PrintPath(path_vector, start, goal, grid, n);

  return 0;
}
#endif
