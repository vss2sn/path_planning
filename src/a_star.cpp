/*

A* grid based planning

*/

#include "a_star.h"
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>

std::vector<Node> AStar::a_star(void *grid, int n, Node start_in, Node goal_in){
  start_ = start_in;
  goal_ = goal_in;
  int (*p_grid)[n][n] = (int (*)[n][n]) grid;

  // Get possible motions
  std::vector<Node> motion = GetMotion();
  open_list_.push_back(start_);

  // Main loop
  Node temp;
  while(!open_list_.empty()){
    //sorting; minor problem with inbuild sort for vector. To optimise.
    for(int i=0; i < open_list_.size(); i++){
      for(int j=0; j < open_list_.size(); j++){
        if(open_list_[i].cost_ + open_list_[i].h_cost_ <= open_list_[j].cost_ + open_list_[j].h_cost_){
          temp = open_list_[i];
          open_list_[i] = open_list_[j];
          open_list_[j] = temp;
        }
      }
    }
    Node current = *(open_list_.begin());
    open_list_.erase(open_list_.begin());
    current.id_ = current.x_ * n + current.y_;
    if(current.x_ == goal_.x_ && current.y_ == goal_.y_){
      closed_list_.push_back(current);
      (*p_grid)[current.x_][current.y_] = 2;
      return closed_list_;
    }

    (*p_grid)[current.x_][current.y_] = 2; // Point opened

    int current_cost = current.cost_;
    for(auto it = motion.begin(); it!=motion.end(); ++it){
      Node new_point;
      new_point = current + *it;
      new_point.id_ = n*new_point.x_+new_point.y_;
      new_point.pid_ = current.id_;
      new_point.h_cost_ = abs(new_point.x_ - goal_.x_) + abs(new_point.y_ - goal_.y_);

      if(new_point == goal_){
        open_list_.push_back(new_point);
        break;
      }
      if(new_point.x_ < 0 || new_point.y_ < 0 || new_point.x_ >= n || new_point.y_ >= n) continue; // Check boundaries
      if((*p_grid)[new_point.x_][new_point.y_]==1){
        continue; //obstacle
      }
      std::vector<Node>::iterator it_v = find (open_list_.begin(), open_list_.end(), new_point);
      if(it_v!=open_list_.end() && ((new_point.cost_ + new_point.h_cost_) < (it_v->cost_ + it_v->h_cost_))){
        *it_v = new_point;
        continue;
      }
      it_v = find (closed_list_.begin(), closed_list_.end(), new_point);
      if(it_v!=closed_list_.end() && ((new_point.cost_ + new_point.h_cost_) > (it_v->cost_ + it_v->h_cost_))){
        continue;
      }
      open_list_.push_back(new_point);
    }
    closed_list_.push_back(current);
  }
  closed_list_.clear();
  Node no_path_node(-1,-1,-1,-1,-1,-1);
  closed_list_.push_back(no_path_node);
  return closed_list_;
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
