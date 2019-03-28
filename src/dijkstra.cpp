/**
* @file dijkstra.cpp
* @author vss2sn
* @brief Contains the Dijkstra class
*/

#include "dijkstra.h"

/**
 * Main algorithm of Dijstra.
 */
std::vector<Node> Dijkstra::dijkstra(std::vector<std::vector<int> > &grid, int n, Node start_in, Node goal_in){
  start_ = start_in;
  goal_ = goal_in;
  int cost_grid[n][n];
  for (int i = 0; i < n; i++){
    for (int j = 0; j < n; j++){
      cost_grid[i][j] = n*n;
    }
  }

  std::vector<Node> motion = GetMotion();
  point_list_.push(start_);
  std::vector<Node> path_vector;
  path_vector.push_back(start_);

  cost_grid[start_.x_][start_.y_] = 0;
  while(!point_list_.empty()){
    Node current = point_list_.top();
    current.id_ = current.x_ * n + current.y_;
    point_list_.pop();
    if(current == goal_){
      return path_vector;
    }
    if(grid[current.x_][current.y_]!=0){
      continue; // Point already opened and
                // points around it added to points list
    }
    grid[current.x_][current.y_] = 2; // Point opened
    int current_cost = current.cost_;
    for(auto it = motion.begin(); it!=motion.end(); ++it){
      Node new_point;
      new_point = current + *it;
      if(new_point.x_ < 0 || new_point.y_ < 0
        || new_point.x_ >= n || new_point.y_ >= n) continue; // Check boundaries
      if(grid[new_point.x_][new_point.y_]==0 && cost_grid[new_point.x_][new_point.y_] > new_point.cost_){//=1 && grid[new_point.x_][new_point.y_]!=2){
        new_point.pid_ = current.id_;
        new_point.id_ = new_point.x_ * n + new_point.y_;
        point_list_.push(new_point);
        cost_grid[new_point.x_][new_point.y_] = new_point.cost_;
        std::vector<Node>::iterator it_v;
        it_v = find (path_vector.begin(), path_vector.end(), new_point);
        if (it_v != path_vector.end()) *it_v = new_point;
        else path_vector.push_back(new_point);
      }
    }
  }
  path_vector.clear();
  Node no_path_node(-1,-1,-1,-1,-1);
  path_vector.push_back(no_path_node);
  return path_vector;
}

#ifdef BUILD_INDIVIDUAL
/**
* @brief Script main function. Generates start and end nodes as well as grid, then creates the algorithm object and calls the main algorithm function.
* @return 0
*/
int main(){
  int n = 8;
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
