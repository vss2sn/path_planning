/*

Dijstra grid based planning

*/

#include "dijkstra.h"

std::vector<Node> DIJKSTRA::dijkstra(void *grid, int n, Node start_in, Node goal_in){
  start_ = start_in;
  goal_ = goal_in;
  int (*p_grid)[n][n] = (int (*)[n][n]) grid;
  int cost_grid[n][n];
  for (int i = 0; i < n; i++){
    for (int j = 0; j < n; j++){
      cost_grid[i][j] = n*n;
    }
  }

  std::vector<Node> motion = GetMotion(n);
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
    if((*p_grid)[current.x_][current.y_]!=0){
      continue; // Point already opened and
                // points around it added to points list
    }
    (*p_grid)[current.x_][current.y_] = 2; // Point opened
    int current_cost = current.cost_;
    for(auto it = motion.begin(); it!=motion.end(); ++it){
      Node new_point;
      new_point = current + *it;
      if(new_point.x_ < 0 || new_point.y_ < 0
        || new_point.x_ >= n || new_point.y_ >= n) continue; // Check boundaries
      if((*p_grid)[new_point.x_][new_point.y_]==0 && cost_grid[new_point.x_][new_point.y_] > new_point.cost_){//=1 && (*p_grid)[new_point.x_][new_point.y_]!=2){
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
int main(){
  int n = 3;
  int num_points = n*n;

  n = 6;
  int grid[n][n] = {
                     { 0 , 0 , 0 , 0 , 0, 0 },
                     { 0 , 1 , 1 , 0 , 0, 0 },
                     { 1 , 1 , 0 , 0 , 1, 0 },
                     { 1 , 0 , 0 , 1 , 1, 1 },
                     { 0 , 1 , 1 , 1 , 1, 0 },
                     { 0 , 0 , 0 , 0 , 0, 0 }
                   } ;

  //int grid[n][n];
  //MakeGrid(grid, n);

  //NOTE:
  // x = row index, y = column index.

  std::cout << "Grid:" << std::endl;
  std::cout << "1. Points not considered ---> 0" << std::endl;
  std::cout << "2. Obstacles             ---> 1" << std::endl;
  PrintGrid(grid, n);

  //Make sure start and goal are not obstacles and their ids are correctly assigned.
  Node start(0,1,0,0,0,0);
  start.id_ = start.x_ * n + start.y_;
  start.pid_ = start.x_ * n + start.y_;
  Node goal(n-1,n-1,0,0,0,0);
  goal.id_ = goal.x_ * n + goal.y_;

  grid[start.x_][start.y_] = 0;
  grid[goal.x_][goal.y_] = 0;

  DIJKSTRA new_dijkstra;
  std::vector<Node> path_vector = new_dijkstra.dijkstra(grid, n, start, goal);
  PrintPath(path_vector, start, goal, grid, n);

  return 0;
}
#endif
