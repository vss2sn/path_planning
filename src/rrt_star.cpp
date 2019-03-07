/*

RRT* grid based planning

*/

#include "rrt_star.h"

Node RRT_STAR::FindNearestPoint(Node& new_node, int n){
  Node nearest_node(-1,-1,-1,-1,-1,-1);
  std::vector<Node>::iterator it_v;
  std::vector<Node>::iterator it_v_store;
  //use total cost not just distance
  double dist = (double)(n*n);
  double new_dist = (double)(n*n);
  for(it_v = point_list.begin(); it_v != point_list.end(); ++it_v){
    new_dist = (double)sqrt(((double)(it_v->x_-new_node.x_)*(double)(it_v->x_-new_node.x_))
                + ((double)(it_v->y_-new_node.y_)*(double)(it_v->y_-new_node.y_)));
    if(new_dist > threshold) continue;
    new_dist += it_v->cost_;

    if(CheckObstacle(*it_v, new_node)) continue;
    if(it_v->id_==new_node.id_) continue;
    // The nearest nodes are stored while searching for the nearest node to
    // speed up th rewire process
    near_nodes.push_back(*it_v);
    near_nodes_dist.push_back(new_dist);
    if(it_v->pid_==new_node.id_) continue;
    if(new_dist >= dist) continue;
    dist = new_dist;
    it_v_store = it_v;
  }
  if(dist!=n*n){
    nearest_node = *it_v_store;
    new_node.pid_ = nearest_node.id_;
    new_node.cost_ = dist;
  }
  return nearest_node;
}

bool RRT_STAR::CheckObstacle(Node& n_1, Node& n_2){
  // As this planner is for grid maps, the obstacles are square.
  if (n_2.y_ - n_1.y_ == 0){
    double c = n_2.y_;
    for(auto it_v = obstacle_list.begin(); it_v!=obstacle_list.end(); ++it_v){
      if(!(((n_1.x_>=it_v->x_) && (it_v->x_>= n_2.x_)) || ((n_1.x_<=it_v->x_) && (it_v->x_<= n_2.x_)))) continue;
      if ((double)it_v->y_ == c) return true;
    }
  }
  else {
    double slope = (double)(n_2.x_ - n_1.x_)/(double)(n_2.y_ - n_1.y_);
    std::vector<Node>::iterator it_v;
    double c = (double)n_2.x_ - slope * (double)n_2.y_;
    for(auto it_v = obstacle_list.begin(); it_v!=obstacle_list.end(); ++it_v){
      if(!(((n_1.y_>=it_v->y_) && (it_v->y_>= n_2.y_)) || ((n_1.y_<=it_v->y_) && (it_v->y_<= n_2.y_)))) continue;
      if(!(((n_1.x_>=it_v->x_) && (it_v->x_>= n_2.x_)) || ((n_1.x_<=it_v->x_) && (it_v->x_<= n_2.x_)))) continue;
      double arr[4];
      arr[0] = (double)it_v->x_+0.5 - slope*((double)it_v->y_+0.5) - c;
      arr[1] = (double)it_v->x_+0.5 - slope*((double)it_v->y_-0.5) - c;
      arr[2] = (double)it_v->x_-0.5 - slope*((double)it_v->y_+0.5) - c;
      arr[3] = (double)it_v->x_-0.5 - slope*((double)it_v->y_-0.5) - c;
      int count = 0;
      int j = 0;

      for (int i=0;i<4;i++){
        if(fabs(arr[i]) <= 0.000001){
          count +=1;
          if(j==0 && i==0)j=1;
          if(count > 1) return true;
          continue;
        }
        arr[i] = arr[i]/fabs(arr[i]);
        if ((arr[j]-arr[i]) != 0 ) return true;
      }
    }
  }
  return false;
}

Node RRT_STAR::GenerateRandomNode(int n){
  std::random_device rd; // obtain a random number from hardware
  std::mt19937 eng(rd()); // seed the generator
  std::uniform_int_distribution<int> distr(0,n-1); // define the range
  int x = distr(eng);
  int y = distr(eng);
  Node new_node(x, y, 0, 0, n*x+y, 0);
  return new_node;
}

void RRT_STAR::Rewire(Node new_node){
  std::vector<Node>::iterator it_v;
  for(int i=0;i<near_nodes.size(); i++){
    if (near_nodes[i].cost_ > near_nodes_dist[i] + new_node.cost_){
      for(it_v = point_list.begin(); it_v!=point_list.end();++it_v){
        if(*it_v==near_nodes[i]) break;
      }
      it_v->pid_ = new_node.id_;
      it_v->cost_ = near_nodes_dist[i] + new_node.cost_;
    }
  }
  near_nodes.clear();
  near_nodes_dist.clear();
}


std::vector<Node> RRT_STAR::rrt_star(void *grid, int n, Node start_in, Node goal_in, int max_iter_x_factor = 500, double threshold_in = std::numeric_limits<double>::infinity()){
  start = start_in;
  goal = goal_in;
  threshold = threshold_in;
  int max_iter = max_iter_x_factor * n * n;
  int (*p_grid)[n][n] = (int (*)[n][n]) grid;
  CreateObstacleList(*p_grid, n);
  point_list.push_back(start);
  (*p_grid)[start.x_][start.y_]=2;
  int iter = 0;
  Node new_node = start;
  if(CheckGoalVisible(new_node)) found_goal = true;
  while(true){
    iter++;
    if(iter > max_iter){
      if(!found_goal){
        Node no_path_node(-1,-1,-1,-1,-1,-1);
        point_list.clear();
        point_list.push_back(no_path_node);
      }
      return point_list;
    }
    new_node = GenerateRandomNode(n);
    if ((*p_grid)[new_node.x_][new_node.y_]==1){
      continue;
    }
    Node nearest_node = FindNearestPoint(new_node, n);
    if(nearest_node.id_ == -1){
      continue;
    }
    (*p_grid)[new_node.x_][new_node.y_]=2;
    std::vector<Node>::iterator it_v;

    for( it_v = point_list.begin(); it_v!=point_list.end(); ++it_v){
      if(new_node.x_ == it_v->x_ && new_node.y_ == it_v->y_){
        if(new_node.cost_ < it_v->cost_){
          point_list.erase(it_v);
          point_list.push_back(new_node);
        }
        break;
      }
    }
    if(it_v==point_list.end()) point_list.push_back(new_node);
    if(CheckGoalVisible(new_node)) found_goal = true;
    Rewire(new_node);
  }
}

bool RRT_STAR::CheckGoalVisible(Node new_node){
  if(!CheckObstacle(new_node, goal)){
    double new_dist = (double)sqrt((double)(goal.x_-new_node.x_)*(double)(goal.x_-new_node.x_)
                + (double)(goal.y_-new_node.y_)*(double)(goal.y_-new_node.y_));
    if(new_dist <=threshold){
      new_dist+=new_node.cost_;
      goal.pid_ = new_node.id_;
      goal.cost_ = new_dist;
      std::vector<Node>::iterator it_v;
      for( it_v = point_list.begin(); it_v!=point_list.end(); ++it_v){
        if(goal.x_ == it_v->x_ && goal.y_ == it_v->y_){
          if(goal.cost_ < it_v->cost_){
            point_list.erase(it_v);
            point_list.push_back(goal);
          }
          break;
        }
      }
      if(it_v==point_list.end()) point_list.push_back(goal);
      return true;
    }
  }
  return false;
}

void RRT_STAR::CreateObstacleList(void *grid, int n){
  int (*p_grid)[n][n] = (int (*)[n][n]) grid;
  for(int i=0; i < n; i++){
    for(int j=0;j < n; j++){
      if((*p_grid)[i][j]==1){
        Node obs(i,j,0,i*n+j,0);
        obstacle_list.push_back(obs);
      }
    }
  }
}

void RRT_STAR::PrintCost(void *grid, int n){
  //NOTE: Using a void pointer isnt the best option
  //std::cout << "POINT LIST SIZE:" << point_list.size() <<std::endl;
  int (*p_grid)[n][n] = (int (*)[n][n]) grid;
  std::vector<Node>::iterator it_v;
  for(int i=0;i<n;i++){
    for(int j=0;j<n;j++){
      for(it_v = point_list.begin(); it_v != point_list.end(); ++it_v){
        if(i == it_v->x_ && j== it_v->y_){
          std::cout << std::setw(10) <<it_v->cost_ << " , ";
          break;
        }
      }
      if(it_v == point_list.end())
      std::cout << std::setw(10) << "  , ";
    }
    std::cout << std::endl << std::endl;
  }
}

#ifdef BUILD_INDIVIDUAL
int main(){
  int n = 8;
  int num_points = n*n;


/*
  int grid[n][n] = {
                     { 0 , 0 , 0 , 0 },
                     { 0 , 0 , 0 , 0 },
                     { 0 , 0 , 1 , 0 },
                     { 0 , 0 , 0 , 0 }
                   };
*/
  /*
  n = 6;
  int grid[n][n] = {
                     { 0 , 0 , 0 , 0 , 0, 0 },
                     { 0 , 1 , 0 , 0 , 0, 0 },
                     { 1 , 0 , 0 , 1 , 1, 0 },
                     { 1 , 0 , 1 , 0 , 1, 0 },
                     { 0 , 0 , 1 , 1 , 1, 1 },
                     { 0 , 0 , 0 , 0 , 0, 0 }
                   } ;
                   */

   int grid[n][n];
   make_grid(grid, n);

  //NOTE:
  // x = row index, y = column index.

  std::cout << "Grid:" << std::endl;
  std::cout << "1. Points not considered ---> 0" << std::endl;
  std::cout << "2. Obstacles             ---> 1" << std::endl;
  PrintGrid(grid, n);

  //Make sure start and goal not obstacles and their ids are correctly assigned.
  Node start(0,0,0,0,0,0);
  start.id_ = start.x_ * n + start.y_;
  start.pid_ = start.x_ * n + start.y_;
  Node goal(n-1,n-1,0,0,0,0);
  goal.id_ = goal.x_ * n + goal.y_;

  grid[start.x_][start.y_] = 0;
  grid[goal.x_][goal.y_] = 0;
  RRT_STAR new_rrt_star;

  std::vector<Node> path_vector = new_rrt_star.rrt_star(grid, n, start, goal, 20, 2);
  print_path(path_vector, start, goal, grid, n);

  return 0;
}
#endif
