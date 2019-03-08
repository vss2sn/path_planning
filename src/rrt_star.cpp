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
  for(it_v = point_list_.begin(); it_v != point_list_.end(); ++it_v){
    new_dist = (double)sqrt(((double)(it_v->x_-new_node.x_)*(double)(it_v->x_-new_node.x_))
                + ((double)(it_v->y_-new_node.y_)*(double)(it_v->y_-new_node.y_)));
    if(new_dist > threshold_) continue;
    new_dist += it_v->cost_;

    if(CheckObstacle(*it_v, new_node)) continue;
    if(it_v->id_==new_node.id_) continue;
    // The nearest nodes are stored while searching for the nearest node to
    // speed up th rewire process
    near_nodes_.push_back(*it_v);
    near_nodes_dist_.push_back(new_dist);
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
    for(auto it_v = obstacle_list_.begin(); it_v!=obstacle_list_.end(); ++it_v){
      if(!(((n_1.x_>=it_v->x_) && (it_v->x_>= n_2.x_)) || ((n_1.x_<=it_v->x_) && (it_v->x_<= n_2.x_)))) continue;
      if ((double)it_v->y_ == c) return true;
    }
  }
  else {
    double slope = (double)(n_2.x_ - n_1.x_)/(double)(n_2.y_ - n_1.y_);
    std::vector<Node>::iterator it_v;
    double c = (double)n_2.x_ - slope * (double)n_2.y_;
    for(auto it_v = obstacle_list_.begin(); it_v!=obstacle_list_.end(); ++it_v){
      if(!(((n_1.y_>=it_v->y_) && (it_v->y_>= n_2.y_)) || ((n_1.y_<=it_v->y_) && (it_v->y_<= n_2.y_)))) continue;
      if(!(((n_1.x_>=it_v->x_) && (it_v->x_>= n_2.x_)) || ((n_1.x_<=it_v->x_) && (it_v->x_<= n_2.x_)))) continue;
      double arr[4];
      arr[0] = (double)it_v->x_+0.5 - slope*((double)it_v->y_+0.5) - c;
      arr[1] = (double)it_v->x_+0.5 - slope*((double)it_v->y_-0.5) - c;
      arr[2] = (double)it_v->x_-0.5 - slope*((double)it_v->y_+0.5) - c;
      arr[3] = (double)it_v->x_-0.5 - slope*((double)it_v->y_-0.5) - c;
      double count = 0;
      for (int i=0;i<4;i++){
        if(fabs(arr[i]) <= 0.000001) arr[i] = 0;
        else count +=arr[i]/fabs(arr[i]);
      }
      if(abs(count) < 3) return true;
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
  for(int i=0;i<near_nodes_.size(); i++){
    if (near_nodes_[i].cost_ > near_nodes_dist_[i] + new_node.cost_){
      it_v = find (point_list_.begin(), point_list_.end(), near_nodes_[i]);
      if (it_v != point_list_.end()){
        it_v->pid_ = new_node.id_;
        it_v->cost_ = near_nodes_dist_[i] + new_node.cost_;
      }
    }
  }
  near_nodes_.clear();
  near_nodes_dist_.clear();
}


std::vector<Node> RRT_STAR::rrt_star(void *grid, int n, Node start_in, Node goal_in, int max_iter_x_factor = 500, double threshold_in = std::numeric_limits<double>::infinity()){
  start_ = start_in;
  goal_ = goal_in;
  threshold_ = threshold_in;
  int max_iter = max_iter_x_factor * n * n;
  int (*p_grid)[n][n] = (int (*)[n][n]) grid;
  CreateObstacleList(*p_grid, n);
  point_list_.push_back(start_);
  (*p_grid)[start_.x_][start_.y_]=2;
  int iter = 0;
  Node new_node = start_;
  if(CheckGoalVisible(new_node)) found_goal_ = true;
  while(true){
    iter++;
    if(iter > max_iter){
      if(!found_goal_){
        Node no_path_node(-1,-1,-1,-1,-1,-1);
        point_list_.clear();
        point_list_.push_back(no_path_node);
      }
      return point_list_;
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

    it_v = find (point_list_.begin(), point_list_.end(), new_node);
    if (it_v != point_list_.end() && new_node.cost_ < it_v->cost_){
      point_list_.erase(it_v);
      point_list_.push_back(new_node);
    }
    else if(it_v==point_list_.end()) point_list_.push_back(new_node);
    if(CheckGoalVisible(new_node)) found_goal_ = true;
    Rewire(new_node);
  }
}

bool RRT_STAR::CheckGoalVisible(Node new_node){
  if(!CheckObstacle(new_node, goal_)){
    double new_dist = (double)sqrt((double)(goal_.x_-new_node.x_)*(double)(goal_.x_-new_node.x_)
                + (double)(goal_.y_-new_node.y_)*(double)(goal_.y_-new_node.y_));
    if(new_dist <=threshold_){
      new_dist+=new_node.cost_;
      goal_.pid_ = new_node.id_;
      goal_.cost_ = new_dist;
      std::vector<Node>::iterator it_v;
      it_v = find (point_list_.begin(), point_list_.end(), goal_);
      if(it_v!=point_list_.end() && goal_.cost_ < it_v->cost_){
        point_list_.erase(it_v);
        point_list_.push_back(goal_);
      }
      else if(it_v==point_list_.end()) point_list_.push_back(goal_);
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
        Node obs(i,j,0,0,i*n+j,0);
        obstacle_list_.push_back(obs);
      }
    }
  }
}

void RRT_STAR::PrintCost(void *grid, int n){
  //NOTE: Using a void pointer isnt the best option
  //std::cout << "POINT LIST SIZE:" << point_list_.size() <<std::endl;
  int (*p_grid)[n][n] = (int (*)[n][n]) grid;
  std::vector<Node>::iterator it_v;
  for(int i=0;i<n;i++){
    for(int j=0;j<n;j++){
      for(it_v = point_list_.begin(); it_v != point_list_.end(); ++it_v){
        if(i == it_v->x_ && j== it_v->y_){
          std::cout << std::setw(10) <<it_v->cost_ << " , ";
          break;
        }
      }
      if(it_v == point_list_.end())
      std::cout << std::setw(10) << "  , ";
    }
    std::cout << std::endl << std::endl;
  }
}

#ifdef BUILD_INDIVIDUAL
int main(){

  int n = 8;
  int num_points = n*n;

  // n=5;
  // int grid[n][n] = {
  //                    { 0 , 1 , 1 , 1 , 0},
  //                    { 0 , 0 , 0 , 0 , 0},
  //                    { 0 , 1 , 1 , 1 , 0},
  //                    { 0 , 0 , 1 , 0 , 0},
  //                    { 0 , 0 , 1 , 0 , 0}
  //                  };


  // n = 6;
  // int grid[n][n] = {
  //                    { 0 , 0 , 0 , 0 , 0, 0 },
  //                    { 0 , 1 , 0 , 0 , 0, 0 },
  //                    { 1 , 0 , 0 , 1 , 1, 0 },
  //                    { 1 , 0 , 1 , 0 , 1, 0 },
  //                    { 0 , 0 , 1 , 1 , 1, 1 },
  //                    { 0 , 0 , 0 , 0 , 0, 0 }
  //                  } ;
  //
  // int grid[n][n] = {
  //                     { 0 , 0 , 1 , 0 , 1 , 0 , 0 , 0  },
  //                     { 0 , 0 , 0 , 0 , 1 , 1 , 1 , 0  },
  //                     { 1 , 0 , 1 , 0 , 1 , 0 , 1 , 0  },
  //                     { 1 , 0 , 0 , 0 , 0 , 0 , 0 , 1  },
  //                     { 0 , 1 , 0 , 0 , 0 , 0 , 1 , 1  },
  //                     { 1 , 1 , 0 , 1 , 1 , 0 , 1 , 1  },
  //                     { 1 , 1 , 0 , 1 , 0 , 0 , 0 , 0  },
  //                     { 0 , 0 , 0 , 1 , 0 , 0 , 0 , 0  }
  //                   };
   int grid[n][n];
   MakeGrid(grid, n);

  //NOTE:
  // x = row index, y = column index.

  std::cout << "Grid:" << std::endl;
  std::cout << "1. Points not considered ---> 0" << std::endl;
  std::cout << "2. Obstacles             ---> 1" << std::endl;
  PrintGrid(grid, n);

  //Make sure start and goal are not obstacles and their ids are correctly assigned.
  Node start(0,0,0,0,0,0);
  start.id_ = start.x_ * n + start.y_;
  start.pid_ = start.x_ * n + start.y_;
  Node goal(n-1,n-1,0,0,0,0);
  goal.id_ = goal.x_ * n + goal.y_;

  grid[start.x_][start.y_] = 0;
  grid[goal.x_][goal.y_] = 0;

  RRT_STAR new_rrt_star;
  double threshold = 2;
  int max_iter_x_factor = 20;
  std::vector<Node> path_vector = new_rrt_star.rrt_star(grid, n, start, goal, max_iter_x_factor, threshold);
  PrintPath(path_vector, start, goal, grid, n);

  return 0;
}
#endif
