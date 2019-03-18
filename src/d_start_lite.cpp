/*

A* grid based planning

*/

#include "utils.h"
#include "d_star_lite.h"

double DStarLite::get_heuristic(Node s1, Node s2){
  return abs(s1.x_ - s2.x_) + abs(s1.y_ - s2.y_);
}

void DStarLite::my_print(){
  std::cout << "G values:" << std::endl;
  for(int i=0;i<n;i++){
    for(int j=0;j<n;j++){
      std::cout<< std::setw(5) <<S_[i][j].first << ",";
    }
    std::cout << std::endl;
  }
  // std::cout << "RHS values:" << std::endl;
  // for(int i=0;i<n;i++){
  //   for(int j=0;j<n;j++){
  //     std::cout<< std::setw(10) << S_[i][j].second << ",";
  //   }
  //   std::cout << std::endl;
  // }
}


std::pair<double,double> DStarLite::CalculateKey(const Node& s){
  return std::make_pair(std::min(S_[s.x_][s.y_].first, S_[s.x_][s.y_].second
                                  +get_heuristic(start_,s)+km_.first),
                        std::min(S_[s.x_][s.y_].first, S_[s.x_][s.y_].second));
}

std::vector<Node> DStarLite::GetPred(Node u){
  std::vector<Node> motions = GetMotion();
  std::vector<Node> succ;
  for(auto it=motions.begin();it!=motions.end(); ++it){
    Node new_node = u + *it;
    // new_node.PrintStatus();
    if(grid[new_node.x_][new_node.y_]==1) continue;
    //if(S_[new_node.x_][new_node.y_].first - S_[u.x_][u.y_].first == (*it).cost_) continue; //new_node is child
    if(new_node.x_ < n && new_node.x_ >= 0 &&
       new_node.y_ < n && new_node.y_ >= 0){
         succ.push_back(new_node);
    }
  }
  return succ;
}

std::vector<Node> DStarLite::GetSucc(Node u){
  std::vector<Node> succ;
  std::vector<Node> motions;
  motions.clear();
  motions = GetMotion();
  for(int i=0;i<motions.size();i++){
  }

  for(auto it=motions.begin();it!=motions.end(); ++it){
    Node new_node = u + *it;
    // if(S_[u.x_][u.y_].second - S_[new_node.x_][new_node.y_].second == -(*it).cost_) continue; //new_node is parent
    if(new_node.x_ < n && new_node.x_ >= 0 &&
       new_node.y_ < n && new_node.y_ >= 0 &&
       grid[new_node.x_][new_node.y_]!=1){
         succ.push_back(new_node);
    }
  }
  return succ;
}

void DStarLite::insertionSort(){
   int n = U_.size();
   int i, j;
   std::pair<Node,std::pair<double,double>> key;

   for (i = 1; i < n; i++) {
       key = U_[i];
       j = i-1;
       while (j >= 0 && (U_[j].second.first > key.second.first
                          || (U_[j].second.first == key.second.first  && U_[j].second.second > key.second.second))){
           U_[j+1] = U_[j];
           j--;
       }
       U_[j+1] = key;
   }
}


double DStarLite::c(Node s1, Node s2){
  if(grid[s1.x_][s1.y_] != 1 && grid[s2.x_][s2.y_] != 1){
    std::vector<Node> motions = GetMotion();
    Node diff = s2-s1;
    for(auto it = motions.begin(); it!=motions.end(); ++it){
      if(diff == *it){
        return (*it).cost_;
      }
    }
  }
  else{
    return n*n;
  }
}

void DStarLite::Init(){
  U_.clear();
  km_=std::make_pair(0,0);
  for(int i=0;i<n;i++){
    for(int j=0;j<n;j++){
      S_[i][j] = std::make_pair(n*n,n*n);
    }
  }
  S_[goal_.x_][goal_.y_].second = 0;
  std::pair<Node, std::pair<double, double>> u_pair = std::make_pair(goal_, CalculateKey(goal_));
  U_.push_back(u_pair);
  insertionSort();
}

void DStarLite::UpdateVertex(Node u){
  // std::cout << "begin" << U_.size()<<std::endl;

  if(U_.size()>20) exit;
  // usleep(100000);
  if(u!=goal_){
    std::vector<Node> succ = GetSucc(u);
    double init_min = n*n;
    for(int i=0;i<succ.size();i++){
      // usleep(50000);
      double new_min = c(u,succ[i])+S_[succ[i].x_][succ[i].y_].first;
      if(new_min < init_min) init_min = new_min;
    }
    // std::cout << U_.size()<<std::endl;
    S_[u.x_][u.y_].second = init_min;
    // std::cout << U_.size()<<std::endl;
  }
  // usleep(100000);
  for(auto it = U_.begin(); it!=U_.end(); ++it){
    // usleep(100000);
    // std::cout << "about to compare" << std::endl;
    // std::cout << U_.size()<<std::endl;
    // (*it).first.PrintStatus();
    // u.PrintStatus();
    if((*it).first == u){
      U_.erase(it);
      break;
    }
  }
  // std::cout << "end compare" << std::endl;
  if(S_[u.x_][u.y_].first != S_[u.x_][u.y_].second){
    U_.push_back(std::make_pair(u, CalculateKey(u)));
    insertionSort();
  }
  // std::cout << "end updatevertex" << U_.size()<<std::endl;

}

int DStarLite::ComputeShortestPath(){
  while((U_[0].second.first < CalculateKey(start_).first || (U_[0].second.first == CalculateKey(start_).first && U_[0].second.second < CalculateKey(start_).second)) || S_[start_.x_][start_.y_].first != S_[start_.x_][start_.y_].second){
    // std::cout << "begin while " << U_.size()<<std::endl;
    if(U_.size()==0) return -1;
    k_old_ = U_[0].second;
    Node u = U_[0].first;
    // usleep(1000000);
    U_.erase(U_.begin());

    if((k_old_.first < CalculateKey(u).first || k_old_.first == CalculateKey(u).first && k_old_.second < CalculateKey(u).second)){
      // std::cout << "begin if " << U_.size()<<std::endl;

      std::pair<Node, std::pair<double, double>> u_pair = std::make_pair(goal_, CalculateKey(goal_));
      U_.push_back(u_pair);
      insertionSort();
      // std::cout << "end if " << U_.size()<<std::endl;

    }
    else if (S_[u.x_][u.y_].first > S_[u.x_][u.y_].second){
      // std::cout << "begin else if " << U_.size()<<std::endl;

      S_[u.x_][u.y_].first = S_[u.x_][u.y_].second;
      std::vector<Node> pred = GetPred(u);
      // std::cout << "PRED"<< std::endl;
      for(int i = 0;i<pred.size();i++){
        // pred[i].PrintStatus();
        UpdateVertex(pred[i]);
      }
      // std::cout << "end else if " << U_.size()<<std::endl;
    }
    else{
      // std::cout << "begin else " << U_.size()<<std::endl;
      S_[u.x_][u.y_].first = n*n;
      std::vector<Node> pred = GetPred(u);
      for(int i = 0;i<pred.size();i++){
        UpdateVertex(pred[i]);
      }
      UpdateVertex(u);
      // std::cout << "end else " << U_.size()<<std::endl;

    }
  }
  return 0;
}

int DStarLite::d_star_lite(void *grid_in, int n_in, Node start_in, Node goal_in){
  start_ = start_in;
  goal_ = goal_in;
  n = n_in;
  int (*p_grid)[n][n] = (int (*)[n][n]) grid_in;
  for(int i=0;i<n;i++){
    for(int j=0;j<n;j++){
      grid[i][j] = (*p_grid)[i][j];
    }
  }
  last_ = start_;
  Init();
  return ComputeShortestPath();
}

int DStarLite::replan(Node u){
  grid[u.x_][u.y_] = 1;
  start_ = main_start_;
  while(start_!=goal_){
    std::vector<Node> succ;
    succ = GetSucc(start_);
    double init_min = n*n;
    double new_min = 0;
    Node new_start = Node(start_.x_, start_.y_);
    for(int i = 0;i<succ.size();i++){
      new_min = c(start_,succ[i])+S_[succ[i].x_][succ[i].y_].first;
      if(new_min < init_min){
        init_min = new_min;
        new_start = succ[i];
      }
    }
    start_ = new_start;
    km_.first = km_.first + get_heuristic(last_,start_);
    last_ = start_;
    UpdateVertex(u);
    succ = GetSucc(u);
    for(int i = 0;i<succ.size();i++){
      UpdateVertex(succ[i]);
    }
    int ans = ComputeShortestPath();
    if(ans < 0) return -1;
  }
  return 0;
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
  std::vector<Node> path_vector;

  DStarLite new_d_star_lite;
  int ans = new_d_star_lite.d_star_lite(grid, n, start, goal);
  if (ans < 0){
    std::cout << "no path" << std::endl;
    exit;
  }
  PrintGrid(grid, n);
  new_d_star_lite.my_print();
  std::cout << "Obstacle = 1,0" << std::endl;
  grid[1][0]=1;
  ans = new_d_star_lite.replan(Node(1,0));
  if (ans < 0){
    std::cout << "no path" << std::endl;
    exit;
  }
  PrintGrid(grid, n);
  new_d_star_lite.my_print();
  std::cout << "Obstacle = 0,1" << std::endl;
  grid[0][1]=1;
  ans = new_d_star_lite.replan(Node(0,1));
  if (ans < 0){
    std::cout << "no path" << std::endl;
    exit;
  }
  PrintGrid(grid, n);
  new_d_star_lite.my_print();
  std::cout << "Obstacle = n-2,n-2" << std::endl;
  grid[n-2][n-2]=1;
  ans = new_d_star_lite.replan(Node(n-2,n-2));
  PrintGrid(grid, n);
  new_d_star_lite.my_print();
  if (ans < 0){
    std::cout << "no path" << std::endl;
    exit;
  }
  // grid[n-2][n-2]=1;
  // replan(Node(n-2,n-2));
  // my_print();
  //
  return 0;
}

#endif BUILD_INDIVIDUAL
