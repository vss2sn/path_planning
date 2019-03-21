/*

D* Lite grid based planning

*/

#include "utils.h"
#include "d_star_lite.h"

using namespace std::chrono;
void VectorInsertionSort(std::vector<Node>& v){
   int n = v.size();
   int i, j;
   Node key;
   for (i = 1; i < n; i++) {
       key = v[i];
       j = i-1;
       while (j >= 0 && (v[j].cost_ + v[j].h_cost_ >= key.cost_+key.h_cost_)){
           v[j+1] = v[j];
           j--;
       }
       v[j+1] = key;
   }
}

double DStarLite::GetHeuristic(Node s1, Node s2){
  return abs(s1.x_ - s2.x_) + abs(s1.y_ - s2.y_);
}

void DStarLite::MyPrint(){
  std::cout << "G values:" << std::endl;
  for(int i=0;i<n;i++){
    for(int j=0;j<n;j++){
      std::cout<< std::setw(5) <<S_[i][j].first << ",";
    }
    std::cout << std::endl;
  }
  for(int i=0;i<n;i++){
    for(int j=0;j<n;j++){
      std::cout<< std::setw(5) <<S_[i][j].second << ",";
    }
    std::cout << std::endl;
  }
}


std::pair<double,double> DStarLite::CalculateKey(const Node& s){
  return std::make_pair(std::min(S_[s.x_][s.y_].first, S_[s.x_][s.y_].second
                                  +GetHeuristic(start_,s)+km_.first),
                        std::min(S_[s.x_][s.y_].first, S_[s.x_][s.y_].second));
}

std::vector<Node> DStarLite::GetPred(Node u){
  std::vector<Node> succ;
  for(auto it=motions.begin();it!=motions.end(); ++it){
    Node new_node = u + *it;
    if(grid[new_node.x_][new_node.y_]==1) continue;
    if(new_node.x_ < n && new_node.x_ >= 0 &&
       new_node.y_ < n && new_node.y_ >= 0){
         succ.push_back(new_node);
    }
  }
  return succ;
}

std::vector<Node> DStarLite::GetSucc(Node u){
  std::vector<Node> succ;
  for(auto it=motions.begin();it!=motions.end(); ++it){
    Node new_node = u + *it;
    if(new_node.x_ < n && new_node.x_ >= 0 &&
       new_node.y_ < n && new_node.y_ >= 0 &&
       grid[new_node.x_][new_node.y_]!=1){
         succ.push_back(new_node);
    }
  }
  return succ;
}

void DStarLite::InsertionSort(){
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


double DStarLite::C(Node s1, Node s2){
  if(grid[s1.x_][s1.y_] != 1 && grid[s2.x_][s2.y_] != 1){
    // Node diff = s2-s1;
    // for(auto it = motions.begin(); it!=motions.end(); ++it){
    //   if(diff == *it){
    //     return (*it).cost_;
    //   }
    // }
    return 1;
  }
  else{
    return n*n;
  }
}

void DStarLite::Init(){
  U_.clear();
  max_iter_ = n*n*n;
  motions = GetMotion();
  km_=std::make_pair(0,0);
  for(int i=0;i<n;i++){
    for(int j=0;j<n;j++){
      S_[i][j] = std::make_pair(n*n,n*n);
    }
  }
  S_[goal_.x_][goal_.y_].second = 0;
  std::pair<Node, std::pair<double, double>> u_pair = std::make_pair(goal_, CalculateKey(goal_));
  U_.push_back(u_pair);
  InsertionSort();
}

void DStarLite::UpdateVertex(Node& u){
  if(u!=goal_){
    std::vector<Node> succ = GetSucc(u);
    double init_min = n*n;
    for(int i=0;i<succ.size();i++){
      double new_min = C(u,succ[i])+S_[succ[i].x_][succ[i].y_].first;
      if(new_min < init_min) init_min = new_min;
    }
    S_[u.x_][u.y_].second = init_min;
  }
  for(auto it = U_.begin(); it!=U_.end(); ++it){
    if((*it).first == u){
      U_.erase(it);
      break;
    }
  }
  if(S_[u.x_][u.y_].first != S_[u.x_][u.y_].second){
    U_.push_back(std::make_pair(u, CalculateKey(u)));
    InsertionSort();
  }
}

bool DStarLite::CompareKey(std::pair<double,double>& pair_in, Node& u){
  std::pair<double,double> node_key = CalculateKey(u);
  if(pair_in.first < node_key.first ||
    (pair_in.first == node_key.first && pair_in.second < node_key.second)){
    return true;
  }
  return false;
}

int DStarLite::ComputeShortestPath(){
  ++iter_;
  if(iter_==max_iter_) return -1;
  while((!U_.empty() && CompareKey(U_[0].second, start_)) || S_[start_.x_][start_.y_].first != S_[start_.x_][start_.y_].second){
    k_old_ = U_[0].second;
    Node u = U_[0].first;
    U_.erase(U_.begin());
    if(CompareKey(k_old_, u)){
      std::pair<Node, std::pair<double, double>> u_pair = std::make_pair(goal_, CalculateKey(goal_));
      U_.push_back(u_pair);
      InsertionSort();
    }
    else if (S_[u.x_][u.y_].first > S_[u.x_][u.y_].second){
      S_[u.x_][u.y_].first = S_[u.x_][u.y_].second;
      std::vector<Node> pred = GetPred(u);
      for(int i = 0;i<pred.size();i++){
        UpdateVertex(pred[i]);
      }
    }
    else{
      S_[u.x_][u.y_].first = n*n;
      std::vector<Node> pred = GetPred(u);
      for(int i = 0;i<pred.size();i++){
        UpdateVertex(pred[i]);
      }
      UpdateVertex(u);
    }
  }
  return 0;
}

void DStarLite::CopyGrid(void *grid_in){
  int (*p_grid)[n][n] = (int (*)[n][n]) grid_in;
  for(int i=0;i<n;i++){
    for(int j=0;j<n;j++){
      (*p_grid)[i][j] = grid[i][j];
    }
  }
}

std::vector<Node> DStarLite::d_star_lite(void *grid_in, int n_in, Node start_in, Node goal_in){
  start_ = start_in;
  main_start_ = start_;
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
  int ans = ComputeShortestPath();
  CopyGrid(grid_in);
  if(ans < 0){
    path_vector_.clear();
    Node no_path_node(-1,-1,-1,-1,-1);
    path_vector_.push_back(no_path_node);
  }
  else GeneratePathVector();
  return ReturnInvertedVector();
}

std::vector<Node> DStarLite::Replan(void *grid_in, Node u){
  grid[u.x_][u.y_] = 1; //cant just do this; need to undo generate grid
  if (grid[start_.x_][start_.y_]==1) grid[start_.x_][start_.y_]=0;
  path_vector_.clear();
  start_ = main_start_;
  iter_=0;
  while(start_!=goal_){
    std::vector<Node> succ;
    succ = GetSucc(start_);
    double init_min = n*n;
    double new_min = 0;
    Node new_start = Node(start_.x_, start_.y_);
    for(int i = 0;i<succ.size();i++){
      new_min = C(start_,succ[i])+S_[succ[i].x_][succ[i].y_].first;
      if(new_min < init_min){
        init_min = new_min;
        new_start = succ[i];
      }
    }
    start_ = new_start;
    km_.first = km_.first + GetHeuristic(last_,start_);
    last_ = start_;
    UpdateVertex(u);
    succ = GetSucc(u);
    for(int i = 0;i<succ.size();i++){
      UpdateVertex(succ[i]);
    }
    int ans = ComputeShortestPath();
    if(ans < 0){
      path_vector_.clear();
      Node no_path_node(-1,-1,-1,-1,-1);
      path_vector_.push_back(no_path_node);
      CopyGrid(grid_in);
      return path_vector_;
    }
  }
  CopyGrid(grid_in);
  GeneratePathVector();
  return ReturnInvertedVector();
}


std::vector<Node> DStarLite::ReturnInvertedVector(){
  std::vector<Node> inverted_path_vector = path_vector_;
  // Inverting costs as dstar moves from goal to start.
  // Then inverting path vector for reordering.
  double start_cost = inverted_path_vector.back().cost_;
  for(auto it=inverted_path_vector.begin(); it!=inverted_path_vector.end(); ++it){
    (*it).cost_ = start_cost - (*it).cost_;
  }
  std::reverse(inverted_path_vector.begin(),inverted_path_vector.end());
  return inverted_path_vector;
}

void DStarLite::GeneratePathVector(){
  main_start_.cost_ = S_[main_start_.x_][main_start_.y_].second;
  path_vector_.push_back(main_start_);
  while(path_vector_[0]!=goal_){
    Node u = path_vector_[0];
    grid[u.x_][u.y_]=2;
    for(auto it=motions.begin();it!=motions.end(); ++it){
      Node new_node = u + *it;
      if(grid[new_node.x_][new_node.y_]==1){
        continue;
      }
      if(new_node.x_ < n && new_node.x_ >= 0 && new_node.y_ < n && new_node.y_ >= 0){
        new_node.cost_= S_[new_node.x_][new_node.y_].second;
        if(new_node.cost_ > u.cost_){
           continue;
        }
        new_node.id_ = n*new_node.x_ + new_node.y_;
        new_node.pid_ = u.id_;
        path_vector_.push_back(new_node);
        VectorInsertionSort(path_vector_);
      }
    }
  }

}


std::vector<Node> DStarLite::UpdateStart(void* grid_in, Node start_in){
  start_ = start_in;
  main_start_ = start_;
  km_.first = km_.first +GetHeuristic(last_, start_);
  last_ = start_;
  //TODO: Check if this section is required
  int ans = ComputeShortestPath();
  CopyGrid(grid_in);
  if(ans < 0){
    path_vector_.clear();
    Node no_path_node(-1,-1,-1,-1,-1);
    path_vector_.push_back(no_path_node);
  }
  //End TODO
  return Replan(grid_in, start_);
}

#ifdef BUILD_INDIVIDUAL
int main(){
  int n = 8;
  int num_points = n*n;

  int main_grid[n][n];
  int grid[n][n]={
{ 0 , 0 , 0 , 0 , 0 , 0 , 1 , 1 },
{ 1 , 0 , 0 , 1 , 1 , 0 , 0 , 0 },
{ 0 , 0 , 0 , 0 , 0 , 0 , 0 , 1 },
{ 1 , 0 , 0 , 0 , 0 , 1 , 1 , 0 },
{ 0 , 0 , 0 , 0 , 1 , 0 , 1 , 0 },
{ 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 },
{ 0 , 0 , 1 , 0 , 0 , 0 , 1 , 1 },
{ 0 , 1 , 1 , 0 , 0 , 0 , 0 , 0 }
  };

  //MakeGrid(grid, n);
  PrintGrid(grid, n);
  int grid_space = n*n*sizeof(int);

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
  path_vector = new_d_star_lite.d_star_lite(grid, n, start, goal);
  PrintPath(path_vector, start, goal, grid, n);

  // Adding obstacle to path.
  Node new_obs;
  if(path_vector.size() > 3){
    for(int j = 0; j < path_vector.size(); j++){
      if(path_vector[0].id_ == path_vector[j].pid_ && path_vector[0]!=path_vector[j]){
        // path_vector[j].PrintStatus();
        new_obs = path_vector[j];
        break;
      }
    }
    std::cout << "Obstacle created at: "<< std::endl;
    new_obs.PrintStatus();
    grid[new_obs.x_][new_obs.y_] = 1;
    grid[start.x_][start.y_] = 0;
    grid[goal.x_][goal.y_] = 0;
    path_vector = new_d_star_lite.Replan(grid, new_obs);
  }
  else std::cout << "Path size too small; no new obstacle created" << std::endl;
  PrintPath(path_vector, start, goal, grid, n);

  // Updating start. TODO: Test more.
  start = Node(4,4);
  auto start_in = high_resolution_clock::now();
  path_vector = new_d_star_lite.UpdateStart(grid, start);
  auto stop = high_resolution_clock::now();
  auto duration = duration_cast<microseconds>(stop - start_in);
  std::cout << duration.count() << std::endl;
  PrintPath(path_vector, start, goal, grid, n);

  return 0;
}
#endif BUILD_INDIVIDUAL
