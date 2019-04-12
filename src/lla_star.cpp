/**
* @file lla_star_lite.cpp
* @author vss2sn
* @brief Contains the LLAStar class
*/

#include "lla_star.hpp"

void LLAStar::VectorInsertionSort(std::vector<Node>& v){
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

double LLAStar::GetHeuristic(Node s1, Node s2){
  return abs(s1.x_ - s2.x_) + abs(s1.y_ - s2.y_);
}

void LLAStar::MyPrint(){
  std::cout << "G values:" << std::endl;
  for(int i=0;i<n;i++){
    for(int j=0;j<n;j++){
      std::cout<< std::setw(5) <<S_[i][j].first << ",";
    }
    std::cout << std::endl;
  }
  std::cout << "RHS values:" << std::endl;
  for(int i=0;i<n;i++){
    for(int j=0;j<n;j++){
      std::cout<< std::setw(5) <<S_[i][j].second << ",";
    }
    std::cout << std::endl;
  }
}

std::pair<double,double> LLAStar::CalculateKey(const Node& s){
  return std::make_pair(std::min(S_[s.x_][s.y_].first, S_[s.x_][s.y_].second)
                                  +GetHeuristic(goal_,s),
                        std::min(S_[s.x_][s.y_].first, S_[s.x_][s.y_].second));
}

std::vector<Node> LLAStar::GetPred(Node u){
  std::vector<Node> pred;
  for(auto it=motions.begin();it!=motions.end(); ++it){
    // Modify to prevent points already in the queue fro  being added?
    Node new_node = u + *it;
    if(new_node.x_ >= n || new_node.x_ < 0 ||
       new_node.y_ >= n || new_node.y_ < 0 ||
       grid[new_node.x_][new_node.y_]==1) continue;
     pred.push_back(new_node);
  }
  return pred;
}

std::vector<Node> LLAStar::GetSucc(Node u){
  std::vector<Node> succ;
  for(auto it=motions.begin();it!=motions.end(); ++it){
    Node new_node = u + *it;
    if(new_node.x_ < n && new_node.x_ >= 0 && new_node.y_ < n && new_node.y_ >= 0){
           if(grid[new_node.x_][new_node.y_]!=1){
         succ.push_back(new_node);
       }
    }
  }
  return succ;
}

void LLAStar::InsertionSort(){
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

double LLAStar::C(Node s1, Node s2){
  if(s1.x_ < n && s1.x_ >= 0 && s1.y_ < n && s1.y_ >= 0 &&
     s2.x_ < n && s2.x_ >= 0 && s2.y_ < n && s2.y_ >= 0 &&
     grid[s1.x_][s1.y_] != 1 && grid[s2.x_][s2.y_] != 1){
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

void LLAStar::Init(){
  U_.clear();
  double n2 = n*n;
  large_num = std::make_pair(n2,n2);

  motions = GetMotion();

  std::vector<std::pair<double,double>> tmp(n);

  std::fill(tmp.begin(), tmp.end(), large_num);
  S_ = std::vector<std::vector<std::pair<double,double>>>(n) ;
  for (int i = 0; i < n; i++){
    S_[i] = tmp;
  }

  S_[start_.x_][start_.y_].second = 0;
  std::pair<Node, std::pair<double, double>> u_pair = std::make_pair(start_, CalculateKey(start_));
  InsertionSort();
  U_.push_back(u_pair);
}

void LLAStar::UpdateVertex(Node& u){
  if(u!=start_){
    std::vector<Node> pred = GetPred(u);
    double init_min = n*n;
    for(int i=0;i<pred.size();i++){
      double new_min = S_[pred[i].x_][pred[i].y_].first + C(u,pred[i]);
      if(new_min < init_min) init_min = new_min;
    }
    S_[u.x_][u.y_].second = init_min;
  }
  // can optimise following by using hash
  for(auto it = U_.begin(); it!=U_.end(); ++it){
    if((*it).first == u){
      U_.erase(it);
      break;
    }
  }
  if(S_[u.x_][u.y_].first != S_[u.x_][u.y_].second){
    std::pair<double,double> key = CalculateKey(u);
    U_.push_back(std::make_pair(u, key));
    InsertionSort();
  }
}

bool LLAStar::CompareKey(std::pair<double,double>& pair_in, Node& u){
  std::pair<double,double> node_key = CalculateKey(u);
  if(pair_in.first < node_key.first ||
    (pair_in.first == node_key.first && pair_in.second < node_key.second)){
    return true;
  }
  return false;
}

int LLAStar::ComputeShortestPath(){
  while((!U_.empty() && CompareKey(U_[0].second, goal_)) || S_[goal_.x_][goal_.y_].first != S_[goal_.x_][goal_.y_].second){
    Node u = U_[0].first;
    U_.erase(U_.begin());
    if(S_[u.x_][u.y_].first > S_[u.x_][u.y_].second){
      S_[u.x_][u.y_].first = S_[u.x_][u.y_].second;
      std::vector<Node> succ = GetSucc(u);
      for(int i = 0;i<succ.size();i++){
        UpdateVertex(succ[i]);
      }
    }
    else{
      S_[u.x_][u.y_].first = n*n;
      std::vector<Node> succ = GetSucc(u);
      for(int i = 0;i<succ.size();i++){
        UpdateVertex(succ[i]);
      }
      UpdateVertex(u);
    }
  }
  if (S_[goal_.x_][goal_.y_] ==  large_num)return -1;
  return 0;
}

std::vector<Node> LLAStar::lla_star(std::vector<std::vector<int>> &grid_in, int n_in, Node start_in, Node goal_in, int max_iter_in){
  max_iter_ = max_iter_in;
  grid = grid_in;
  start_ = start_in;
  goal_ = goal_in;
  n = n_in;
  std::random_device rd; // obtain a random number from hardware
  std::mt19937 eng(rd()); // seed the generator
  std::uniform_int_distribution<int> distr(0,n-1); // define the range
  Init();
  int ans = ComputeShortestPath();
  if(ans < 0 || S_[start_.x_][start_.y_].first==large_num.first){
    path_vector_.clear();
    Node no_path_node(-1,-1,-1,-1,-1);
    path_vector_.push_back(no_path_node);
    grid_in = grid;
    for(int i=0;i<n;i++){
      for(int j=0;j<n;j++){
        if(grid_in[i][j]==2) grid_in[i][j]=0;
      }
    }
    return path_vector_;
  }
  GeneratePathVector();
  while(iter_ < max_iter_){

    if(distr(eng) > n-2){
      int rand = distr(eng)*(path_vector_.size()/(n-1)); // Scaling along path so any point on path could become an obstacle
      Node new_obs = path_vector_[rand];
      std::vector<Node> succ = GetSucc(new_obs);
      SetObs(new_obs);
      for(int i=0;i<succ.size();i++){
        UpdateVertex(succ[i]);
      }
      UpdateVertex(new_obs);
    }
    int ans = ComputeShortestPath();
    if(ans < 0 || S_[start_.x_][start_.y_].first==large_num.first){
      path_vector_.clear();
      Node no_path_node(-1,-1,-1,-1,-1);
      path_vector_.push_back(no_path_node);
      grid_in = grid;
      for(int i=0;i<n;i++){
        for(int j=0;j<n;j++){
          if(grid_in[i][j]==2) grid_in[i][j]=0;
        }
      }
      return path_vector_;
    }
    GeneratePathVector();
    iter_++;
  }
  grid_in = grid;
  grid_in = grid;
  for(int i=0;i<n;i++){
    for(int j=0;j<n;j++){
      if(grid_in[i][j]==2) grid_in[i][j]=0;
    }
  }
  return path_vector_;
}

void LLAStar::SetObs(Node u){
  // PrintGrid(grid,n); // Uncomment if you want to see old and new path
  if(u==goal_ || u==start_){
    std::cout << "Cannot set current start or goal as obstacle" << std::endl;
    return;
  }
  grid[u.x_][u.y_] = 1;
  std::cout << "Obstacle found at: " << std::endl;
  u.PrintStatus();
}

void LLAStar::GeneratePathVector(){
  path_vector_.clear();
  goal_.cost_ = S_[goal_.x_][goal_.y_].second;
  path_vector_.push_back(goal_);
  while(path_vector_[0]!=start_){
    Node u = path_vector_[0];
    grid[u.x_][u.y_]=2;
    for(auto it=motions.begin();it!=motions.end(); ++it){
      Node new_node = u + *it;
      if(new_node.x_ >= n || new_node.x_ < 0 || new_node.y_ >= n || new_node.y_ < 0
         || grid[new_node.x_][new_node.y_]==1){
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
  if(path_vector_[0]==goal_){
    grid[goal_.x_][goal_.y_]=2;
  }
}

void LLAStar::DisplayGrid(){
  std::cout << "Grid: " << std::endl;
  std::cout << "1. Points not considered ---> 0" << std::endl;
  std::cout << "2. Obstacles             ---> 1" << std::endl;
  std::cout << "3. Points considered     ---> 2" << std::endl;
  std::cout << "4. Points in final path  ---> 3" << std::endl;
  std::cout << "5. Current point         ---> 4" << std::endl;
  for(int j=0;j<n;j++){
    std::cout << "---";
  }
  std::cout << std::endl;
  for(int i=0;i<n;i++){
    for(int j=0;j<n;j++){
      if(grid[i][j]==3) std::cout << GREEN << grid[i][j] << RESET << " , ";
      else if(grid[i][j]==1) std::cout << RED << grid[i][j] << RESET << " , ";
      else if(grid[i][j]==2) std::cout << BLUE << grid[i][j] << RESET << " , ";
      else if(grid[i][j]==4) std::cout << YELLOW << grid[i][j] << RESET << " , ";
      else std::cout << grid[i][j] << " , ";
    }
    std::cout << std::endl << std::endl;
  }
  for(int j=0;j<n;j++) std::cout <<  "---";
  std::cout << std::endl;
}

Node LLAStar::NextPoint(){
  int i = 0;
  for(i = 0; i < path_vector_.size(); i++){
    if(goal_ == path_vector_[i]){
      break;
    }
  }
  while(path_vector_[i].pid_!=start_.id_){
    for(int j = 0; j < path_vector_.size(); j++){
      if(path_vector_[i].pid_ == path_vector_[j].id_){
        i=j;
        break;
      }
    }
    if(path_vector_[i].pid_==start_.id_) break;
  }
  return path_vector_[i];
}

#ifdef BUILD_INDIVIDUAL
/**
* @brief Script main function. Generates start and end nodes as well as grid, then creates the algorithm object and calls the main algorithm function.
* @return 0
*/
int main(){
  int n = 8;
  // std::vector<std::vector<int>> grid = {
  //   { 0 , 0 , 1 , 0 , 0 , 1 , 0 , 0 },
  //   { 1 , 0 , 0 , 0 , 0 , 0 , 0 , 0 },
  //   { 0 , 0 , 0 , 1 , 0 , 0 , 1 , 0 },
  //   { 1 , 0 , 0 , 0 , 0 , 0 , 0 , 0 },
  //   { 0 , 0 , 0 , 1 , 0 , 0 , 1 , 0 },
  //   { 0 , 1 , 0 , 1 , 0 , 1 , 0 , 0 },
  //   { 0 , 0 , 0 , 0 , 1 , 0 , 0 , 0 },
  //   { 0 , 0 , 1 , 0 , 0 , 0 , 1 , 0 }
  // };

  std::vector<std::vector<int>> grid(n);
  std::vector<int> tmp(n);
  for (int i = 0; i < n; i++){
    grid[i] = tmp;
  }
  MakeGrid(grid, n);

  Node start(0,0,0,0,0,0);
  Node goal(n-1,n-1,0,0,0,0);

  start.id_ = start.x_ * n + start.y_;
  start.pid_ = start.x_ * n + start.y_;
  goal.id_ = goal.x_ * n + goal.y_;
  start.h_cost_ = abs(start.x_ - goal.x_) + abs(start.y_ - goal.y_);
  //Make sure start and goal are not obstacles and their ids are correctly assigned.
  grid[start.x_][start.y_] = 0;
  grid[goal.x_][goal.y_] = 0;
  int max_iter = n;
  PrintGrid(grid, n);
  std::vector<Node> path_vector;
  LLAStar new_lla_star;
  path_vector = new_lla_star.lla_star(grid, n, start, goal, max_iter);
  PrintPath(path_vector, goal, start, grid, n); //Order of start and goal switched here due to the way LPA* works.
}
#endif BUILD_INDIVIDUAL
