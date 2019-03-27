/**
* @file d_star_lite.cpp
* @author vss2sn
* @brief Contains the DStarLite class
*/

#include "d_star_lite.h"

/**
* @brief Using insertion sort to sort the vector list.
* @param v The vector to be sorted
* @return void
*/
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

/**
* @brief Calculate and return the heuristic distance between 2 nodes
* @param s1 Node 1
* @param s2 Node 2
* @return Heuritic distance between the 2 nodes
*/
double DStarLite::GetHeuristic(Node s1, Node s2){
  return abs(s1.x_ - s2.x_) + abs(s1.y_ - s2.y_);
}

/**
* @brief Displays the G and RHS values for the entire grid.
* @return void
*/
void DStarLite::MyPrint(){
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

/**
* @brief Returns the key (pair) values for a given node.
* @param s Node whose key values are to be calcualted
* @return Key for given input node
*/
std::pair<double,double> DStarLite::CalculateKey(const Node& s){
  std::cout << "In cal"<< std::endl;
  auto key = std::make_pair(std::min(S_[s.x_][s.y_].first, S_[s.x_][s.y_].second
                                  +GetHeuristic(start_,s)+km_.first),
                        std::min(S_[s.x_][s.y_].first, S_[s.x_][s.y_].second));
std::cout << "out cal"<< std::endl;

  return key;
}

/**
* @brief Returns the possible predecessors of a given node, based on allowed motion primatives
* @param u Node
* @return Vector of nodes that are possible predecessors to input node
*/
std::vector<Node> DStarLite::GetPred(Node u){
  std::cout << "in getpr"<< std::endl;

  std::vector<Node> pred;
  for(auto it=motions.begin();it!=motions.end(); ++it){
    std::cout << "hello"<< std::endl;

    Node new_node = u + *it;

    if(new_node.x_ >= n || new_node.x_ < 0 ||
       new_node.y_ >= n || new_node.y_ < 0 ||
       grid[new_node.x_][new_node.y_]==1) continue;
     std::cout << "aboe pushgetpr"<< std::endl;
     pred.push_back(new_node);
     std::cout << "belo pushgetpr"<< std::endl;
  }
  std::cout << "about to return"<< std::endl;

  return pred;
}

/**
* @brief Returns the possible successors of a given node, based on allowed motion primatives
* @param u Node
* @return Vector of nodes that are possible successors to input node
*/
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

/**
* @brief Using insertion sort to sort the vector list that maintains the priority queue. Good for a mostly sorted queue. Sort called after every insertion to maintain queue. Not using standard queue as iterating over is not allowed.
* @return void
*/
void DStarLite::InsertionSort(){
  std::cout << "in sort" << std::endl;
   int n = U_.size();
   int i, j;
   std::pair<Node,std::pair<double,double>> key;

   for (i = 1; i < n; i++) {
       key = U_[i];
       j = i-1;
       while (j >= 0 && (U_[j].second.first > key.second.first
                          || (U_[j].second.first == key.second.first  && U_[j].second.second >= key.second.second))){
           U_[j+1] = U_[j];
           j--;
       }
       U_[j+1] = key;
   }
   std::cout << "out sort" << std::endl;

}

/**
* @brief Returns the cost of motion moving from one node to another, based on allowed motion primatives. Currently set to 1 for speed up given only 4 motions permitted, at constant cost. Change as required, based on permitted motions.
* @param s1 Node
* @param s2 Node
* @return cost of motion moving from first node to second
*/
double DStarLite::C(Node s1, Node s2){
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

/**
* @brief Initialisation function of D*. Initialises G and RHS values for all nodes, store motion allowable primatives, km value and the first value of the priority queue.
* @return void
*/
void DStarLite::Init(){
  std::cout << "hello" << std::endl;
  U_.clear();
  std::cout << "hello" << std::endl;

  iter_ = 0;
  double n2 = n*n;
  large_num = std::make_pair(n2,n2);
  max_iter_ = n2*n;
  std::cout << "hello" << std::endl;

  motions = GetMotion();
  std::cout << "hello" << std::endl;

  km_=std::make_pair(0,0);

  std::vector<std::pair<double,double>> tmp(n);
  std::cout << "filing" << std::endl;

  std::fill(tmp.begin(), tmp.end(), large_num);
  std::cout << "filled" << std::endl;
  S_ = std::vector<std::vector<std::pair<double,double>>>(n) ;
  for (int i = 0; i < n; i++){
    std::cout << "In loop"<< i<<std::endl;
    S_[i] = tmp;
  }
  std::cout << "out loop"<< std::endl;
  //MyPrint();
  // for(int i=0;i<n;i++){
  //   for(int j=0;j<n;j++){
  //     std::cout << "setting" << std::endl;
  //
  //     S_[i][j] = large_num;
  //     std::cout << "donesetting" << std::endl;
  //
  //   }
  // }
  std::cout << "largenum" << std::endl;

  S_[goal_.x_][goal_.y_].second = 0;
  std::pair<Node, std::pair<double, double>> u_pair = std::make_pair(goal_, CalculateKey(goal_));
  std::cout << U_.size() << std::endl;
  U_.push_back(u_pair);
  std::cout << U_.size() << std::endl;
  InsertionSort();
}

/**
* @brief Update vertex procedure as per D* Lite algorithm, Figure 3.
* @param u Node on which UpdateVertex has to be called
* @return void
*/
void DStarLite::UpdateVertex(Node& u){
  std::cout << "In update"<< std::endl;
  if(u!=goal_){
    std::cout << "get succ"<< std::endl;
    std::vector<Node> succ = GetSucc(u);
    std::cout << "out succ"<< std::endl;
    double init_min = n*n;
    for(int i=0;i<succ.size();i++){
      std::cout << "in c" << std::endl;
      double new_min = C(u,succ[i])+S_[succ[i].x_][succ[i].y_].first;
      std::cout << "out c"<< std::endl;
      if(new_min < init_min) init_min = new_min;
    }
    std::cout << "hello_1" << std::endl;

    S_[u.x_][u.y_].second = init_min;
    std::cout << "hello_2" << std::endl;

  }
  for(auto it = U_.begin(); it!=U_.end(); ++it){
    std::cout << "In for"<< std::endl;
    if((*it).first == u){
      U_.erase(it);
      break;
    }
  }
  std::cout << "before if"<< std::endl;
  if(S_[u.x_][u.y_].first != S_[u.x_][u.y_].second){
    std::cout << "In if"<< std::endl;
    std::pair<double,double> key = CalculateKey(u);
    std::cout << "key returned"<< std::endl;
    u.PrintStatus();
    std::cout << U_.size() << std::endl;

    U_.push_back(std::make_pair(u, key));
    std::cout << U_.size() << std::endl;

    std::cout << "above sort" << std::endl;
    InsertionSort();
    std::cout << "below sort" << std::endl;
  }
  std::cout << "Out update"<< std::endl;
}

/**
* @brief Compare keys function for D* Lite. Compares the key given as input with the key values of the node given as input. Calls CalculateKey on the node.
* @param pair_in Key pair
* @param u Node whose key values will be calculated and compared to above key pair
* @return bool value based on comparison of key values
*/
bool DStarLite::CompareKey(std::pair<double,double>& pair_in, Node& u){
  std::pair<double,double> node_key = CalculateKey(u);
  if(pair_in.first < node_key.first ||
    (pair_in.first == node_key.first && pair_in.second < node_key.second)){
    return true;
  }
  return false;
}

/**
* @brief ComputeShortestPath procedure as per D* Lite algorithm, Figure 3.
* @return void
*/
int DStarLite::ComputeShortestPath(){
  ++iter_;
  if(iter_>=max_iter_){ // >= coz testing in multiple places for break condition
    return -1;
  }
  while((!U_.empty() && CompareKey(U_[0].second, start_)) || S_[start_.x_][start_.y_].first != S_[start_.x_][start_.y_].second){
    k_old_ = U_[0].second;
    Node u = U_[0].first;
    U_.erase(U_.begin());
    std::cout << "above else if"<< std::endl;
    if(CompareKey(k_old_, u)){
      std::cout << "in if"<< std::endl;
      std::pair<Node, std::pair<double, double>> u_pair = std::make_pair(goal_, CalculateKey(goal_));
      U_.push_back(u_pair);
      InsertionSort();
    }
    else if (S_[u.x_][u.y_].first > S_[u.x_][u.y_].second){
      std::cout << "in else if"<< std::endl;
      S_[u.x_][u.y_].first = S_[u.x_][u.y_].second;
      std::cout << "in belo else if"<< std::endl;
      std::vector<Node> pred = GetPred(u);
      std::cout << "below getpr"<< std::endl;
      for(int i = 0;i<pred.size();i++){
        std::cout << "above udv"<< std::endl;
        UpdateVertex(pred[i]);
        std::cout << "below udv"<< std::endl;

      }
    }
    else{
      std::cout << "in else"<< std::endl;
      S_[u.x_][u.y_].first = n*n;
      std::vector<Node> pred = GetPred(u);
      for(int i = 0;i<pred.size();i++){
        UpdateVertex(pred[i]);
      }
      UpdateVertex(u);
    }
    std::cout << "out if"<< std::endl;
  }
  return 0;
}

/**
* @brief Main algorithm of D* Lite
* @param grid_in Main grid
* @param n_in number of rows/columns
* @param start_in starting node
* @param goal_in goal node
* @return path vector of nodes
*/
std::vector<Node> DStarLite::d_star_lite(std::vector<std::vector<int>> &grid_in, int n_in, Node start_in, Node goal_in){
  std::cout << "in algo" << std::endl;
  grid = grid_in;
  std::cout << "addigned"<< std::endl;
  start_ = start_in;
  main_start_ = start_;
  goal_ = goal_in;
  last_ = start_;
  std::cout << "preinit" << std::endl;
  n = n_in;
  Init();
  std::cout << "init" << std::endl;

  int ans = ComputeShortestPath();
  std::cout << "compute" << std::endl;

  if(ans < 0 || S_[start_.x_][start_.y_].first==large_num.first){
    path_vector_.clear();
    Node no_path_node(-1,-1,-1,-1,-1);
    path_vector_.push_back(no_path_node);
  }
  else GeneratePathVector();
  std::cout << "gen" << std::endl;

  return ReturnInvertedVector();
}

/**
* @brief Create an obstacle on input node. Does not allow start or goal to be declared an obstacle. Prints out the obstacle if created and displays the grid. Calls Replan function.
* @param u Node at which obstacle is to be created
* @return path vector of nodes
*/
std::vector<Node> DStarLite::SetObs(Node u){
  if(u==goal_ || u==start_){
    std::cout << "Cannot set current start or goal as obstacle" << std::endl;
    return path_vector_;
  }
  grid[u.x_][u.y_] = 1;
  std::cout << "Obstacle found at: " << std::endl;
  u.PrintStatus();
  DisplayGrid();
  return Replan(u);
}

/**
* @brief Replan route, called whenever a previously unknown obstacle is detected.
        Equivalent of the effects of the code after an edge change is detectedd in the while loop within the main procedure of D* Lite.
* @param u Node at which the change was detected
* @return path vector of nodes
*/
std::vector<Node> DStarLite::Replan(Node u){
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
    for(int i =0; i< succ.size(); i++){
      UpdateVertex(succ[i]);
    }
    int ans = ComputeShortestPath();
    if(ans < 0){
      path_vector_.clear();
      Node no_path_node(-1,-1,-1,-1,-1);
      path_vector_.push_back(no_path_node);
      return path_vector_;
    }
  }
  GeneratePathVector();
  return ReturnInvertedVector();
}

/**
* @brief As D* Lite moves from goal to start, inverts the path vector taht has been generated as well as the cost, ensuring that cost to start is 0.
* @return path vector of nodes
*/
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

/**
* @brief Generate the path vector and set the appropriate grid values
* @return void
*/
void DStarLite::GeneratePathVector(){
  std::cout << "in gen" << std::endl;
  main_start_.cost_ = S_[main_start_.x_][main_start_.y_].second;
  path_vector_.push_back(main_start_);
  while(path_vector_[0]!=goal_){
    std::cout << "in gen while1" << std::endl;
    Node u = path_vector_[0];
    grid[u.x_][u.y_]=2;
    std::cout << "in gen while2" << std::endl;

    for(auto it=motions.begin();it!=motions.end(); ++it){
      Node new_node = u + *it;
      std::cout << "in gen while3" << std::endl;
      if(new_node.x_ >= n || new_node.x_ < 0 || new_node.y_ >= n || new_node.y_ < 0
         || grid[new_node.x_][new_node.y_]==1){
        std::cout << "in gen while4" << std::endl;
        continue;
      }
      std::cout << "in gen while5" << std::endl;
      if(new_node.x_ < n && new_node.x_ >= 0 && new_node.y_ < n && new_node.y_ >= 0){
        std::cout << "in gen while6" << std::endl;
        new_node.cost_= S_[new_node.x_][new_node.y_].second;
        if(new_node.cost_ > u.cost_){
          std::cout << "in gen while7" << std::endl;
           continue;
        }
        new_node.id_ = n*new_node.x_ + new_node.y_;
        new_node.pid_ = u.id_;
        path_vector_.push_back(new_node);
        VectorInsertionSort(path_vector_);
        std::cout << "in gen while8" << std::endl;
      }
    }
  }
  if(path_vector_[0]==goal_){
    grid[goal_.x_][goal_.y_]=2;
  }
}

/**
* @brief Update the starting point of the algorithm. Created to be independant, used by RunDStarLite function to update the position of the bot. If using independantly, uncomment the commented section within the function. Letting it return path_vector_ as that is required for independent run.
* @param start_in new starting position
* @return Path vector of nodes. Can be made to void, but left as path vector to allow independent call.
*/


std::vector<Node> DStarLite::UpdateStart(Node start_in){
  // Prevent teleportations
  if(path_vector_[0].cost_ == -1){
    std::cout << "Teleport disabled." << std::endl;
    return path_vector_;
  }
  //If no path found to
  // goal from current start, not movement from start will reach a point that
  // can be reached from the goal. Teleportation is not supported by D* Lite
  // unless a second iter counter is added at the beginning of the compute cost
  // before the while loop.
  start_ = start_in;
  main_start_ = start_;
  km_.first = km_.first +GetHeuristic(last_, start_);
  last_ = start_;
  // int ans = ComputeShortestPath();
  // if(ans < 0){
  //   path_vector_.clear();
  //   Node no_path_node(-1,-1,-1,-1,-1);
  //   path_vector_.push_back(no_path_node);
  // }
  //return Replan(start_);
  return path_vector_;
}

/**
 * Displays the grid stored by the D* Lite object.
 */
void DStarLite::DisplayGrid(){
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

/**
* @brief Find and return the next point in the path_vector
* @return next point node
*/
Node DStarLite::NextPoint(){
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
/**
* @brief Function to run D* Lite live, showing the movement of the bot with time. Timeout after each movement set in .h file. Next point in path might beset to obstacle with probability 1/n. Calls UpdateStart and SetObs.
* @param disp_inc_in Bool value to allow display incremental progress
* @return void
*/
void DStarLite::RunDStarLite(bool disp_inc_in){
  disp_inc = disp_inc_in;
  if(path_vector_[0].cost_==-1){
    std::cout << "No path" << std::endl;
    return; // No path
  }
  std::random_device rd; // obtain a random number from hardware
  std::mt19937 eng(rd()); // seed the generator
  std::uniform_int_distribution<int> distr(0,n); // define the range
  Node current = path_vector_.back();
  while(current!=goal_){
    grid[start_.x_][start_.y_] = 3;
    UpdateStart(current);
    start_ = current;
    Node next_point = NextPoint();
    if(distr(eng) > n-2 && next_point!=goal_) SetObs(next_point);
    grid[current.x_][current.y_] = 4;
    if(disp_inc){
      usleep(disp_p);
      DisplayGrid();
    }
    if(path_vector_[0].cost_==-1){
      DisplayGrid(); // Shows path traversed and current point
      std::cout << "No path" << std::endl;
      return; // No path
    }
    start_ = current;
    current = NextPoint();
  }
  grid[start_.x_][start_.y_] = 3;
  grid[current.x_][current.y_] = 4;
  if(disp_inc) {
    usleep(disp_p);
    DisplayGrid();
  }
  grid[current.x_][current.y_] = 3;
  DisplayGrid();
  return;
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

  Node start(1,1,0,0,0,0);
  Node goal(n-1,n-1,0,0,0,0);

  start.id_ = start.x_ * n + start.y_;
  start.pid_ = start.x_ * n + start.y_;
  goal.id_ = goal.x_ * n + goal.y_;
  start.h_cost_ = abs(start.x_ - goal.x_) + abs(start.y_ - goal.y_);
  //Make sure start and goal are not obstacles and their ids are correctly assigned.
  grid[start.x_][start.y_] = 0;
  grid[goal.x_][goal.y_] = 0;
  PrintGrid(grid, n);
  std::vector<Node> path_vector;
std::cout << "hello" << std::endl;
  DStarLite new_d_star_lite;
  std::cout << "hello2" << std::endl;

  path_vector = new_d_star_lite.d_star_lite(grid, n, start, goal);
  PrintPath(path_vector, start, goal, grid, n);

  //new_d_star_lite.RunDStarLite();
  return 0;
}
#endif BUILD_INDIVIDUAL
