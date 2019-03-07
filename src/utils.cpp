#include "utils.h"

Node::Node(int x = 0, int y = 0, double cost = 0, double h_cost = 0, int id = 0, int pid = 0){
  this->x_ = x;
  this->y_ = y;
  this->cost_ = cost;
  this->h_cost_ = h_cost;
  this->id_ = id;
  this->pid_ = pid;
}
void Node::PrintStatus(){
  std::cout << "--------------"              << std::endl
            << "Node          :"             << std::endl
            << "x             : " << x_      << std::endl
            << "y             : " << y_      << std::endl
            << "cost          : " << cost_   << std::endl
            << "Heuristic cost: " << h_cost_ << std::endl
            << "id            : " << id_     << std::endl
            << "pid           : " << pid_    << std::endl
            << "--------------"              << std::endl;
}
Node Node::operator+(Node p){
  Node tmp;
  tmp.x_ = this->x_ + p.x_;
  tmp.y_ = this->y_ + p.y_;
  tmp.cost_ = this->cost_ + p.cost_;
  return tmp;
}
Node Node::operator=(Node p){
  this->x_ = p.x_;
  this->y_ = p.y_;
  this->cost_ = p.cost_;
  this->h_cost_ = p.h_cost_;
  this->id_ = p.id_;
  this->pid_ = p.pid_;
}
bool Node::operator==(Node p){
  if (this->x_ == p.x_ && this->y_ == p.y_) return true;
  return false;
}
bool Node::operator!=(Node p){
  if (this->x_ != p.x_ || this->y_ != p.y_) return true;
  return false;
}


bool compare_cost::operator()(Node& p1, Node& p2){
  if(p1.cost_ + p1.h_cost_ >= p2.cost_ + p2.h_cost_) return true;
  return false;
}

bool compare_points::operator()(const Node p1, const Node p2){
  if(p1.x_!=p2.x_ || p1.y_!=p2.y_) return true;
  return false;
}

bool compare_id::operator()(const Node p1, const Node p2){
  if(p1.id_>p2.id_) return true;
  return false;
}


std::vector<Node> GetMotion(int n){
  Node down(0,1,1,0,0,0);
  Node up(0,-1,1,0,0,0);
  Node left(-1,0,1,0,0,0);
  Node right(1,0,1,0,0,0);
  Node dr(1,1,1.4,0,0,0);
  Node dl(1,-1,1.4,0,0,0);
  Node ur(-1,1,1.4,0,0,0);
  Node ul(-1,-1,1.4,0,0,0);
  std::vector<Node> v;
  v.push_back(down);
  v.push_back(up);
  v.push_back(left);
  v.push_back(right);
  v.push_back(dr);
  v.push_back(dl);
  v.push_back(ur);
  v.push_back(ul);
  return v;
}

void MakeGrid(void *grid, int n){
  //NOTE: Using a void pointer isnt the best option
  int (*p_grid)[n][n] = (int (*)[n][n]) grid;

  std::random_device rd; // obtain a random number from hardware
  std::mt19937 eng(rd()); // seed the generator
  std::uniform_int_distribution<int> distr(0,n); // define the range

  for(int i=0;i<n;i++){
    for(int j=0;j<n;j++){
      (*p_grid)[i][j] = distr(eng)/((n-1)); // probability of obstacle is 1/n
      //(*p_grid)[i][j] = 0; // For no obstacles
    }
  }
}

void PrintGrid(void *grid, int n){
  //NOTE: Using a void pointer isnt the best option

  int (*p_grid)[n][n] = (int (*)[n][n]) grid;
  for(int j=0;j<n;j++){
    std::cout << "---";
  }
  std::cout << std::endl;
  for(int i=0;i<n;i++){
    for(int j=0;j<n;j++){
      if((*p_grid)[i][j]==3) std::cout << "\033[1;32m" << (*p_grid)[i][j] << "\033[1;0m" << " , ";
      else if((*p_grid)[i][j]==1) std::cout << "\033[1;31m" << (*p_grid)[i][j] << "\033[1;0m" << " , ";
      else if((*p_grid)[i][j]==2) std::cout << "\033[1;34m" << (*p_grid)[i][j] << "\033[1;0m" << " , ";
      else std::cout << (*p_grid)[i][j] << " , ";
    }
    std::cout << std::endl << std::endl;
  }
  for(int j=0;j<n;j++){
    std::cout <<  "---";
  }
  std::cout << std::endl;
}

void PrintPath(std::vector<Node> path_vector, Node start, Node goal, void *grid, int n){
  //NOTE: Using a void pointer isnt the best option
  int (*p_grid)[n][n] = (int (*)[n][n]) grid;
  if(path_vector[0].id_ == -1){
    std::cout << "No path exists" << std::endl;
    PrintGrid(*p_grid, n);
    return 0;
  }

  std::cout << "Path (goal to start):" << std::endl;
  int i = 0;
  for(int j = 0; j < path_vector.size(); j++){
    if(goal == path_vector[j]){
      i=j;
      break;
    }
  }
  path_vector[i].PrintStatus();
  (*p_grid)[path_vector[i].x_][path_vector[i].y_] = 3;
  while(path_vector[i].id_!=start.id_){
    if(path_vector[i].id_ == path_vector[i].pid_) break;
    for(int j = 0; j < path_vector.size(); j++){
      if(path_vector[i].pid_ == path_vector[j].id_){
        i=j;
        path_vector[j].PrintStatus();
        (*p_grid)[path_vector[j].x_][path_vector[j].y_] = 3;
      }
    }
  }
  (*p_grid)[start.x_][start.y_] = 3;
  std::cout << "Grid: " << std::endl;
  std::cout << "1. Points not considered ---> 0" << std::endl;
  std::cout << "2. Obstacles             ---> 1" << std::endl;
  std::cout << "3. Points considered     ---> 2" << std::endl;
  std::cout << "4. Points in final path  ---> 3" << std::endl;
  PrintGrid((*p_grid), n);
}
