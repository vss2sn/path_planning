/**
* @file utils.cpp
* @author vss2sn
* @brief Contains common/commonly used funtions and classes
*/

#include "utils.hpp"

Node::Node(int x, int y, double cost, double h_cost, int id, int pid){
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
            << "Cost          : " << cost_   << std::endl
            << "Heuristic cost: " << h_cost_ << std::endl
            << "Id            : " << id_     << std::endl
            << "Pid           : " << pid_    << std::endl
            << "--------------"              << std::endl;
}

Node Node::operator+(Node p){
  Node tmp;
  tmp.x_ = this->x_ + p.x_;
  tmp.y_ = this->y_ + p.y_;
  tmp.cost_ = this->cost_ + p.cost_;
  return tmp;
}

Node Node::operator-(Node p){
  Node tmp;
  tmp.x_ = this->x_ - p.x_;
  tmp.y_ = this->y_ - p.y_;
  return tmp;
}

void Node::operator=(Node p){
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
  // Can modify this to allow tie breaks based on heuristic cost if required
  if (p1.cost_ + p1.h_cost_ >= p2.cost_ + p2.h_cost_) return true;
  return false;
}

// Possible motions for dijkstra, A*, and similar algorithms.
// Not using this for RRT & RRT* to allow random direction movements.
// TODO: Consider adding option for motion restriction in RRT and RRT* by
//       replacing new node with nearest node that satisfies motion constraints

std::vector<Node> GetMotion(){
  Node down(0,1,1,0,0,0);
  Node up(0,-1,1,0,0,0);
  Node left(-1,0,1,0,0,0);
  Node right(1,0,1,0,0,0);
  std::vector<Node> v;
  v.push_back(down);
  v.push_back(up);
  v.push_back(left);
  v.push_back(right);
  // NOTE: Add diagonal movements for A* and D* only after the heuristics in the
  // algorithms have been modified. Refer to README.md. The heuristics currently
  // implemented are based on Manhattan distance and dwill not account for
  //diagonal/ any other motions
  return v;
}

void MakeGrid(std::vector<std::vector<int>> &grid){
  int n = grid.size();
  std::random_device rd; // obtain a random number from hardware
  std::mt19937 eng(rd()); // seed the generator
  std::uniform_int_distribution<int> distr(0,n); // define the range

  for(int i=0;i<n;i++){
    for(int j=0;j<n;j++){
      grid[i][j] = distr(eng)/((n-1)); // probability of obstacle is 1/n
      // grid[i][j] = 0; // For no obstacles
    }
  }
}

void PrintGrid(std::vector<std::vector<int>> &grid){
  int n = grid.size();
  std::cout << "Grid: " << std::endl;
  std::cout << "1. Points not considered ---> 0" << std::endl;
  std::cout << "2. Obstacles             ---> 1" << std::endl;
  std::cout << "3. Points considered     ---> 2" << std::endl;
  std::cout << "4. Points in final path  ---> 3" << std::endl;
  for(int j=0;j<n;j++){
    std::cout << "---";
  }
  std::cout << std::endl;
  for(int i=0;i<n;i++){
    for(int j=0;j<n;j++){
      if(grid[i][j]==3) std::cout << GREEN << grid[i][j] << RESET << " , ";
      else if(grid[i][j]==1) std::cout << RED << grid[i][j] << RESET << " , ";
      else if(grid[i][j]==2) std::cout << BLUE << grid[i][j] << RESET << " , ";
      else std::cout << grid[i][j] << " , ";
    }
    std::cout << std::endl << std::endl;
  }
  for(int j=0;j<n;j++) std::cout <<  "---";
  std::cout << std::endl;
}

void PrintPath(std::vector<Node> path_vector, Node start, Node goal, std::vector<std::vector<int>> &grid){
  if(path_vector[0].id_ == -1){
    std::cout << "No path exists" << std::endl;
    PrintGrid(grid);
    return;
  }
  std::cout << "Path (goal to start):" << std::endl;
  int i = 0;
  for(i = 0; i < path_vector.size(); i++){
    if(goal == path_vector[i]) break;
  }
  path_vector[i].PrintStatus();
  grid[path_vector[i].x_][path_vector[i].y_] = 3;
  while(path_vector[i].id_!=start.id_){
    if(path_vector[i].id_ == path_vector[i].pid_) break;
    for(int j = 0; j < path_vector.size(); j++){
      if(path_vector[i].pid_ == path_vector[j].id_){
        i=j;
        path_vector[j].PrintStatus();
        grid[path_vector[j].x_][path_vector[j].y_] = 3;
      }
    }
  }
  grid[start.x_][start.y_] = 3;
  PrintGrid(grid);
}

void PrintCost(std::vector<std::vector<int>> &grid, std::vector<Node> point_list){
  int n = grid.size();
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

void PrintPathInOrder(std::vector<Node>& path_vector, Node start, Node goal, std::vector<std::vector<int>>& grid){
  if(path_vector[0].id_ == -1){
    std::cout << "Path not found" << std::endl;
    PrintGrid(grid);
    return;
  }
  std::cout << "Path (goal to start):" << std::endl;
  int i=0;
  while(path_vector[i]!=goal) i++;
  for(; i>=0; i=i-1){
    path_vector[i].PrintStatus();
    grid[path_vector[i].x_][path_vector[i].y_]=3;
  }
  PrintGrid(grid);
}
