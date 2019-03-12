#include "utils.h"

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
  // std::cout << "===================="<< std::endl;
  // p1.PrintStatus();
  // p2.PrintStatus();
  // std::cout << "===================="<< std::endl;
  // Can modify this to allow tie breaks based on heuristic cost if required
  if (p1.cost_ + p1.h_cost_ >= p2.cost_ + p2.h_cost_) return true;
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
  // NOTE: Do not use the diagonals for A* as the heuristic used
  // does not cover that motion
  return v;
}

// Create the random grid
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

// Print out the grid
void PrintGrid(void *grid, int n){
  std::cout << "Calling print grid"<< std::endl;

  //NOTE: Using a void pointer isnt the best option
  std::cout << "Grid: " << std::endl;
  std::cout << "1. Points not considered ---> 0" << std::endl;
  std::cout << "2. Obstacles             ---> 1" << std::endl;
  std::cout << "3. Points considered     ---> 2" << std::endl;
  std::cout << "4. Points in final path  ---> 3" << std::endl;

  int (*p_grid)[n][n] = (int (*)[n][n]) grid;
  for(int j=0;j<n;j++){
    std::cout << "---";
  }
  std::cout << std::endl;
  for(int i=0;i<n;i++){
    for(int j=0;j<n;j++){
      if((*p_grid)[i][j]==3) std::cout << GREEN << (*p_grid)[i][j] << RESET << " , ";
      else if((*p_grid)[i][j]==1) std::cout << RED << (*p_grid)[i][j] << RESET << " , ";
      else if((*p_grid)[i][j]==2) std::cout << BLUE << (*p_grid)[i][j] << RESET << " , ";
      else std::cout << (*p_grid)[i][j] << " , ";
    }
    std::cout << std::endl << std::endl;
  }
  for(int j=0;j<n;j++) std::cout <<  "---";
  std::cout << std::endl;
}

void PrintPath(std::vector<Node> path_vector, Node start, Node goal, void *grid, int n){
  //NOTE: Using a void pointer isn't the best option
  std::cout << "In print path"<< std::endl;

  int (*p_grid)[n][n] = (int (*)[n][n]) grid;
  if(path_vector[0].id_ == -1){
    std::cout << "No path exists" << std::endl;
    PrintGrid(*p_grid, n);
    return;
  }
  std::cout << "Path (goal to start):" << std::endl;
  int i = 0;
  for(i = 0; i < path_vector.size(); i++){
    if(goal == path_vector[i]) break;
  }
  // path_vector[i].PrintStatus();
  (*p_grid)[path_vector[i].x_][path_vector[i].y_] = 3;
  while(path_vector[i].id_!=start.id_){
    if(path_vector[i].id_ == path_vector[i].pid_) break;
    for(int j = 0; j < path_vector.size(); j++){
      if(path_vector[i].pid_ == path_vector[j].id_){
        i=j;
        // path_vector[j].PrintStatus();
        (*p_grid)[path_vector[j].x_][path_vector[j].y_] = 3;
      }
    }
  }
  (*p_grid)[start.x_][start.y_] = 3;
  //std::cout << "Calling print grid"<< std::endl;
  PrintGrid((*p_grid), n);
}

// Prints out the cost for reaching points on the grid in the grid shape
void PrintCost(void *grid, int n, std::vector<Node> point_list){
  //NOTE: Using a void pointer isnt the best option
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
