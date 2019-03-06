#include "utils.h"

Node::Node(int x = 0, int y = 0, double cost = 0, double h_cost = 0, int id = 0, int pid = 0){
  this->x = x;
  this->y = y;
  this->cost = cost;
  this->h_cost = h_cost;
  this->id = id;
  this->pid = pid;
}
void Node::print_status(){
  std::cout << "--------------"             << std::endl
            << "Node          :"            << std::endl
            << "x             : " << x      << std::endl
            << "y             : " << y      << std::endl
            << "cost          : " << cost   << std::endl
            << "Heuristic cost: " << h_cost << std::endl
            << "id            : " << id     << std::endl
            << "pid           : " << pid    << std::endl
            << "--------------"             << std::endl;
}
Node Node::operator+(Node p){
  Node tmp;
  tmp.x = this->x + p.x;
  tmp.y = this->y + p.y;
  tmp.cost = this->cost + p.cost;
  return tmp;
}
Node Node::operator=(Node p){
  this->x = p.x;
  this->y = p.y;
  this->cost = p.cost;
  this->h_cost = p.h_cost;
  this->id = p.id;
  this->pid = p.pid;
}
bool Node::operator==(Node p){
  if (this->x == p.x && this->y == p.y) return true;
  return false;
}
bool Node::operator!=(Node p){
  if (this->x != p.x || this->y != p.y) return true;
  return false;
}


bool compare_cost::operator()(Node& p1, Node& p2){
  if(p1.cost + p1.h_cost >= p2.cost + p2.h_cost) return true;
  return false;
}

bool compare_points::operator()(const Node p1, const Node p2){
  if(p1.x!=p2.x || p1.y!=p2.y) return true;
  return false;
}

bool compare_id::operator()(const Node p1, const Node p2){
  if(p1.id>p2.id) return true;
  return false;
}


std::vector<Node> get_motion(int n){
  Node down(0,1,1,0,0,0);
  Node up(0,-1,1,0,0,0);
  Node left(-1,0,1,0,0,0);
  Node right(1,0,1,0,0,0);
  Node dr(1,1,1,0,0,0);
  Node dl(1,-1,1,0,0,0);
  Node ur(-1,1,1,0,0,0);
  Node ul(-1,-1,1,0,0,0);
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

void make_grid(void *grid, int n){
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

void print_grid(void *grid, int n){
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

void print_path(std::vector<Node> path_vector, Node start, Node goal, void *grid, int n){
  //NOTE: Using a void pointer isnt the best option
  int (*p_grid)[n][n] = (int (*)[n][n]) grid;
  if(path_vector[0].id == -1){
    std::cout << "No path exists" << std::endl;
    print_grid(*p_grid, n);
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
  path_vector[i].print_status();
  (*p_grid)[path_vector[i].x][path_vector[i].y] = 3;
  while(path_vector[i].id!=start.id){
    if(path_vector[i].id == path_vector[i].pid) break;
    for(int j = 0; j < path_vector.size(); j++){
      if(path_vector[i].pid == path_vector[j].id){
        i=j;
        path_vector[j].print_status();
        (*p_grid)[path_vector[j].x][path_vector[j].y] = 3;
      }
    }
  }
  (*p_grid)[start.x][start.y] = 3;
  std::cout << "Grid: " << std::endl;
  std::cout << "1. Points not considered ---> 0" << std::endl;
  std::cout << "2. Obstacles             ---> 1" << std::endl;
  std::cout << "3. Points considered     ---> 2" << std::endl;
  std::cout << "4. Points in final path  ---> 3" << std::endl;
  print_grid((*p_grid), n);
}
