/*

Dijstra grid based planning

*/

#include "main.h"
#include <iomanip>
#include <queue>
#include <list>
#include <map>

class Node{
public:

  int x, y, id, pid;
  double cost, h_cost;

  Node(int x = 0, int y = 0, double cost = 0, double h_cost = 0, int id = 0, int pid = 0){
    this->x = x;
    this->y = y;
    this->cost = cost;
    this->h_cost = h_cost;
    this->id = id;
    this->pid = pid;
  }
  void print_status(){
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
  Node operator+(Node p){
    Node tmp;
    tmp.x = this->x + p.x;
    tmp.y = this->y + p.y;
    tmp.cost = this->cost + p.cost;
    return tmp;
  }
  Node operator=(Node p){
    this->x = p.x;
    this->y = p.y;
    this->cost = p.cost;
    this->h_cost = p.h_cost;
    this->id = p.id;
    this->pid = p.pid;
  }
  bool operator==(Node p){
    if (this->x == p.x && this->y == p.y) return true;
    return false;
  }
  bool operator!=(Node p){
    if (this->x != p.x || this->y != p.y) return true;
    return false;
  }
};

struct compare_cost{
  bool operator()(Node& p1, Node& p2){
    if(p1.cost + p1.h_cost >= p2.cost + p2.h_cost) return true;
    return false;
  }
};

struct compare_points{
  bool operator()(const Node p1, const Node p2){
    if(p1.x!=p2.x || p1.y!=p2.y) return true;
    return false;
  }
};

struct compare_id{
  bool operator()(const Node p1, const Node p2){
    if(p1.id>p2.id) return true;
    return false;
  }
};

void make_grid(void *grid, int n){
  //NOTE: Using a void pointer isnt the best option
  int (*p_grid)[n][n] = (int (*)[n][n]) grid;

  std::random_device rd; // obtain a random number from hardware
  std::mt19937 eng(rd()); // seed the generator
  std::uniform_int_distribution<int> distr(0,4); // define the range

  for(int i=0;i<n;i++){
    for(int j=0;j<n;j++){
      (*p_grid)[i][j] = distr(eng)/(n-1);
      (*p_grid)[i][j] = 0; // For no obstacles
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
      std::cout << (*p_grid)[i][j] << " , ";
    }
    std::cout << std::endl << std::endl;
  }
  for(int j=0;j<n;j++){
    std::cout <<  "---";
  }
  std::cout << std::endl;
}

std::vector<Node> get_motion(int n){
  Node down(0,1,1,0,0,0);
  Node up(0,-1,1,0,0,0);
  Node left(-1,0,1,0,0,0);
  Node right(1,0,1,0,0,0);
  Node dr(1,1,5,0,0,0);
  Node dl(1,-1,5,0,0,0);
  Node ur(-1,1,5,0,0,0);
  Node ul(-1,-1,5,0,0,0);
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

std::map<Node, Node, compare_id> dijkstra(void *grid, int n, Node start, Node goal){
  int (*p_grid)[n][n] = (int (*)[n][n]) grid;
  double cost_grid[n][n];
  for (int i = 0; i < n; i++){
    for (int j = 0; j < n; j++){
      cost_grid[i][j] = n*n;
    }
  }

  std::map<Node, Node, compare_id> path;
  std::vector<Node> motion = get_motion(n);
  std::priority_queue<Node, std::vector<Node>, compare_cost> points_list;
  points_list.push(start);

  path[start] = start;
  cost_grid[start.x][start.y] = 0;
  while(!points_list.empty()){
    Node current = points_list.top();
    current.id = current.x * n + current.y;
    points_list.pop();
    if(current == goal){
      return path;
    }
    if((*p_grid)[current.x][current.y]!=0){
      continue; // Point already opened and
                // points around it added to points list
    }
    (*p_grid)[current.x][current.y] = 2; // Point opened
    int current_cost = current.cost;
    for(auto it = motion.begin(); it!=motion.end(); ++it){
      Node new_point;
      new_point = current + *it;
      new_point.h_cost = abs(new_point.x - goal.x) + abs(new_point.y - goal.y);
      if(new_point.x < 0 || new_point.y < 0
          || new_point.x >= n || new_point.y >= n) continue; // Check boundaries
      if(cost_grid[new_point.x][new_point.y] > new_point.cost + new_point.h_cost){//=1 && (*p_grid)[new_point.x][new_point.y]!=2){
        // TODO: check if this is required, does not seem to make a difference:
        // (*p_grid)[new_point.x][new_point.y]!=2
        new_point.pid = current.id;
        new_point.id = new_point.x * n + new_point.y;
        points_list.push(new_point);
        path[new_point] = current;
        cost_grid[new_point.x][new_point.y] = new_point.cost + new_point.h_cost;
      }
    }
  }
  path.clear();
  Node no_path_node(-1,-1,-1,-1,-1,-1);
  path[no_path_node] = no_path_node;
  return path;
}

int main(){
  int n = 3;
  int num_points = n*n;


  n = 6;
  int grid[n][n] = {
                     { 0 , 0 , 0 , 0 , 0, 0 },
                     { 0 , 1 , 0 , 0 , 0, 0 },
                     { 0 , 1 , 0 , 0 , 1, 0 },
                     { 0 , 1 , 0 , 0 , 1, 0 },
                     { 0 , 1 , 1 , 1 , 1, 0 },
                     { 0 , 0 , 0 , 0 , 0, 0 }
                   } ;

  //int grid[n][n];
  //make_grid(grid, n);
  std::cout << "Grid (Obstcacles set as 1):" << std::endl;
  print_grid(grid, n);

  //Make sure start and goal not obstacles and their ids are correctly assigned.
  Node start(0,1,0,0,0,0);
  start.id = start.x * n + start.y;
  start.pid = start.x * n + start.y;
  Node goal(n-1,n-1,0,0,0,0);
  goal.id = goal.x * n + goal.y;
  start.h_cost = abs(goal.x - start.x) + abs(goal.y - start.y);

  grid[start.x][start.y] = 0;
  grid[goal.x][goal.y] = 0;
  std::map<Node, Node, compare_id> path;
  path = dijkstra(grid, n, start, goal);
  if(path.begin()->first.id == -1){
    std::cout << "No path exists" << std::endl;
    return 0;
  }

  std::map<Node, Node, compare_id>::iterator it;
  std::map<Node, Node, compare_id>::iterator it2;
  for(it = path.begin(); it!=path.end(); it++){
      if(it->first.x == goal.x && it->first.y == goal.y) break;
  }

  while(true){
    grid[it->first.x][it->first.y] = 3;
    grid[it->second.x][it->second.y] = 3;

    for(it2 = path.begin(); it2!=path.end(); it2++){
      // std::cout << "ID pairs: " << it->first.pid << " , " << it2->first.id << std::endl;
      // The second arguement of the map is constantly updated,
      // while the first's id and pid are not, which is why this method is used.
      // The first one cannot be updated as it is a read only object
      // The mapping is the equivalent of creating a sparse matrix
        if(it->second.pid == it2->first.id){
          it = it2;
          break;
        }
    }
    if(it->first == start) {
      grid[it->first.x][it->first.y] = 3;
      std::cout << "Grid: " << std::endl;
       std::cout << "1. Points not considered ---> 0" << std::endl;
       std::cout << "2. Obstacles             ---> 1" << std::endl;
       std::cout << "3. Points considered     ---> 2" << std::endl;
       std::cout << "4. Points in final path  ---> 3" << std::endl;
      print_grid(grid, n);
      break;
    }
  }
  return 0;
}
