/*

Dijstra grid based planning

*/

#include "main.h"

class Node{
//private:
public:

  int x, y, cost, id, pid;

  Node(int x = 0, int y = 0, int cost = 0, int id = 0, int pid = 0){
    this->x = x;
    this->y = y;
    this->cost = cost;
    this->id = id;
    this->pid = pid;
  }
  void print_status(){
    std::cout << "--------------" << std::endl
              << "Node:"          << std::endl
              << "x   : " << x    << std::endl
              << "y   : " << y    << std::endl
              << "cost: " << cost << std::endl
              << "id  : " << id   << std::endl
              << "pid : " << pid  << std::endl
              << "--------------" << std::endl;
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
    if(p1.cost>p2.cost) return true;
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
  std::uniform_int_distribution<int> distr(0,n); // define the range

  for(int i=0;i<n;i++){
    for(int j=0;j<n;j++){
      (*p_grid)[i][j] = distr(eng)/(n-1); // probability of obstacle is 1/n
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
  Node down(0,1,1,0,0);
  Node up(0,-1,1,0,0);
  Node left(-1,0,1,0,0);
  Node right(1,0,1,0,0);
  Node dr(1,1,5,0,0);
  Node dl(1,-1,5,0,0);
  Node ur(-1,1,5,0,0);
  Node ul(-1,-1,5,0,0);
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

std::vector<Node> dijkstra(void *grid, int n, Node start, Node goal){
  int (*p_grid)[n][n] = (int (*)[n][n]) grid;
  int cost_grid[n][n];
  for (int i = 0; i < n; i++){
    for (int j = 0; j < n; j++){
      cost_grid[i][j] = n*n;
    }
  }

  std::vector<Node> motion = get_motion(n);
  std::priority_queue<Node, std::vector<Node>, compare_cost> points_list;
  points_list.push(start);
  std::vector<Node> path_vector;
  path_vector.push_back(start);

  cost_grid[start.x][start.y] = 0;
  while(!points_list.empty()){
    Node current = points_list.top();
    current.id = current.x * n + current.y;
    points_list.pop();
    if(current == goal){
      return path_vector;
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
      if(new_point.x < 0 || new_point.y < 0
        || new_point.x >= n || new_point.y >= n) continue; // Check boundaries
      if((*p_grid)[new_point.x][new_point.y]==0 && cost_grid[new_point.x][new_point.y] > new_point.cost){//=1 && (*p_grid)[new_point.x][new_point.y]!=2){
        new_point.pid = current.id;
        new_point.id = new_point.x * n + new_point.y;
        points_list.push(new_point);
        cost_grid[new_point.x][new_point.y] = new_point.cost;
        std::vector<Node>::iterator it_v;
        it_v = find (path_vector.begin(), path_vector.end(), new_point);
        if (it_v != path_vector.end()) *it_v = new_point;
        else path_vector.push_back(new_point);
      }
    }
  }
  path_vector.clear();
  Node no_path_node(-1,-1,-1,-1,-1);
  path_vector.push_back(no_path_node);
  return path_vector;
}

print_path(std::vector<Node> path_vector, Node start, Node goal, void *grid, int n){
  //NOTE: Using a void pointer isnt the best option
  int (*p_grid)[n][n] = (int (*)[n][n]) grid;
  if(path_vector[0].id == -1){
    std::cout << "No path exists" << std::endl;
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
  while(path_vector[i].pid!=path_vector[i].id){
    for(int j = 0; j < path_vector.size(); j++){
      if(path_vector[i].pid == path_vector[j].id){
        i=j;
        path_vector[j].print_status();
        (*p_grid)[path_vector[j].x][path_vector[j].y] = 3;
      }
    }
  }
  std::cout << "Grid: " << std::endl;
  std::cout << "1. Points not considered ---> 0" << std::endl;
  std::cout << "2. Obstacles             ---> 1" << std::endl;
  std::cout << "3. Points considered     ---> 2" << std::endl;
  std::cout << "4. Points in final path  ---> 3" << std::endl;
  print_grid((*p_grid), n);
}

int main(){
  int n = 3;
  int num_points = n*n;


  n = 6;
  int grid[n][n] = {
                     { 0 , 0 , 0 , 0 , 0, 0 },
                     { 0 , 1 , 1 , 0 , 0, 0 },
                     { 1 , 1 , 0 , 0 , 1, 0 },
                     { 1 , 0 , 0 , 1 , 1, 1 },
                     { 0 , 1 , 1 , 1 , 1, 0 },
                     { 0 , 0 , 0 , 0 , 0, 0 }
                   } ;

  //int grid[n][n];
  //make_grid(grid, n);

  //NOTE:
  // x = row index, y = column index.

  std::cout << "Grid:" << std::endl;
  std::cout << "1. Points not considered ---> 0" << std::endl;
  std::cout << "2. Obstacles             ---> 1" << std::endl;
  print_grid(grid, n);

  //Make sure start and goal not obstacles and their ids are correctly assigned.
  Node start(0,1,0,0,0);
  start.id = start.x * n + start.y;
  start.pid = start.x * n + start.y;
  Node goal(n-1,n-1,0,0,0);
  goal.id = goal.x * n + goal.y;

  grid[start.x][start.y] = 0;
  grid[goal.x][goal.y] = 0;
  std::vector<Node> path_vector = dijkstra(grid, n, start, goal);

  print_path(path_vector, start, goal, grid, n);

  return 0;
}
