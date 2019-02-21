/*

Dijstra grid based planning

*/

#include "main.h"

class Node{

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

class RRT{
public:

  std::vector<Node> point_list;
  std::vector<Node> obstacle_list;
  int n;

  Node find_nearest_point(Node& new_node, int n){
    Node nearest_node(-1,-1,-1,-1,-1);
    std::vector<Node>::iterator it_v;
    std::vector<Node>::iterator it_v_store;
    float dist = (float)(n*n);
    float new_dist = (float)(n*n);
    for(it_v = point_list.begin(); it_v != point_list.end(); ++it_v){
      new_dist = (float)sqrt((it_v->x-new_node.x)*(it_v->x-new_node.x) + (it_v->y-new_node.y)*(it_v->y-new_node.y));
      if(new_dist < dist){
        if(check_obstacle(*it_v, new_node)) continue;
        if(*it_v==new_node) continue;
        if(it_v->pid==new_node.id) continue;
        dist = new_dist;
        it_v_store = it_v;
      }
    }
    if(dist!=n*n){
      nearest_node = *it_v_store;
      new_node.pid = nearest_node.id;
    }
    return nearest_node;
  }

  bool check_obstacle(Node& n_1, Node& n_2){
    if (n_2.y - n_1.y == 0){
      float c = n_2.y;
      for(auto it_v = obstacle_list.begin(); it_v!=obstacle_list.end(); ++it_v){
        if ((float)it_v->y == c) return true;
      }
    }
    else {
      float slope = (float)(n_2.x - n_1.x)/(float)(n_2.y - n_1.y);
      std::vector<Node>::iterator it_v;
      float c = (float)n_2.x - slope * (float)n_2.y;
      for(auto it_v = obstacle_list.begin(); it_v!=obstacle_list.end(); ++it_v){
        if(!(((n_1.y>=it_v->y) && (it_v->y>= n_2.y)) || ((n_1.y<=it_v->y) && (it_v->y<= n_2.y)))) continue;
        float arr[4];
        arr[0] = (float)it_v->x+0.5 - slope*((float)it_v->y+0.5) - c;
        arr[1] = (float)it_v->x+0.5 - slope*((float)it_v->y-0.5) - c;
        arr[2] = (float)it_v->x-0.5 - slope*((float)it_v->y+0.5) - c;
        arr[3] = (float)it_v->x-0.5 - slope*((float)it_v->y-0.5) - c;
        int count = 0;
        int j = 0;
        for (int i=0;i<4;i++){
          if(fabs(arr[i]) <= 0.000001){
            count +=1;
            if(j==0 && i==0)j=1;
            if(count > 1) return true;
            continue;
          }
          arr[i] = arr[i]/fabs(arr[i]);
          if ((arr[j]-arr[i]) != 0 )return true;
        }


      }
    }
    return false;
  }

  Node generate_random_node(int n){
    std::random_device rd; // obtain a random number from hardware
    std::mt19937 eng(rd()); // seed the generator
    std::uniform_int_distribution<int> distr(0,n-1); // define the range
    int x = distr(eng);
    int y = distr(eng);
    Node new_node(x, y, 0, n*x+y, 0);
    return new_node;
  }


  std::vector<Node> rrt(void *grid, int n, Node start, Node goal, int max_iter_x_factor = 500){
    int max_iter = max_iter_x_factor * n * n;
    int (*p_grid)[n][n] = (int (*)[n][n]) grid;
    point_list.push_back(start);
    (*p_grid)[start.x][start.y]=2;
    int iter = 0;
    Node new_node = start;
    if(!check_obstacle(new_node, goal)){
      goal.pid=new_node.id;
      point_list.push_back(goal);
      return this->point_list;
    }

    while(true){
      iter++;

      if(iter > max_iter){
        Node no_path_node(-1,-1,-1,-1,-1);
        point_list.clear();
        point_list.push_back(no_path_node);
        return point_list;
      }
      new_node = generate_random_node(n);
      if ((*p_grid)[new_node.x][new_node.y]!=0){
        continue;
      }
      Node nearest_node = find_nearest_point(new_node, n);
      if(nearest_node.id == -1){
        continue;
      }
      (*p_grid)[new_node.x][new_node.y]=2;
      point_list.push_back(new_node);
      if(!check_obstacle(new_node, goal)){
        goal.pid=new_node.id;
        point_list.push_back(goal);
        return this->point_list;
      }
      //break;
    }
  }

  void create_obstacle_list(void *grid, int n){
    int (*p_grid)[n][n] = (int (*)[n][n]) grid;
    for(int i=0; i < n; i++){
      for(int j=0;j < n; j++){
        if((*p_grid)[i][j]==1){
          Node obs(i,j,0,i*n+j,0);
          obstacle_list.push_back(obs);
        }
      }
    }
  }

};

print_path(std::vector<Node> path_vector, Node start, Node goal, void *grid, int n){
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

  /*
  int grid[n][n] = {
                     { 0 , 1 , 1 },
                     { 1 , 0 , 1 },
                     { 1 , 1 , 0 }
                   };
  */

  n = 6;
  int grid[n][n] = {
                     { 0 , 0 , 0 , 0 , 0, 0 },
                     { 0 , 1 , 0 , 0 , 0, 0 },
                     { 1 , 1 , 1 , 1 , 1, 0 },
                     { 1 , 0 , 1 , 0 , 1, 0 },
                     { 0 , 0 , 1 , 1 , 1, 1 },
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
  Node start(0,0,0,0,0);
  start.id = start.x * n + start.y;
  start.pid = start.x * n + start.y;
  Node goal(n-1,n-1,0,0,0);
  goal.id = goal.x * n + goal.y;

  grid[start.x][start.y] = 0;
  grid[goal.x][goal.y] = 0;
  RRT new_rrt;
  new_rrt.create_obstacle_list(grid, n);

  std::vector<Node> path_vector = new_rrt.rrt(grid, n, start, goal, 5000);
  print_path(path_vector, start, goal, grid, n);

  return 0;
}
