/*

Dijstra grid based planning

*/

#include "main.h"
#include <iomanip>
#include <queue>
#include <list>
#include <map>

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
              << "Node:" << std::endl
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
    // tmp.id = this->id + p.id;
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
  std::uniform_int_distribution<int> distr(0,4); // define the range

  for(int i=0;i<n;i++){
    for(int j=0;j<n;j++){
      (*p_grid)[i][j] = distr(eng)/(n-1);
      //(*p_grid)[i][j] = 0;
    }
  }
}

void print_grid(void *grid, int n){
  //NOTE: Using a void pointer isnt the best option

  int (*p_grid)[n][n] = (int (*)[n][n]) grid;
  for(int i=0;i<n;i++){
    for(int j=0;j<n;j++){
      std::cout << (*p_grid)[i][j] << " , ";
    }
    std::cout << std::endl;
  }
}

std::vector<Node> get_motion(int n){
  Node down(0,1,1,0,0);
  Node up(0,-1,1,0,0);
  Node left(-1,0,1,0,0);
  Node right(1,0,1,0,0);
  std::vector<Node> v;
  v.push_back(down);
  v.push_back(up);
  v.push_back(left);
  v.push_back(right);
  return v;
}

std::map<Node, Node, compare_id> dijkstra(void *grid, int n, Node start, Node goal){
  int (*p_grid)[n][n] = (int (*)[n][n]) grid;
  Node reached_start(0,0,0,0,0);
  std::map<Node, Node, compare_id> path;
  std::vector<Node> motion = get_motion(n);
  std::priority_queue<Node, std::vector<Node>, compare_cost> points_list;
  points_list.push(start);
  path[start] = reached_start;
  while(!points_list.empty()){
    Node current = points_list.top();
    std::cout<< "New current point:" << std::endl;
    current.print_status();
    current.id = current.x * n + current.y;
    points_list.pop();
    if(current == goal){
      std::cout << "Returning" << std::endl;
      //std::cout << "Path length before return is: " << path.size() << std::endl;
      return path;
    }
    if((*p_grid)[current.x][current.y]!=0){
      //std::cout << "Continue triggered" << std::endl;
      continue;
    }
    (*p_grid)[current.x][current.y] = 1;
    //std::cout << (*p_grid)[current.x][current.y] <<std::endl;
    int current_cost = current.cost;
    for(auto it = motion.begin(); it!=motion.end(); ++it){
      Node new_point;
      new_point = current + *it;
      if(new_point.x < 0 || new_point.y < 0 || new_point.x >= n || new_point.y >= n) continue;
      if((*p_grid)[new_point.x][new_point.y]!=1){
        new_point.pid = current.id;
        new_point.id = new_point.x * n + new_point.y;

        //std::cout<< "Point being added:" << std::endl;
        //new_point.print_status();
        //print_grid(*p_grid, n);

        points_list.push(new_point);
        //path.insert({new_point,current});
        path[new_point]=current;
/*
        std::cout << "End loop reached. Printing path size: " << path.size() << std::endl;

        std::map<Node, Node, compare_id>::iterator it;
        it = path.find(new_point);
        if(it==path.end()) std::cout << "Not found" << std::endl;
        else std::cout << "Found" << std::endl;
        for(it = path.begin(); it!=path.end(); ++it){
          std::cout << std::endl << "Printing using iterator" << std::endl;
          std::cout << "From point:" << std::endl;
          it->first.print_status();
          std::cout << "To point:" << std::endl;
          it->second.print_status();
          std::cout << std::endl;
        }*/
      }
    }
  }
  path.clear();
  Node no_path_node(-1,-1,-1,-1,-1);
  path[no_path_node] = no_path_node;
  return path;
}



int main(){
/*
  Node a(1,1,0,0);
  Node b(2,1,0,0);
  Node c;
  c=a+b;
  c.print_status();
  c=a;
  c.print_status();
*/
  int n = 6;
  int num_points = n*n;
//  point sptSet[num_points];
int grid[n][n] = {
                   { 0 , 0 , 0 , 0 , 1, 1 },
                   { 0 , 1 , 1 , 0 , 0, 0 },
                   { 1 , 0 , 0 , 1 , 0, 0 },
                   { 1 , 0 , 0 , 0 , 1, 0 },
                   { 1 , 0 , 1 , 0 , 0, 0 },
                   { 0 , 0 , 1 , 0 , 0, 0 }
                 } ;

  //make_grid(grid, n);
  print_grid(grid, n);
  Node start(0,0,0,0,0);
  Node goal(n-1,0,0,0,0);
  std::map<Node, Node, compare_id> path;
  path = dijkstra(grid, n, start, goal);
  if(path.begin()->first.id == -1){
    std::cout << "No path exists" << std::endl;
    return 0;
  }
  std::map<Node, Node, compare_id>::iterator it;
  std::map<Node, Node, compare_id>::iterator it2;
  //std::cout << "Path length after return is: " << path.size() << std::endl;
  std::cout << std::endl << "Path:" << std::endl;

  /*for(it = path.begin(); it!=path.end(); ++it){
    std::cout << "From point:" << std::endl;
    it->first.print_status();
    std::cout << "To point:" << std::endl;
    it->second.print_status();
  }*/
  it = path.find(goal);
  //it->second.print_status();
  int a = 0;
  /*
  while(true){
    a++;
    //std::cout << "In while " << std::endl;
    for(it2 = path.begin(); it2!=path.end(); ++it2){
      //std::cout << "In for " << std::endl;
      //if(it->second.x==it2->first.x && it->second.y==it2->first.y){
      if(it->second==it2->first){
        //std::cout << "In if " << std::endl;
        grid[it2->first.x][it2->first.y] = 2;
        it = it2;
        break;
      }
    }
    it->first.print_status();
    if(it->first == start) break;
    //std::cout << "Not in if " << std::endl;
    if(a>2*n){
      std::cout << "Ouch." << std::endl;
      break;
    }
    print_grid(grid, n);
  }*/


//KEEP
  for(it = path.begin(); it!=path.end(); it++){
      if(it->first.x == goal.x && it->first.y == goal.y){
        //std::cout << "First.x =  " << it->first.x << std::endl << "First.y = " <<it->first.y << std::endl;
        //std::cout << "Goal found" << std::endl;
        break;
    }
  }
  while(true){
    a++;
    grid[it->first.x][it->first.y] = 2;
    //std::cout << "First.x =  " << it->first.x << std::endl << "First.y = " <<it->first.y << std::endl;
    print_grid(grid, n);
    it = path.find(it->second);
    it->first.print_status();
    if(it->first == start) {
      grid[it->first.x][it->first.y] = 2;
      print_grid(grid, n);
      break;
    }

    //std::cout << "Not in if " << std::endl;
    if(a>5*n){
      std::cout << "Ouch." << std::endl;
      break;
    }

  }
/*
  for(it = path.begin(); it!=path.end(); ++it){
      if(it->first.x == goal.x && it->first.y == goal.y){
        std::cout << "Goal found" << std::endl;
      }
  }
  for(it = path.begin(); it!=path.end(); ++it){
    std::cout<< "First:" <<std::endl;
    it->first.print_status();
    std::cout<< "Second:" <<std::endl;
    it->second.print_status();
  }
*/

  return 0;
}
