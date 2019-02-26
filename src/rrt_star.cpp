/*

Dijstra grid based planning

*/

#include "main.h"

class Node{

public:

  int x, y, id, pid;
  double cost;

  Node(int x = 0, int y = 0, double cost = 0, int id = 0, int pid = 0){
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
      std::cout << (*p_grid)[i][j] << " , ";
    }
    std::cout << std::endl << std::endl;
  }
  for(int j=0;j<n;j++){
    std::cout <<  "---";
  }
  std::cout << std::endl;
}

class Edge{
public:
  Node child;
  Node parent;
  double dist = 0;

  Edge(Node child, Node parent){
    this->child = child;
    this->parent = parent;
    update_dist();
  }

  void update_dist(){
    dist = (double)sqrt((double)((double)(child.x-parent.x) * (double)(child.x-parent.x) +
                (double)(child.y-parent.y) * (double)(child.y-parent.y)));
  }

  void update_parent(Node parent){
    this->parent = parent;
    child.pid = parent.id;
    update_dist();
  }
  void reverse_edge(){
    Node temp = parent;
    parent = child;
    child = temp;
    child.pid = parent.id;
  }
};




class RRT_STAR{
public:

  std::vector<Node> point_list;
  std::vector<Node> obstacle_list;
  std::vector<Edge> edge_list;
  std::vector<Node> near_nodes;
  std::vector<double> near_nodes_dist;
  Node start, goal;
  double threshold = 1;
  int n;
  bool found_goal = false;

  std::vector<Edge>::iterator find_edge(Node child_in){
    //std::cout <<"Looking for"<< std::endl;
    //child_in.print_status();
    std::vector<Edge>::iterator it;
    for(it = edge_list.begin(); it!=edge_list.end(); ++it){
      if(it->child == child_in) break;
    }
    //std::cout <<"found"<< std::endl;
    //it->child.print_status();
    //std::cout <<"returning"<< std::endl;
    return it;
  }

  Node find_nearest_point(Node& new_node, int n){
    Node nearest_node(-1,-1,-1,-1,-1);
    std::vector<Node>::iterator it_v;
    std::vector<Node>::iterator it_v_store;
    //use total cost not just distance
    double dist = (double)(n*n);
    double new_dist = (double)(n*n);
    for(it_v = point_list.begin(); it_v != point_list.end(); ++it_v){
      if(new_node == goal) std::cout << "Node 1:" << std::endl;
      if(new_node == goal) new_node.print_status();
      if(new_node == goal) std::cout << "Node 2:" << std::endl;
      if(new_node == goal) it_v->print_status();
      new_dist = (double)sqrt(((double)(it_v->x-new_node.x)*(double)(it_v->x-new_node.x))
                  + ((double)(it_v->y-new_node.y)*(double)(it_v->y-new_node.y)));
      if(new_dist > threshold) continue;
      if(new_node == goal) std::cout << "new dist before update :  " <<new_dist << std::endl;
      //new_dist += get_cost(*it_v);
      new_dist += it_v->cost;
      if(new_node == goal) std::cout << "new dist after update :  " <<new_dist << std::endl;
      if(new_node == goal) std::cout << "dist    :    " << dist     << std::endl;
      if(new_node == goal) std::cout << "new_dist:    " << new_dist << std::endl;

      if(check_obstacle(*it_v, new_node)){
        if(new_node == goal) std::cout << "Obstacle" << std::endl;
        continue;
      }
      if(it_v->id==new_node.id) continue;
      near_nodes.push_back(*it_v);
      near_nodes_dist.push_back(new_dist);
      if(it_v->pid==new_node.id) continue;
      if(new_dist >= dist) continue;
      if(new_node == goal)  std::cout <<"Distance updated"<< std::endl;
      dist = new_dist;
      it_v_store = it_v;
    }
    if(dist!=n*n){
      nearest_node = *it_v_store;
      new_node.pid = nearest_node.id;
      new_node.cost = dist;
      edge_list.push_back(Edge(new_node, nearest_node));
    }
    return nearest_node;
  }

  bool check_obstacle(Node& n_1, Node& n_2){
    return false;
    if (n_2.y - n_1.y == 0){
      double c = n_2.y;
      for(auto it_v = obstacle_list.begin(); it_v!=obstacle_list.end(); ++it_v){
        if(!(((n_1.x>=it_v->x) && (it_v->x>= n_2.x)) || ((n_1.x<=it_v->x) && (it_v->x<= n_2.x)))) continue;
        if ((double)it_v->y == c) return true;
      }
    }
    else {
      double slope = (double)(n_2.x - n_1.x)/(double)(n_2.y - n_1.y);
      std::vector<Node>::iterator it_v;
      double c = (double)n_2.x - slope * (double)n_2.y;
      for(auto it_v = obstacle_list.begin(); it_v!=obstacle_list.end(); ++it_v){
        if(!(((n_1.y>=it_v->y) && (it_v->y>= n_2.y)) || ((n_1.y<=it_v->y) && (it_v->y<= n_2.y)))) continue;
        if(!(((n_1.x>=it_v->x) && (it_v->x>= n_2.x)) || ((n_1.x<=it_v->x) && (it_v->x<= n_2.x)))) continue;
        double arr[4];
        arr[0] = (double)it_v->x+0.5 - slope*((double)it_v->y+0.5) - c;
        arr[1] = (double)it_v->x+0.5 - slope*((double)it_v->y-0.5) - c;
        arr[2] = (double)it_v->x-0.5 - slope*((double)it_v->y+0.5) - c;
        arr[3] = (double)it_v->x-0.5 - slope*((double)it_v->y-0.5) - c;
        int count = 0;
        int j = 0;

        for (int i=0;i<4;i++){
          if(fabs(arr[i]) <= 0.000001){
            count +=1;
            if(j==0 && i==0)j=1;
            if(count > 1){
              // std::cout << "THis obs" << std::endl;
              // it_v->print_status();
             return true;
            }
            continue;
          }
          arr[i] = arr[i]/fabs(arr[i]);
          if ((arr[j]-arr[i]) != 0 ){
            // std::cout << "THis obs" << std::endl;
            // it_v->print_status();
           return true;
          }
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

  double get_cost(Node my_node){
    double cost = 0;
    std::vector<Edge>::iterator it_e;
    while(my_node!=start){
      if(my_node.x == 3 && my_node.y ==3) std::cout << my_node.id <<",";
      it_e = find_edge(my_node);
      cost+=it_e->dist;
      my_node = it_e->parent;
    }
     if(my_node.x == 3 && my_node.y ==3) std::cout << std::endl;
    return cost;
  }

  void rewire(Node new_node){
    // std::cout << "In rewire--------------------------------------" << std::endl;
    // std::cout << "--------------------------------------" << std::endl;
    // std::cout << "--------------------------------------" << std::endl;
    // std::cout << "--------------------------------------" << std::endl;
    // std::cout << "--------------------------------------" << std::endl;
    // std::cout << "--------------------------------------" << std::endl;
    // std::cout << "--------------------------------------" << std::endl;
    //
    // std::cout << "New node:" << std::endl;
    // new_node.print_status();
    double new_cost = get_cost(new_node);
    std::vector<Edge>::iterator it_e;
    std::vector<Node>::iterator it_v;
    for(int i=0;i<near_nodes.size(); i++){
      double current_cost = get_cost(near_nodes[i]);
      if (current_cost > near_nodes_dist[i] + new_cost){
         std::cout << "Node causing rewire:" << std::endl;
         new_node.print_status();
        it_e = find_edge(near_nodes[i]);
        std::cout << "Node under consideration for rewire:" << std::endl;
         it_e->child.print_status();
        // it_e->parent.print_status();
        it_e->update_parent(new_node);
        // it_e->parent.print_status();
        // std::cout << "Rewire triggered--------------------------------------" << std::endl;
        // std::cout << "--------------------------------------" << std::endl;
        // std::cout << "--------------------------------------" << std::endl;
        // std::cout << "--------------------------------------" << std::endl;
        // std::cout << "--------------------------------------" << std::endl;
        // std::cout << "--------------------------------------" << std::endl;
        // std::cout << "--------------------------------------" << std::endl;


        for(it_v = point_list.begin(); it_v!=point_list.end();++it_v){
          if(*it_v==near_nodes[i]) break;
        }
        it_v->pid = new_node.id;
        it_v->cost = near_nodes_dist[i] + new_cost;
      }
    }
    near_nodes.clear();
    near_nodes_dist.clear();
  }


  std::vector<Node> rrt_star(void *grid, int n, Node start_in, Node goal_in, int max_iter_x_factor = 500, double threshold_in = std::numeric_limits<double>::infinity()){
    start = start_in;
    goal = goal_in;
    threshold = threshold_in;
    int max_iter = max_iter_x_factor * n * n;
    int (*p_grid)[n][n] = (int (*)[n][n]) grid;
    point_list.push_back(start);
    (*p_grid)[start.x][start.y]=2;
    int iter = 0;
    Node new_node = start;
    if(check_goal_visible(new_node)) found_goal = true;//return point_list;

    while(true){
      iter++;
      std::cout << "ITER ------------------------->>>>>>>>>>>>>>>>> " <<iter<< std::endl;
      if(iter > max_iter){
        if(!found_goal){
          Node no_path_node(-1,-1,-1,-1,-1);
          point_list.clear();
          point_list.push_back(no_path_node);
        }
        print_cost((*p_grid),n);
        return point_list;
      }
      new_node = generate_random_node(n);
      if ((*p_grid)[new_node.x][new_node.y]==1){
        continue;
      }
      Node nearest_node = find_nearest_point(new_node, n);
      if(nearest_node.id == -1){
        continue;
      }
      (*p_grid)[new_node.x][new_node.y]=2;
      new_node.print_status();
      std::vector<Node>::iterator it_v;
      for( it_v = point_list.begin(); it_v!=point_list.end(); ++it_v){
        if(new_node.x == it_v->x && new_node.y == it_v->y){
          if(new_node.cost < it_v->cost){
            point_list.erase(it_v);
            point_list.push_back(new_node);
          }
          break;
        }
      }
      if(it_v==point_list.end()) point_list.push_back(new_node);

      // std::cout << "-------------------New node is--------------------" << std::endl;
      if(check_goal_visible(new_node)) found_goal = true;//return point_list;
      // new_node.print_status();
      rewire(new_node);
      print_grid((*p_grid), n);
      std::cout << "COST:"<< std::endl;
      print_cost((*p_grid), n);
      //break;
    }
  }

  bool check_goal_visible(Node new_node){
    if(!check_obstacle(new_node, goal)){
      double new_dist = (double)sqrt((double)(goal.x-new_node.x)*(double)(goal.x-new_node.x)
                  + (double)(goal.y-new_node.y)*(double)(goal.y-new_node.y));
      if(new_dist <=threshold){
        new_dist+=get_cost(new_node);
        goal.pid = new_node.id;
        goal.cost = new_dist;
        edge_list.push_back(Edge(goal,new_node));
        std::vector<Node>::iterator it_v;
        for( it_v = point_list.begin(); it_v!=point_list.end(); ++it_v){
          if(goal.x == it_v->x && goal.y == it_v->y){
            if(goal.cost < it_v->cost){
              point_list.erase(it_v);
              point_list.push_back(goal);
            }
            break;
          }
        }
        if(it_v==point_list.end()) point_list.push_back(goal);
        //point_list.push_back(goal);
        return true;
      }
    }
    return false;
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

  void print_cost(void *grid, int n){
    //NOTE: Using a void pointer isnt the best option
    //std::cout << "POINT LIST SIZE:" << point_list.size() <<std::endl;
    int (*p_grid)[n][n] = (int (*)[n][n]) grid;
    std::vector<Node>::iterator it_v;
    for(int i=0;i<n;i++){
      for(int j=0;j<n;j++){
        for(it_v = point_list.begin(); it_v != point_list.end(); ++it_v){
          if(i == it_v->x && j== it_v->y){
            std::cout << std::setw(10) <<it_v->cost << " , ";
            break;
          }
        }
        if(it_v == point_list.end())
        std::cout << std::setw(10) << "  , ";
      }
      std::cout << std::endl << std::endl;
    }
    for(it_v = point_list.begin(); it_v != point_list.end(); ++it_v){
      if( it_v->x == it_v->y){
          if((*p_grid)[it_v->x][it_v->y] != 3){
            std::cout << "OIIIIII!!!!!!!!!!!"<< std::endl;
            break;
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
    if(path_vector[i].id == path_vector[i].pid) break;
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
  int n = 8;
  int num_points = n*n;


/*
  int grid[n][n] = {
                     { 0 , 0 , 0 , 0 },
                     { 0 , 0 , 0 , 0 },
                     { 0 , 0 , 1 , 0 },
                     { 0 , 0 , 0 , 0 }
                   };
*/
  /*
  n = 6;
  int grid[n][n] = {
                     { 0 , 0 , 0 , 0 , 0, 0 },
                     { 0 , 1 , 0 , 0 , 0, 0 },
                     { 1 , 0 , 1 , 1 , 1, 0 },
                     { 1 , 0 , 1 , 0 , 1, 0 },
                     { 0 , 0 , 1 , 1 , 1, 1 },
                     { 0 , 0 , 0 , 0 , 0, 0 }
                   } ;
 */

  int grid[n][n];
  make_grid(grid, n);

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
  RRT_STAR new_rrt_star;
  new_rrt_star.create_obstacle_list(grid, n);

  std::vector<Node> path_vector = new_rrt_star.rrt_star(grid, n, start, goal, 20, 1.5);
  print_path(path_vector, start, goal, grid, n);

  return 0;
}
