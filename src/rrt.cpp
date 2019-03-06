/*

RRT* grid based planning

*/

#include "rrt.h"

Node RRT::find_nearest_point(Node& new_node, int n){
  Node nearest_node(-1,-1,-1,-1,-1,-1);
  std::vector<Node>::iterator it_v;
  std::vector<Node>::iterator it_v_store;
  //use just distance not total cost
  double dist = (double)(n*n);
  double new_dist = (double)(n*n);
  for(it_v = point_list.begin(); it_v != point_list.end(); ++it_v){
    new_dist = (double)sqrt((double)(it_v->x-new_node.x)*(double)(it_v->x-new_node.x));
    if(new_dist > threshold) continue;
    if(check_obstacle(*it_v, new_node)) continue;
    if(it_v->id==new_node.id) continue;
    if(it_v->pid==new_node.id) continue;
    if(new_dist >= dist) continue;
    dist = new_dist;
    it_v_store = it_v;
  }
  if(dist!=n*n){
    nearest_node = *it_v_store;
    new_node.pid = nearest_node.id;
    new_node.cost = nearest_node.cost + dist;
  }
  return nearest_node;
}

bool RRT::check_obstacle(Node& n_1, Node& n_2){
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
          if(count > 1) return true;
          continue;
        }
        arr[i] = arr[i]/fabs(arr[i]);
        if ((arr[j]-arr[i]) != 0 ) return true;
      }
    }
  }
  return false;
}

Node RRT::generate_random_node(int n){
  std::random_device rd; // obtain a random number from hardware
  std::mt19937 eng(rd()); // seed the generator
  std::uniform_int_distribution<int> distr(0,n-1); // define the range
  int x = distr(eng);
  int y = distr(eng);
  Node new_node(x, y, 0, 0, n*x+y, 0);
  return new_node;
}


std::vector<Node> RRT::rrt(void *grid, int n, Node start_in, Node goal_in, int max_iter_x_factor = 500, double threshold_in = std::numeric_limits<double>::infinity()){
  start = start_in;
  goal = goal_in;

  threshold = threshold_in;
  int max_iter = max_iter_x_factor * n * n;
  int (*p_grid)[n][n] = (int (*)[n][n]) grid;
  create_obstacle_list(*p_grid, n);
  point_list.push_back(start);
  (*p_grid)[start.x][start.y]=2;
  int iter = 0;
  Node new_node = start;
  if(!check_obstacle(new_node, goal)){
    double new_dist = (double)sqrt((goal.x-new_node.x)*(goal.x-new_node.x) + (goal.y-new_node.y)*(goal.y-new_node.y));
    if(new_dist <= threshold){
      goal.pid=new_node.id;
      goal.cost = new_dist + new_node.cost;
      point_list.push_back(goal);
      return this->point_list;
    }
  }
  while(true){
    iter++;

    if(iter > max_iter){
      Node no_path_node(-1,-1,-1,-1,-1,-1);
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
      double new_dist = (double)sqrt((goal.x-new_node.x)*(goal.x-new_node.x) + (goal.y-new_node.y)*(goal.y-new_node.y));
      if(new_dist <= threshold){
        goal.pid=new_node.id;
        goal.cost = new_dist + new_node.cost;
        point_list.push_back(goal);
        return this->point_list;
      }
    }
    //break;
  }
}

void RRT::create_obstacle_list(void *grid, int n){
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

#ifdef BUILD_INDIVIDUAL
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
                     { 1 , 0 , 1 , 1 , 1, 0 },
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
  Node start(0,0,0,0,0,0);
  start.id = start.x * n + start.y;
  start.pid = start.x * n + start.y;
  Node goal(n-1,n-1,0,0,0,0);
  goal.id = goal.x * n + goal.y;

  grid[start.x][start.y] = 0;
  grid[goal.x][goal.y] = 0;
  RRT new_rrt;

  std::vector<Node> path_vector = new_rrt.rrt(grid, n, start, goal, 5000, 2);
  print_path(path_vector, start, goal, grid, n);

  return 0;
}
#endif
