/**
* @file a_star.cpp
* @author vss2sn
* @brief Contains the AStar class
*/

#include "a_star.h"
using namespace std::chrono;

/**
* @brief Using insertion sort to sort the vector list that maintains the priority queue. Good for a mostly sorted queue. Sort called afterevery insertion to maintain queue. Not using standard queue as iterating over is not allowed.
* @param v Vector to be sorted
* @return void
*/
void InsertionSort(std::vector<Node>& v){
   int n = v.size();
   int i, j;
   Node key;
   for (i = 1; i < n; i++) {
       key = v[i];
       j = i-1;
       while (j >= 0 && (v[j].cost_ + v[j].h_cost_ > key.cost_+key.h_cost_)){
           v[j+1] = v[j];
           j--;
       }
       v[j+1] = key;
   }
}

/**
* @brief Main algorithm of A*
* @param grid Main grid
* @param n number of rows/columns
* @param start_in start node
* @param goal_in goal node
* @return vector of path
*/
std::vector<Node> AStar::a_star(std::vector<std::vector<int>> &grid, int n, Node start_in, Node goal_in){
  start_ = start_in;
  goal_ = goal_in;

  // Get possible motions
  std::vector<Node> motion = GetMotion();
  open_list_.push(start_);

  // Main loop
  Node temp;
  while(!open_list_.empty()){
    Node current = open_list_.top();
    open_list_.pop();
    current.id_ = current.x_ * n + current.y_;
    if(current.x_ == goal_.x_ && current.y_ == goal_.y_){
      closed_list_.push_back(current);
      grid[current.x_][current.y_] = 2;
      return closed_list_;
    }
    grid[current.x_][current.y_] = 2; // Point opened
    int current_cost = current.cost_;
    for(auto it = motion.begin(); it!=motion.end(); ++it){
      Node new_point;
      new_point = current + *it;
      new_point.id_ = n*new_point.x_+new_point.y_;
      new_point.pid_ = current.id_;
      new_point.h_cost_ = abs(new_point.x_ - goal_.x_) + abs(new_point.y_ - goal_.y_);
      if(new_point == goal_){
        open_list_.push(new_point);
        break;
      }
      if(new_point.x_ < 0 || new_point.y_ < 0 || new_point.x_ >= n || new_point.y_ >= n) continue; // Check boundaries
      if(grid[new_point.x_][new_point.y_]!=0){
        continue; //obstacle or visited
      }
      open_list_.push(new_point);
    }
    closed_list_.push_back(current);
  }
  closed_list_.clear();
  Node no_path_node(-1,-1,-1,-1,-1,-1);
  closed_list_.push_back(no_path_node);
  return closed_list_;
}

#ifdef BUILD_INDIVIDUAL
/**
* @brief Script main function. Generates start and end nodes as well as grid, then creates the algorithm object and calls the main algorithm function.
* @return 0
*/
int main(){
  int n = 11;
  int num_points = n*n;

  std::vector<std::vector<int>> grid(n);
  std::vector<int> tmp(n);
  for (int i = 0; i < n; i++){
    grid[i] = tmp;
  }
  MakeGrid(grid, n);
  std::random_device rd; // obtain a random number from hardware
  std::mt19937 eng(rd()); // seed the generator
  std::uniform_int_distribution<int> distr(0,n-1); // define the range

  // Node start(distr(eng),distr(eng),0,0,0,0);
  // Node goal(distr(eng),distr(eng),0,0,0,0);
  Node start(0,0,0,0,0,0);
  Node goal(n-1,n-1,0,0,0,0);
  start.PrintStatus();
  goal.PrintStatus();
  start.id_ = start.x_ * n + start.y_;
  start.pid_ = start.x_ * n + start.y_;
  goal.id_ = goal.x_ * n + goal.y_;
  start.h_cost_ = abs(start.x_ - goal.x_) + abs(start.y_ - goal.y_);
  //Make sure start and goal are not obstacles and their ids are correctly assigned.
  grid[start.x_][start.y_] = 0;
  grid[goal.x_][goal.y_] = 0;
  PrintGrid(grid, n);

  auto start_time = high_resolution_clock::now();
  AStar new_a_star;
  std::vector<Node> path_vector = new_a_star.a_star(grid, n, start, goal);
  auto stop_time = high_resolution_clock::now();
  auto duration = duration_cast<microseconds>(stop_time - start_time);
  std::cout << duration.count() << std::endl;

  PrintPath(path_vector, start, goal, grid, n);
  return 0;
}
#endif
