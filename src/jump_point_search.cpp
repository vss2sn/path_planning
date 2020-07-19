#include "jump_point_search.hpp"

Node JumpPointSearch::jump(Node& new_point, Node& motion, int id){
  Node next_point  = new_point + motion;
  next_point.id_ = n*next_point.x_+next_point.y_;
  next_point.pid_ = id;
  next_point.h_cost_ = abs(next_point.x_ - goal_.x_) + abs(next_point.y_ - goal_.y_);
  if(next_point.x_ < 0 || next_point.y_ < 0 || next_point.x_ >= n || next_point.y_ >= n || grid[next_point.x_][next_point.y_]!=0){
    return new_point;
    // return Node(-1,-1,-1,-1,-1,-1);
  }
  if(pruned.find(next_point.id_)!=pruned.end()) pruned.insert(next_point.id_);
  if(next_point == goal_) return next_point;
  bool fn = false;
  fn = has_forced_neighbours(new_point, next_point, motion);
  if(fn){
    // std::cout << "Forced neighbours found"<<std::endl;
    return next_point;
  }
  else{
    Node jump_node = jump(next_point, motion, id);
    // Prevent over shoot
    if(jump_node.cost_ !=-1 &&  jump_node.cost_+ jump_node.h_cost_  <= next_point.cost_ + next_point.h_cost_) return jump_node;
    else return next_point;
  }
}

bool JumpPointSearch::has_forced_neighbours(Node& new_point, Node& next_point, Node& motion){
  int cn1x = new_point.x_ + motion.y_;
  int cn1y = new_point.y_ + motion.x_;

  int cn2x = new_point.x_ - motion.y_;
  int cn2y = new_point.y_ - motion.x_;

  int nn1x = next_point.x_ + motion.y_;
  int nn1y = next_point.y_ + motion.x_;

  int nn2x = next_point.x_ - motion.y_;
  int nn2y = next_point.y_ - motion.x_;

  bool a = !(cn1x < 0 || cn1y < 0 || cn1x >= n || cn1y >= n || grid[cn1x][cn1y]==1);
  bool b = !(nn1x < 0 || nn1y < 0 || nn1x >= n || nn1y >= n || grid[nn1x][nn1y]==1);
  if(a!=b) return true;

  a = !(cn2x < 0 || cn2y < 0 || cn2x >= n || cn2y >= n || grid[cn2x][cn2y]==1);
  b = !(nn2x < 0 || nn2y < 0 || nn2x >= n || nn2y >= n || grid[nn2x][nn2y]==1);
  if(a!=b) return true;

  return false;

}

#ifdef CUSTOM_DEBUG_HELPER_FUNCION
void JumpPointSearch::InsertionSort(std::vector<Node>& v){
   int nV = v.size();
   int i, j;
   Node key;
   for (i = 1; i < nV; i++) {
       key = v[i];
       j = i-1;
       while (j >= 0 && (v[j].cost_ + v[j].h_cost_ > key.cost_+key.h_cost_)){
           v[j+1] = v[j];
           j--;
       }
       v[j+1] = key;
   }
}
#endif

std::vector<Node> JumpPointSearch::jump_point_search(std::vector<std::vector<int>> &grid, Node start_in, Node goal_in){
  this->grid = grid;
  start_ = start_in;
  goal_ = goal_in;
  n = grid.size();
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

      Node jump_point = jump(new_point, *it, current.id_);
      if(jump_point.id_!=-1){
        open_list_.push(jump_point);
        if(jump_point.x_ == goal_.x_ && jump_point.y_ == goal_.y_){
          closed_list_.push_back(current);
          closed_list_.push_back(jump_point);
          grid[jump_point.x_][jump_point.y_] = 2;
          return closed_list_;
        }
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
int main(){
  int n = 11;

  std::vector<std::vector<int>> grid(n);
  std::vector<int> tmp(n);
  for (int i = 0; i < n; i++){
    grid[i] = tmp;
  }
  MakeGrid(grid);
  std::random_device rd; // obtain a random number from hardware
  std::mt19937 eng(rd()); // seed the generator
  std::uniform_int_distribution<int> distr(0,n-1); // define the range

  Node start(distr(eng),distr(eng),0,0,0,0);
  Node goal(distr(eng),distr(eng),0,0,0,0);

  start.id_ = start.x_ * n + start.y_;
  start.pid_ = start.x_ * n + start.y_;
  goal.id_ = goal.x_ * n + goal.y_;
  start.h_cost_ = abs(start.x_ - goal.x_) + abs(start.y_ - goal.y_);
  //Make sure start and goal are not obstacles and their ids are correctly assigned.
  grid[start.x_][start.y_] = 0;
  grid[goal.x_][goal.y_] = 0;
  PrintGrid(grid);

  JumpPointSearch new_jump_point_search;
  std::vector<Node> path_vector = new_jump_point_search.jump_point_search(grid, start, goal);

  PrintPath(path_vector, start, goal, grid);
  return 0;
}
#endif  // BUILD_INDIVIDUAL
