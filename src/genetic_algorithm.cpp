#include "genetic_algorithm.hpp"
using namespace std;
class GeneticAlgorithm{
  vector<vector<int>> grid_;
  vector<Node> motions_;
  Node start_, goal_;
  int path_length_, n, f_val, generation_, generations_, popsize_;
  float c = 1.2;
  vector<vector<Node>> paths, truepaths;
  bool found;

public:
  GeneticAlgorithm(int generations = 10000, int popsize = 100){
    this->generations_ = generations;
    this->popsize_ = popsize;
    this->generation_ = 0;
    motions_ = GetMotion();
    f_val = INT_MAX;
  }

  vector<Node> genetic_algorithm(vector<vector<int>>& grid, Node start, Node goal, int path_length = 30){
    this->grid_ = grid;
    this->start_ = start;
    this->goal_ = goal;
    this->path_length_ = path_length;

    n = grid_.size();
    vector<Node> path;

    // Create initial chromosome
    InitialSetup(path);
    paths.push_back(path);

    // Check first path to goal
    found = check_path(path);
    if(found) truepaths.push_back(path);

    // apply algorithm
    while(found == false && generation_ <=generations_){
      // cout << "Generation: "<< generation_ << endl;
      int paths_size = paths.size();
      vector<int> f_vals(paths_size);
      for(int i=0;i < paths_size; ++i){
        f_vals[i] = CalculateFitness(paths[i]);
        if(f_vals[i] < f_val){
          f_val = f_vals[i];
        }
      }
      vector<vector<Node>> new_paths;
      for(int i=0;i < paths_size; ++i){
        if(f_vals[i] <= f_val * c) new_paths.push_back(paths[i]); //c provides a margin of error from best path
      }

      paths = new_paths;
      while (paths.size() < popsize_) CrossoverMutation();

      for(auto& path_seen : paths){
        if(check_path(path_seen)){
          found = true;
          truepaths.push_back(path_seen);
        }
        // PrintChromosome(path_seen);
      }
      generation_++;
    }
    if(!truepaths.empty())cout << "True paths: " << endl;
    for(int i=0;i<truepaths.size();i++) PrintPathOfChromosome(truepaths[i]);
    return ReturnLastPath();
  }

  vector<Node> ReturnLastPath(){ // given the way a genetic algorithm decreases fitness values, the last path is likely ot be the best. Can reorder ased on actual fitness values if required.
    vector<Node> v;
    v.push_back(start_);
    Node current = start_;
    int truepaths_size = truepaths.size();
    if(truepaths_size==0){
      v.clear();
      v.push_back(Node(-1,-1,-1,-1,-1,-1));
      return v;
    }
    for(int i=0;i<truepaths[truepaths_size-1].size(); i++){
      Node tmp = current + truepaths[truepaths_size-1][i];
      tmp.id_ = n*tmp.x_ + tmp.y_;
      tmp.pid_ = current.id_;
      current = tmp;
      v.push_back(tmp);
    }

    if(v.size()==1){
      v.clear();
      v.push_back(Node(-1,-1,-1,-1,-1,-1));
    }
    return v;
  }

  void PrintChromosome(vector<Node>& path){
    cout << "Chromosome: ";
    for (auto v : path){
      for(int i=0;i<motions_.size();i++)
        if(v==motions_[i]) cout << i << " ";
    }
    cout << endl;
  }

  void PrintPathOfChromosome(vector<Node>& path){
    cout << "Path: ";
    Node tmp = start_;
    cout << "("<<tmp.x_ << "," << tmp.y_ << ")" << setw(3);
    for (auto v : path){
      tmp=tmp+v;
      cout << "("<<tmp.x_ << "," << tmp.y_ << ")" << setw(3);
    }
    cout << endl;
  }

  void InitialSetup(vector<Node>& path){
    int d_x = goal_.x_ - start_.x_;
    int d_y = goal_.y_ - start_.y_;
    Node dx, dy;
    Node h = start_;
    if(d_x>0) dx = Node(1,0);
    else dx = Node(-1,0);
    if(d_y>0) dy = Node(0,1);
    else dy = Node(0,-1);
    while(h!=goal_){
      if(d_x > 0){
        d_x--;
        path.emplace_back(dx);
        h = h + dx;
      }
      if(d_y > 0){
        d_y--;
        path.emplace_back(dy);
        h = h + dy;
      }
    }
    for(int i=path.size();i < path_length_;i++) path.emplace_back(motions_[rand()%4]);
  }

  int CalculateFitness(vector<Node>& path){
    int cost = 0;
    Node i = start_;
    for(auto& tmp : path){
      i=i+tmp;
      if(i.x_ < 0 || i.x_ >= n || i.y_ < 0 || i.y_ >= n) return INT_MAX;
      else if(grid_[i.x_][i.y_]==1) cost += n*abs(goal_.x_ - i.x_) + n*abs(goal_.y_-i.y_);
      else if(i==goal_) break;
      else cost+=pow(abs(goal_.x_ - i.x_),2) + pow(abs(goal_.y_-i.y_),2);
    }
    // cout << "here"<<endl;
    return cost;
  }

  void CrossoverMutation(){
    // cout << "hello " << paths.size()<<endl;
    int p1 = rand()%paths.size();
    int p2 = rand()%paths.size();
    // cout << p1 << " "<<p2  << endl;
    vector<Node> child;
    int a;
    if(paths[p1].size() > paths[p2].size()) a = paths[p1].size();
    else  a = paths[p2].size();
    for(int i=0;i<a;i++){
      int random_int = rand()%100;
      if(random_int<33) child.push_back(paths[p1][i]);
      else if(random_int<66) child.push_back(paths[p2][i]);
      else child.push_back(motions_[rand()%4]);
    }
    paths.push_back(child);
  }

  bool check_path(vector<Node>& path){
    Node current = start_;
    if(current == goal_) return true;
    for(auto it = path.begin(); it!=path.end();it++){
      current = current + *it;
      if(current==goal_) return true;
      else if(current.x_ < 0 || current.x_ >= n || current.y_ < 0 || current.y_ >= n || grid_[current.x_][current.y_]==1) return false;
    }
    if(current==goal_) return true;
    return false;
  }
};

int main(){
  int n = 10;

  std::vector<std::vector<int>> grid(n);
  std::vector<int> tmp(n);
  for (int i = 0; i < n; i++){
    grid[i] = tmp;
  }
  MakeGrid(grid);
  std::random_device rd; // obtain a random number from hardware
  std::mt19937 eng(rd()); // seed the generator
  std::uniform_int_distribution<int> distr(0,n-1); // define the range

  // Node start(distr(eng),distr(eng),0,0,0,0);
  // Node goal(distr(eng),distr(eng),0,0,0,0);
  Node start(0,0,0,0,0,0);
  Node goal(n-1,n-1,0,0,0,0);

  start.id_ = start.x_ * n + start.y_;
  start.pid_ = start.x_ * n + start.y_;
  goal.id_ = goal.x_ * n + goal.y_;
  start.h_cost_ = abs(start.x_ - goal.x_) + abs(start.y_ - goal.y_);
  //Make sure start and goal are not obstacles and their ids are correctly assigned.
  grid[start.x_][start.y_] = 0;
  grid[goal.x_][goal.y_] = 0;
  int max_iter = n;
  PrintGrid(grid);
  std::vector<Node> path_vector;
  GeneticAlgorithm new_genetic_algorithm;
  path_vector = new_genetic_algorithm.genetic_algorithm(grid, start, goal, 30);
  PrintPathInOrder(path_vector, start, goal, grid);
}
