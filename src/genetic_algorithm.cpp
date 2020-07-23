/**
* @file genetic_algorithm.cpp
* @author vss2sn
* @brief Contains the GeneticAlgorithm class
*/

#include <algorithm>
#include <climits>
#include <iomanip>  // TODO(vss): replace setw
#include <iostream>
#include <random>

#include "genetic_algorithm.hpp"

constexpr int random_range_max = 100;

GeneticAlgorithm::GeneticAlgorithm(const int generations, const int popsize, const float c, const bool shorten_chromosome){
  this->generations_ = generations;
  this->popsize_ = popsize;
  this->c_ = c;
  this->generation_ = 0;
  this->shorten_chromosome_ = shorten_chromosome;
  motions_ = GetMotion();
  f_val = INT_MAX;
}

std::vector<Node> GeneticAlgorithm::genetic_algorithm(std::vector<std::vector<int>>& grid, const Node& start, const Node& goal, int path_length){
  this->grid_ = grid;
  this->start_ = start;
  this->goal_ = goal;
  this->path_length_ = path_length;
  n_ = grid_.size();
  std::vector<Node> path;

  // Create initial chromosome
  InitialSetup(path);
  paths_.push_back(path);

  // Check first path to goal
  found_ = CheckPath(path);
  if(found_) {
    truepaths_.push_back(path);
  }

  // apply algorithm
  // Allow while loop to continue beyond path found to find optimum path
  // while(found_ == false && generation_ <=generations_){
  while(generation_ <=generations_){
    // std::cout << "Generation: "<< generation_ << std::endl;
    int paths_size = paths_.size();
    std::vector<int> f_vals(paths_size);
    for(int i=0;i < paths_size; ++i){
      f_vals[i] = CalculateFitness(paths_[i]);
      if(f_vals[i] < f_val){
        f_val = f_vals[i];
      }
    }
    std::vector<std::vector<Node>> new_paths_;
    for(int i=0;i < paths_size; ++i){
      if(f_vals[i] <= f_val * static_cast<int>(c_)) {
        if(shorten_chromosome_) {
          new_paths_.emplace_back(std::vector<Node>(paths_[i].begin(), paths_[i].begin()+path_length_)); //c provides a margin of error from best path
        } else {
          new_paths_.push_back(paths_[i]);
        }
      }
    }

    paths_ = new_paths_;
    while (static_cast<int>(paths_.size()) < popsize_) {
      CrossoverMutation();
    }
    // TODO(vss): Consider dynamically modifying path length to move towards optimality
    for(auto& path_seen : paths_){
      int tmp_length = INT_MAX;
      if(CheckPath(path_seen)){
        found_ = true;
        truepaths_.push_back(path_seen);
        if(shorten_chromosome_){
          auto tmp = start_;
          if(static_cast<int>(path_seen.size()) < tmp_length) {
            tmp_length = path_seen.size();
          }
          if(compareCoordinates(tmp, goal_)) {
            tmp_length = 0;
          }
          for(size_t i=0;i<path_seen.size();i++){
            tmp = tmp + path_seen[i];
            if(compareCoordinates(tmp, goal_)){
              tmp_length = i;
              break;
            }
          }
          path_length_ = tmp_length;
        }
      }
      // PrintChromosome(path_seen);
    }
    generation_++;
  }
  // if(!truepaths_.empty())std::cout << "True paths: " << std::endl;
  // for(int i=0;i<truepaths_.size();i++) PrintPathOfChromosome(truepaths_[i]);
  return ReturnLastPath();
}

std::vector<Node> GeneticAlgorithm::ReturnLastPath() const { // given the way a genetic algorithm decreases fitness values, the last path is likely ot be the best. Can reorder ased on actual fitness values if required.
  std::vector<Node> v;
  v.push_back(start_);
  Node current = start_;
  int truepaths__size = truepaths_.size();
  if(truepaths__size==0){
    v.clear();
    v.emplace_back(Node(-1,-1,-1,-1,-1,-1));
    return v;
  }
  for(const auto& node : truepaths_[truepaths__size-1]) {
    current = current + node;
    current.pid_ = current.id_;
    current.id_ = n_*current.x_ +current.y_;
    v.push_back(current);
  }
  // for(size_t i=0;i<truepaths_[truepaths__size-1].size(); i++){
  //   Node tmp = current + truepaths_[truepaths__size-1][i];
  //   tmp.id_ = n_*tmp.x_ + tmp.y_;
  //   tmp.pid_ = current.id_;
  //   current = tmp;
  //   v.push_back(tmp);
  // }

  if(v.size()==1){
    v.clear();
    v.emplace_back(Node(-1,-1,-1,-1,-1,-1));
  }
  return v;
}

#ifdef CUSTOM_DEBUG_HELPER_FUNCION
void GeneticAlgorithm::PrintChromosome(const std::vector<Node>& path) const{
  std::cout << "Chromosome: ";
  for (const auto& v : path){
    for(size_t i=0; i < motions_.size(); i++)
      if(compareCoordinates(v, motions_[i])) {
        std::cout << i << " ";
      }
  }
  std::cout << "Fitness value: " << CalculateFitness(path) << std::endl;
}

void GeneticAlgorithm::PrintPathOfChromosome(const std::vector<Node>& path) const {
  std::cout << "Path: ";
  Node tmp = start_;
  std::cout << "("<<tmp.x_ << "," << tmp.y_ << ")" << std::setw(3);
  for (auto v : path){
    tmp=tmp+v;
    std::cout << "("<<tmp.x_ << "," << tmp.y_ << ")" << std::setw(3);
  }
  std::cout << std::endl;
}
#endif  // CUSTOM_DEBUG_HELPER_FUNCION

void GeneticAlgorithm::InitialSetup(std::vector<Node>& path) const {
  int d_x = goal_.x_ - start_.x_;
  int d_y = goal_.y_ - start_.y_;
  Node dx(d_x >= 0 ? 1 : -1, 0);
  Node dy(0, d_y >= 0 ? 1 : -1);
  // if (d_x > 0) {
  //   dx = Node(1, 0);
  // } else {
  //   dx = Node(-1, 0);
  // }
  // if (d_y > 0) {
  //   dy = Node(0, 1);
  // } else {
  //   dy = Node(0, -1);
  // }
  path.resize(path_length_);
  auto it = path.begin();
  std::fill_n(it,
            std::abs(d_x),
            dx);
  std::advance(it, std::abs(d_x));
  std::fill_n(it,
            std::abs(d_y),
            dy);
  std::advance(it, std::abs(d_y));
  std::generate_n(it,
                  std::distance(it, path.end()),
                  [&](){ return motions_[rand()%4];});
}

int GeneticAlgorithm::CalculateFitness(const std::vector<Node>& path) const {
  int cost = 0;
  Node i = start_;
  for(const auto& p : path){
    i = i + p;
    if(i.x_ < 0 || i.x_ >= n_ || i.y_ < 0 || i.y_ >= n_) {
      return INT_MAX;
    }
    if(compareCoordinates(i, goal_)) {
      break;
    }
    if(grid_[i.x_][i.y_]==1) {
      cost += n_*abs(goal_.x_ - i.x_) + n_*abs(goal_.y_-i.y_);
    } else {
      cost+=abs(goal_.x_ - i.x_) + abs(goal_.y_-i.y_); // Can add a scaling factor here
    }
  }
  return cost;
}

void GeneticAlgorithm::CrossoverMutation(){
  int p1 = static_cast<int>(rand()%paths_.size());
  int p2 = static_cast<int>(rand()%paths_.size());
  std::vector<Node> child;
  int a;
  if(paths_[p1].size() > paths_[p2].size()) {
    a = paths_[p1].size();
  } else {
    a = paths_[p2].size();
  }
  if(a > path_length_) {
    a = path_length_;
  }
  Node current = start_;

  for(int i=0;i<a;i++){
    int random_int = rand()%random_range_max;
    if(random_int < random_range_max/4) {
      child.push_back(paths_[p1][i]);
    } else if(random_int<random_range_max/2) {
      child.push_back(paths_[p2][i]);
    } else {
      child.push_back(motions_[rand()%4]);
    }
    auto tmp = current;
    current = current + child.back();
    // Prevents the new chromosome from going beyond the grid
    // Added as a very large percentage (majority) of these are out of bounds, and a waste of resources
    // Chromosomes that contain paths travelling over obstacles are still allowed
    if(current.x_ < 0 || current.x_ >= n_ || current.y_ < 0 || current.y_ >= n_){
      child.pop_back();
      current = tmp;
      i--;
    }
  }
  paths_.push_back(child);
}

// NOTE: Consider storing the point where an obstacle is encountereed and forcig that gene/motion to randomly mutate for a quicker convergence to a solution while maintaining randomness
bool GeneticAlgorithm::CheckPath(const std::vector<Node>& path) const {
  Node current = start_;
  if (compareCoordinates(current, goal_)) {
    return true;
  }
  for(const auto& node : path){
    current = current + node;
    if(compareCoordinates(current, goal_)) {
      return true;
    }
    if(current.x_ < 0 || current.x_ >= n_ || current.y_ < 0 || current.y_ >= n_ || grid_[current.x_][current.y_]==1) {
      return false;
    }
  }
  return compareCoordinates(current, goal_);
}

#ifdef BUILD_INDIVIDUAL
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

  Node start(distr(eng),distr(eng),0,0,0,0);
  Node goal(distr(eng),distr(eng),0,0,0,0);
  // Node start(0,0,0,0,0,0);
  // Node goal(n-1,n-1,0,0,0,0);

  start.id_ = start.x_ * n + start.y_;
  start.pid_ = start.x_ * n + start.y_;
  goal.id_ = goal.x_ * n + goal.y_;
  start.h_cost_ = abs(start.x_ - goal.x_) + abs(start.y_ - goal.y_);
  //Make sure start and goal are not obstacles and their ids are correctly assigned.
  grid[start.x_][start.y_] = 0;
  grid[goal.x_][goal.y_] = 0;
  PrintGrid(grid);
  std::vector<Node> path_vector;
  GeneticAlgorithm new_genetic_algorithm;
  path_vector = new_genetic_algorithm.genetic_algorithm(grid, start, goal, 2*start.h_cost_);
  PrintPathInOrder(path_vector, start, goal, grid);
}
#endif  // BUILD_INDIVIDUAL
