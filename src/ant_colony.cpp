/**
* @file ant_colony.cpp
* @author vss2sn
* @brief Contains the Ant Colony class
*/

#include "ant_colony.hpp"

template <typename T>
void print(T t){
  std::cout << t << std::endl;
}

class Ant{
public:
  bool found_goal = false;
  int steps = 0;
  std::vector<Node> path;
  Node previous_node;
  Node current_node;
  Ant(Node start = Node(0,0)){
    this->current_node = start;
    this->previous_node = Node(-1,-1);
  }
};

class AntColony{
  std::vector<std::vector<int>> grid;
  std::vector<std::vector<double>> pheromone_grid;
  int n_ants, iterations, max_steps, grid_size;
  double alpha, beta, evap_rate;
  Node start, goal;
  std::vector<Ant> ants;
public:
  AntColony(std::vector<std::vector<int>>& grid, Node start, Node goal, int n_ants = 10, double alpha = 0.5, double beta = 0.5, double evap_rate = 0.5, int iterations = 10){
    this->grid = grid;
    this->n_ants = n_ants;
    this->alpha = alpha;
    this->beta = beta;
    this->evap_rate = evap_rate;
    this->iterations = iterations;
    this->start = start;
    this->goal = goal;
    grid_size = grid.size();
    pheromone_grid = std::vector<std::vector<double>>(grid_size, std::vector<double> (grid_size, 0));
    max_steps = pow(grid_size,2);
    ants = std::vector<Ant>(n_ants);
  }

  std::vector<Node> ant_colony(){
    std::random_device device;
    std::mt19937 engine(device());
    // print(1);
    for(int i=0;i<iterations;i++){
      int n_success = 0;
      auto motions = GetMotion();
      // print(2);
      for(int j=0;j<n_ants;j++){
        Ant ant(start);
        // print(3);
        // print(ant.current_node!=goal);
        // print(ant.steps < max_steps);
        while(ant.current_node!=goal && ant.steps < max_steps){
          // sleep(1);
          // print("Steps: ");
          // print(ant.steps);
          // print(4);
          ant.path.push_back(ant.current_node);
          // print("Current Node:");
          // ant.current_node.PrintStatus();
          std::vector<Node> possible_positions;
          std::vector<double> possible_probabilities;
          float prob_sum = 0;
          int n_obs = 0;
          for(auto& m : motions){
            // print(5);
            Node possible_position = ant.current_node + m;
            if(possible_position.x_ >=0 && possible_position.x_ < grid_size && possible_position.y_ >=0 && possible_position.y_ < grid_size
            && possible_position!=ant.previous_node){
              possible_positions.push_back(possible_position);
              std::cout << "First term: " << pow(pheromone_grid[possible_position.x_][possible_position.y_], alpha) << std::endl;
              std::cout << "Second term: " << pow(1.0/pow(pow((possible_position.x_ - goal.x_),2) +pow((possible_position.y_ - goal.y_),2)/grid_size,0.5), beta) << std::endl;
              double new_prob = pow(pheromone_grid[possible_position.x_][possible_position.y_], alpha) *
                pow(1.0/pow(pow((possible_position.x_ - goal.x_),2) +pow((possible_position.y_ - goal.y_),2),0.5), beta);
              possible_probabilities.push_back(new_prob);
              prob_sum += pheromone_grid[possible_position.x_][possible_position.y_];
              if(grid[possible_position.x_][possible_position.y_]==1) n_obs+=1;
            }
          }
          if(n_obs==possible_positions.size()){
            // Ant in a cul-de-sac
            std::cout << "Breaking" << std::endl;
            break;
          }
          else if(prob_sum == 0){
            double new_prob = 1.0/(possible_positions.size()-n_obs);
            for(int i=0;i<possible_positions.size();i++){
              if(grid[possible_positions[i].x_][possible_positions[i].y_]==0) possible_probabilities[i]=new_prob;
              else possible_probabilities[i]=0;
            }
          }
          std::discrete_distribution<> dist(possible_probabilities.begin(), possible_probabilities.end());
          ant.previous_node = ant.current_node;
          ant.current_node = possible_positions[dist(engine)];
          ant.steps++;
        }
        if(ant.current_node==goal){
          ant.path.push_back(ant.current_node);
          ant.found_goal = true;
          n_success++;
        }
        ants[j] = ant;
      }//for every ant loop ends here
      for(auto row : pheromone_grid){
        for(auto col : row){
          if(col!=0) col = (1-evap_rate)*col;
        }
      }
      double avg_len = 0;
      int count = 0;
      for(Ant& ant : ants){
        if(ant.found_goal){
          avg_len += ant.path.size();
          count+=1;
          double c = 1.0/(ant.path.size()-1)/ants.size();
          std::cout << "c: " << c << std::endl;
          for(auto& j:ant.path){
            pheromone_grid[j.x_][j.y_]+= c;
          }
        }
      }
      for(auto& row : pheromone_grid){
        for(auto& col : row){
          std::cout << col << " ";
        }
        std::cout << std::endl;
      }
      if (count>0)std::cout << "Avg path length = " << (avg_len-1)/count << std::endl;
      else std::cout << "Failed to find path" << std::endl;
      // sleep(3);
    }//for every iteration loop ends here
    return std::vector<Node>();
  }

};

#ifdef BUILD_INDIVIDUAL
/**
* @brief Script main function. Generates start and end nodes as well as grid, then creates the algorithm object and calls the main algorithm function.
* @return 0
*/
int main(){
  int n = 8;

  // std::vector<std::vector<int>> grid(n);
  // std::vector<int> tmp(n);
  // for (int i = 0; i < n; i++){
  //   grid[i] = tmp;
  // }
  // MakeGrid(grid);
  // std::random_device rd; // obtain a random number from hardware
  // std::mt19937 eng(rd()); // seed the generator
  // std::uniform_int_distribution<int> distr(0,n-1); // define the range
  //
  // Node start(distr(eng),distr(eng),0,0,0,0);
  // Node goal(distr(eng),distr(eng),0,0,0,0);
  std::vector<std::vector<int>> grid{
    { 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 },
    { 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 },
    { 0 , 1 , 0 , 0 , 0 , 0 , 1 , 0 },
    { 0 , 1 , 0 , 0 , 0 , 0 , 1 , 0 },
    { 0 , 1 , 0 , 0 , 0 , 0 , 1 , 0 },
    { 0 , 1 , 0 , 0 , 0 , 0 , 1 , 0 },
    { 0 , 1 , 1 , 1 , 1 , 1 , 1 , 0 },
    { 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 }
                   } ;
  Node start(0,0,0,0,0,0);
  Node goal(n-1,n-1,0,0,0,0);

  start.id_ = start.x_ * n + start.y_;
  start.pid_ = start.x_ * n + start.y_;
  goal.id_ = goal.x_ * n + goal.y_;
  start.h_cost_ = abs(start.x_ - goal.x_) + abs(start.y_ - goal.y_);
  //Make sure start and goal are not obstacles and their ids are correctly assigned.
  grid[start.x_][start.y_] = 0;
  grid[goal.x_][goal.y_] = 0;
  PrintGrid(grid);

  AntColony new_ant_colony(grid, start, goal, 10, 0.5, 0.5, 0.6, 100);
  std::cout << "Constructor called" << std::endl;
  std::vector<Node> path_vector = new_ant_colony.ant_colony();
  print("out");
  // PrintPath(path_vector, start, goal, grid);
  return 0;
}
#endif
