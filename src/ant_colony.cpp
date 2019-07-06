/**
* @file ant_colony.cpp
* @author vss2sn
* @brief Contains the Ant Colony class
*/

#include "ant_colony.hpp"
#include<climits>
#include <unordered_map>
#include <utility>

struct pair_hash
{
	template <class T1, class T2>
	std::size_t operator() (const std::pair<T1, T2> &pair) const
	{
		return std::hash<T1>()(pair.first) ^ std::hash<T2>()(pair.second);
	}
};

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
  std::unordered_map<std::pair<int,int>,double, pair_hash> pheromone_grid;
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
    // pheromone_grid = std::vector<std::vector<double>>(grid_size, std::vector<double> (grid_size, 1));
    for(int i=0;i<grid_size;i++){
      for(int j=0;j<grid_size;j++){
        auto motions = GetMotion();
        for(auto& motion : motions){
          Node c = Node(i,j) + motion;
          if(c.x_ >=0 && c.x_ < grid_size && c.y_ >=0 && c.y_ < grid_size){
              pheromone_grid.insert({std::make_pair(i*grid_size + j, c.x_*grid_size + c.y_) , 1.0});
          }

        }
        // std::cout << "Hello"<<std::endl;
      }
    }
    this->max_steps = pow(grid_size,2);
    this->ants = std::vector<Ant>(n_ants);
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
          // usleep(250000);
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
              // std::cout << "First term: " << pow(pheromone_grid[std::make_pair(possible_position.x_*grid_size + possible_position.y_, ant.current_node.x_*grid_size + ant.current_node.y_)], alpha) << std::endl;
              // std::cout << "Second term: " << pow(1.0/pow(pow((possible_position.x_ - goal.x_),2) +pow((possible_position.y_ - goal.y_),2)/grid_size,0.5), beta) << std::endl;
              double new_prob = pow(pheromone_grid[std::make_pair(possible_position.x_*grid_size + possible_position.y_, ant.current_node.x_*grid_size + ant.current_node.y_)], alpha) *
                pow(1.0/pow(pow((possible_position.x_ - goal.x_),2) +pow((possible_position.y_ - goal.y_),2),0.5), beta);
                // std::cout << "New prob: " << new_prob << std::endl;
              possible_probabilities.push_back(new_prob);
              prob_sum += new_prob;
              if(grid[possible_position.x_][possible_position.y_]==1) n_obs+=1;
            }
          }
          if(n_obs==possible_positions.size()){
            // Ant in a cul-de-sac
            std::cout << "Breaking" << std::endl;
            break;
          }
          else if(prob_sum == 0){
            // std::cout << "Nobs: " <<n_obs << std::endl;
            double new_prob = 1.0/(possible_positions.size()-n_obs);
            for(int i=0;i<possible_positions.size();i++){
              if(grid[possible_positions[i].x_][possible_positions[i].y_]==0) possible_probabilities[i]=new_prob;
              else possible_probabilities[i]=0;
            }
          }
          else{
              for(auto& p : possible_probabilities) p/=prob_sum;
          }
          // for(auto p : possible_probabilities ) std::cout << p << " ";
          // std::cout<< std::endl;
          std::discrete_distribution<> dist(possible_probabilities.begin(), possible_probabilities.end());
          ant.previous_node = ant.current_node;
          ant.current_node = possible_positions[dist(engine)];
          for(auto it=ant.path.begin(); it!=ant.path.end();++it){
            // std::cout << "in here" << std::endl;
            if(*it==ant.current_node){
              ant.steps = ant.path.end() - it;
              ant.path.erase(it, ant.path.end());
              break;
            }
          }
          // std::cout << "out here" << std::endl;
          ant.steps++;
        }
        if(ant.current_node==goal){
          ant.path.push_back(ant.current_node);
          ant.found_goal = true;
          n_success++;
        }
        ants[j] = ant;
      }//for every ant loop ends here
      // for(auto row : pheromone_grid){
      //   for(auto col : row){
      //     if(col!=0) col = (1-evap_rate)*col;
      //   }
      // }
      for(auto it = pheromone_grid.begin(); it!=pheromone_grid.end();it++){
        it->second = it->second*(1-evap_rate);
      }
      double avg_len = 0;
      int count = 0;
      int bpl = INT_MAX;
      std::vector<Node> bp;
      for(Ant& ant : ants){
        if(ant.found_goal){
          avg_len += ant.path.size();
          if(ant.path.size()<bpl){
            bpl = ant.path.size();
            bp = ant.path;
          }
          count+=1;
          double c = 10.0/(ant.path.size()-1);///ants.size();
          // std::cout << "c: " << c << std::endl;
          for(int i=1;i<ant.path.size();i++){
            auto it = pheromone_grid.find(std::make_pair(ant.path[i].x_*grid_size + ant.path[i].y_, ant.path[i-1].x_*grid_size + ant.path[i-1].y_));
            it->second+= c;
            // it = pheromone_grid.find(std::make_pair(ant.path[i-1].x_*grid_size + ant.path[i-1].y_, ant.path[i].x_*grid_size + ant.path[i].y_));
            // it->second+= c;
          }
        }
      }
      // for(auto& row : pheromone_grid){
      //   for(auto& col : row){
      //     std::cout << col << " ";
      //   }
      //   std::cout << std::endl;
      // }
      if (count>0){
        // std::cout << "Avg path length = " << (avg_len-1)/count << std::endl;
        std::cout << "Best path length = " << bpl << std::endl;
      }
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
  int n = 30;

  std::vector<std::vector<int>> grid(n);
  std::vector<int> tmp(n);
  for (int i = 0; i < n; i++){
    grid[i] = tmp;
  }
  // MakeGrid(grid);
  // std::random_device rd; // obtain a random number from hardware
  // std::mt19937 eng(rd()); // seed the generator
  // std::uniform_int_distribution<int> distr(0,n-1); // define the range
  //
  // Node start(distr(eng),distr(eng),0,0,0,0);
  // Node goal(distr(eng),distr(eng),0,0,0,0);
  // std::vector<std::vector<int>> grid{
  //   { 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 },
  //   { 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 },
  //   { 0 , 1 , 0 , 0 , 0 , 0 , 1 , 0 },
  //   { 0 , 1 , 0 , 0 , 0 , 0 , 1 , 0 },
  //   { 0 , 1 , 0 , 0 , 0 , 0 , 1 , 0 },
  //   { 0 , 1 , 0 , 0 , 0 , 0 , 1 , 0 },
  //   { 0 , 1 , 1 , 1 , 1 , 1 , 1 , 0 },
  //   { 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 }
  //                  } ;
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

  AntColony new_ant_colony(grid, start, goal, 20, 1, 0, 0.3, 25);
  std::cout << "Constructor called" << std::endl;
  std::vector<Node> path_vector = new_ant_colony.ant_colony();
  print("out");
  // PrintPath(path_vector, start, goal, grid);
  return 0;
}
#endif
