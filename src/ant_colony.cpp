/**
* @file ant_colony.cpp
* @author vss2sn
* @brief Contains the Ant and Ant Colony class
*/

#include "ant_colony.hpp"

Ant::Ant(Node start, int id){
	this->id = id;
  this->current_node = start;
  this->previous_node = Node(-1,-1);
}

AntColony::AntColony(std::vector<std::vector<int>>& grid, Node start, Node goal, int n_ants, double alpha, double beta, double evap_rate, int iterations, double Q){
  this->grid = grid;
  this->n_ants = n_ants;
  this->alpha = alpha;
  this->beta = beta;
  this->evap_rate = evap_rate;
  this->iterations = iterations;
  this->start = start; // Make sure start has id
  this->goal = goal;
  grid_size = grid.size();
	Node c;
	motions = GetMotion();
  for(int i=0;i<grid_size;i++){
    for(int j=0;j<grid_size;j++){
      for(auto& motion : motions){
        c = Node(i,j) + motion;
        if(c.x_ >=0 && c.x_ < grid_size && c.y_ >=0 && c.y_ < grid_size) pheromone_edges.insert({std::make_pair(i*grid_size + j, c.x_*grid_size + c.y_) , 1.0});
      }
    }
  }
  this->max_steps = pow(grid_size,2)/2+grid_size;
  this->ants = std::vector<Ant>(n_ants);
	this->Q = Q;
}

void AntColony::PrintAntPath(Ant& ant){
	for(int k=1;k<ant.path.size();k++) ant.path[k].pid_ = ant.path[k-1].id_;
	ant.path.back().id_ = ant.path.back().x_*grid_size + ant.path.back().y_;
	auto grid_2 = grid;
	PrintPath(ant.path, start, ant.path.back(), grid_2);
	sleep(1);
}

void AntColony::RemoveLoop(Ant& ant){
	for(auto it=ant.path.begin(); it!=ant.path.end();++it){
		if(*it==ant.current_node){
			ant.steps = ant.path.end() - it;
			ant.path.erase(it, ant.path.end());
			break;
		}
	}
}

std::vector<Node> AntColony::ant_colony(){
  std::random_device device;
  std::mt19937 engine(device());
  std::vector<Node> last_best_path; // saves best path of last iteration. Not over all.
  Node possible_position;
  for(int i=0;i<iterations;i++){
    for(int j=0;j<n_ants;j++){
			// Can assign a thread to each ant if parallel required
      Ant ant(start, j);
      while(ant.current_node!=goal && ant.steps < max_steps){
        ant.path.push_back(ant.current_node);

				// Get next position
        std::vector<Node> possible_positions;
        std::vector<double> possible_probabilities;
        float prob_sum = 0;
        int n_obs = 0;
        for(auto& m : motions){
          possible_position = ant.current_node + m;
					possible_position.id_ = possible_position.x_*grid_size + possible_position.y_;
          if(possible_position.x_ >=0 && possible_position.x_ < grid_size && possible_position.y_ >=0 && possible_position.y_ < grid_size
             && possible_position!=ant.previous_node){
            possible_positions.push_back(possible_position);
            double new_prob = pow(pheromone_edges[std::make_pair(possible_position.id_, ant.current_node.id_)], alpha) *
              pow(1.0/pow(pow((possible_position.x_ - goal.x_),2) +pow((possible_position.y_ - goal.y_),2),0.5), beta);
						if(grid[possible_position.x_][possible_position.y_]==1){
							n_obs+=1;
							new_prob = 0;
						}
            possible_probabilities.push_back(new_prob);
            prob_sum += new_prob;
          }
        }
        if(n_obs==possible_positions.size()) break;// Ant in a cul-de-sac
        else if(prob_sum == 0){
          double new_prob = 1.0/(possible_positions.size()-n_obs);
          for(int i=0;i<possible_positions.size();i++){
            if(grid[possible_positions[i].x_][possible_positions[i].y_]==0) possible_probabilities[i]=new_prob;
            else possible_probabilities[i]=0;
          }
        }
        else for(auto& p : possible_probabilities) p/=prob_sum;
        std::discrete_distribution<> dist(possible_probabilities.begin(), possible_probabilities.end());
        ant.previous_node = ant.current_node;
        ant.current_node = possible_positions[dist(engine)];

				// Removing any loops if reached previously reached point
				RemoveLoop(ant);

        ant.steps++;
      }
			// If goal found, add to path
      if(ant.current_node==goal){
				ant.current_node.id_ = ant.current_node.x_*grid_size + ant.current_node.y_;
        ant.path.push_back(ant.current_node);
        ant.found_goal = true;
      }
      ants[j] = ant;
    }

		// Pheromone deterioration
    for(auto it = pheromone_edges.begin(); it!=pheromone_edges.end();it++) it->second = it->second*(1-evap_rate);

    int bpl = INT_MAX;
    std::vector<Node> bp;

		// Pheromone update based on successful ants
    for(Ant& ant : ants){
			// PrintAntPath(Ant& ant);
      if(ant.found_goal){ // Use iff goal reached
        if(ant.path.size() < bpl){ // Save best path yet in this iteration
          bpl = ant.path.size();
          bp = ant.path;
        }
        double c = Q/(ant.path.size()-1); //c = cost / reward. Reward here, increased pheromone
        for(int i=1;i<ant.path.size();i++){ // Assuming ant can tell which way the food was based on how the phermones detereorate. Good for path planning as prevents moving in the opposite direction to path and improves convergence
          auto it = pheromone_edges.find(std::make_pair(ant.path[i].id_, ant.path[i-1].id_));
          it->second+= c;
        }
      }
    }

    if (!bp.empty()) std::cout << "Best path length = " << bpl << std::endl;
    else std::cout << "Failed to find path" << std::endl;
    if(i+1==iterations) last_best_path = bp;
  }//for every iteration loop ends here
	if(last_best_path.empty()){
		last_best_path.clear();
		last_best_path.push_back(Node(-1,-1));
		return last_best_path;
	}
  for(int i=1;i<last_best_path.size();i++) last_best_path[i].pid_ = last_best_path[i-1].id_;
  last_best_path.back().id_ = last_best_path.back().x_*grid_size + last_best_path.back().y_;
  return last_best_path;
}

#ifdef BUILD_INDIVIDUAL
/**
* @brief Script main function. Generates start and end nodes as well as grid, then creates the algorithm object and calls the main algorithm function.
* @return 0
*/
int main(){
  int n = 20;

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
  PrintGrid(grid);

	//Normally as  beta increases the solution becomes greedier. However, as the heuristic is < 1 here, reducing beta increases the value places on the heuristic
  AntColony new_ant_colony(grid, start, goal, 10, 1, 0.5, 0.3, 50, 10.0);
  std::vector<Node> path_vector = new_ant_colony.ant_colony();
  PrintPath(path_vector, start, goal, grid);
  return 0;
}
#endif
