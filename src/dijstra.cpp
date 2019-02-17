/*

Dijstra grid based planning

*/

#include "main.h"
#include <iomanip>
class Node{
//private:
public:

  int x, y, cost, pind;

  Node(int x = 0, int y = 0, int cost = 0, int pind = 0){
    this->x = x;
    this->y = y;
    this->cost = cost;
    this->pind = pind;
  }
  void print_status(){
    std::cout << "--------------" << std::endl
              << "Node:" << std::endl
              << "x   : " << x    << std::endl
              << "y   : " << y    << std::endl
              << "cost: " << cost << std::endl
              << "pind: " << pind << std::endl
              << "--------------" << std::endl;
  }
  Node operator+(Node p){
    Node tmp;
    tmp.x = this->x + p.x;
    tmp.y = this->y + p.y;
    tmp.cost = this->cost + p.cost;
    tmp.pind = this->pind + p.pind;
    return tmp;
  }
  Node operator=(Node p){
    this->x = p.x;
    this->y = p.y;
    this->cost = p.cost;
    this->pind = p.pind;
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
      (*p_grid)[i][j] = distr(eng)/3;
    }
  }
}

void print_grid(void *grid, int n){
  //NOTE: Using a void pointer isnt the best option

  int (*p_grid)[n][n] = (int (*)[n][n]) grid;
  for(int i=0;i<n;i++){
    for(int j=0;j<n;j++){
      //std::cout << std::setw(10) << (*p_grid)[i][j] << " , ";
      std::cout << (*p_grid)[i][j] << " , ";
    }
    std::cout << std::endl;
  }
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
  int n = 10;
  int num_points = n*n;
//  point sptSet[num_points];
  int grid[n][n];
  make_grid(grid, n);
  print_grid(grid, n);
  return 0;
}
