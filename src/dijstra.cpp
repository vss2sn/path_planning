/*

Dijstra grid based planning

*/

#include "main.h"

struct point {
  int x,y;
  point operator+(point p){
    std::cout << "In overloaded operator +" << std::endl;
    point tmp;
    tmp.x = this->x + p.x;
    tmp.y = this->y + p.y;
    std::cout << "About to return" << std::endl;
    return tmp;
  }
  point operator=(point p){
    std::cout << "In overloaded operator =" << std::endl;
    this->x = p.x;
    this->y = p.y;
  }
};

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
    std::cout << "Node:" << std::endl
              << "x   : " << x    << std::endl
              << "y   : " << y    << std::endl
              << "cost: " << cost << std::endl
              << "pind: " << pind << std::endl;
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
  std::srand(10);
  for(int i=0;i<n;i++){
    for(int j=0;j<n;j++){
      *p_grid[i][j] = (std::rand() % 4)/3;
    }
  }
}

void print_grid(void *grid, int n){
  //NOTE: Using a void pointer isnt the best option
  int (*p_grid)[n][n] = (int (*)[n][n]) grid;
  for(int i=0;i<n;i++){
    for(int j=0;j<n;j++){
      std::cout << *p_grid[i][j] << " , ";
    }
    std::cout << std::endl;
  }
}

int main(){
/*
  Node a;
  Node b;
  a.x = 1;
  a.y = 1;
  b.x = 1;
  b.y = 1;
  std::cout << "Done with init" <<std::endl;
  Node c;
  c=a+b;
  std::cout << c.x <<std::endl;
  std::cout << c.y <<std::endl;
  c=a;
  std::cout << c.x <<std::endl;
  std::cout << c.y <<std::endl;
*/

  int n = 10;
  int num_points = n*n;
//  point sptSet[num_points];
  int grid[n][n];
  make_grid(grid, n);
  print_grid(grid, n);
  return 0;
}
