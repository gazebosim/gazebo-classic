#include <iostream>
#include <fstream>
#include <stdio.h>
#include <vector>

#include "collision_map_grid.h"

using namespace std;

CollisionMapGrid::CollisionMapGrid(int xStart, int xDim, int yStart, int yDim)
{
  x_start = xStart;
  x_dim = xDim;
  y_start = yStart;
  y_dim = yDim;
  cells = new int[xDim * yDim];
  for (int i=0; i<(xDim*yDim); i++) {
    cells[i] = 0;
  }
}

CollisionMapGrid::~CollisionMapGrid()
{
  delete [] cells;
}

int CollisionMapGrid::getXDim()
{
  return x_dim;
}

int CollisionMapGrid::getYDim()
{
  return y_dim;
}

int CollisionMapGrid::getXStart()
{
  return x_start;
}

int CollisionMapGrid::getYStart()
{
  return y_start;
}

bool CollisionMapGrid::isFree(int x, int y) 
{
  return !(cells[x + (y*y_dim)]);
}

void CollisionMapGrid::fill(int x, int y)
{
  cells[x + (y*y_dim)] = 1;
}

void CollisionMapGrid::print(std::string filename)
{
  ofstream outfile;
  outfile.open (filename);
  outfile << "+";
  for (int i=0; i<y_dim; i++) {
    outfile << "-";
  }
  outfile << "+\n";
  for (int i=0; i<y_dim; i++) {
    outfile << "|";
    for (int j=0; j<x_dim; j++) {    
      outfile << cells[(y_dim * i) + j];
    }
    outfile << "|\n";
  }
  outfile << "+";
  for (int i=0; i<y_dim; i++) {
    outfile << "-";
  }
  outfile << "+\n";
}

int main()
{
  CollisionMapGrid *demo = new CollisionMapGrid(0, 5, 0, 5);  
  demo->print("test.txt");
  return 0;
}
