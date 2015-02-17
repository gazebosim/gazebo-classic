#include <iostream>
#include <fstream>
#include <stdio.h>
#include <vector>

#include "collision_map_grid.h"

using namespace std;

Node::Node()
{
  full = 0;
  dst = 0;
  priority = 0;
}

int Node::getXPos()
{
  return x_pos;
}

int Node::getYPos()
{
  return y_pos;
}

int Node::getFull()
{
  return full;
}

int Node::getDst()
{
  return dst;
}

int Node::getPriority()
{
  return priority;
}

Node *Node::getParent()
{
  return parent;
}

void Node::setFull(int x)
{
  full = x;
}

void Node::setDst(int x)
{
  dst = x;
}

void Node::setPriority(int x)
{
  priority = x;
}

void Node::setParent(Node *n)
{
  parent = n;
}

CollisionMapGrid::CollisionMapGrid(int xStart, int xDim, int yStart, int yDim)
{
  x_start = xStart;
  x_dim = xDim;
  y_start = yStart;
  y_dim = yDim;
  nodes = new Node *[xDim * yDim];
  for (int i=0; i<(xDim*yDim); i++) {
    nodes[i] = new Node();
  }
}

CollisionMapGrid::~CollisionMapGrid()
{
  delete [] nodes;
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
  return !(nodes[x + (y*y_dim)]->getFull());
}

void CollisionMapGrid::fill(int x, int y)
{
  nodes[x + (y*y_dim)]->setFull(1);
}

Node *CollisionMapGrid::getNode(int x, int y)
{
  return nodes[x + (y*y_dim)];
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
      outfile << nodes[(y_dim * i) + j]->getFull();
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
