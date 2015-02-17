// In collision_map_grid.cpp
#include <stdlib.h>

class Node
{
 public:
  int x_pos;
  int y_pos;
  int full; // is this cell occupied?
  int dst; // distance from start node
  int priority; // dst + remaining distance estimate
  Node *parent; // Previous node in this path

  Node();
  int getXPos();
  int getYPos();
  int getFull();
  int getDst();
  int getPriority();
  Node *getParent();
  void setFull(int x);
  void setDst(int x);
  void setPriority(int x);
  void setParent(Node *n);
};

class CollisionMapGrid
{
public:
  int x_start; // world x coordinate bound (smallest x)
  int x_dim; // x dimension or row length
  int y_start; // world y coordinate bound (largest y)
  int y_dim; // y dimension or num rows
  Node **nodes; // pointer to grid of cells

  CollisionMapGrid(int x1, int x_dim, int y1, int y_dim);
  ~CollisionMapGrid();
  int getXDim();
  int getYDim();
  int getXStart();
  int getYStart();
  Node *getNode(int x, int y);
  bool isFree(int x, int y);
  void fill(int x, int y);
  void print(std::string filename);
};
