// In collision_map_grid.cpp
#include <stdlib.h>

class CollisionMapGrid
{
public:
  int x_start; // world x coordinate bound (smallest x)
  int x_dim; // x dimension or row length
  int y_start; // world y coordinate bound (largest y)
  int y_dim; // y dimension or num rows
  int *cells; // pointer to grid of cells

  CollisionMapGrid(int x1, int x_dim, int y1, int y_dim);
  ~CollisionMapGrid();
  int getXDim();
  int getYDim();
  int getXStart();
  int getYStart();
  bool isFree(int x, int y);
  void fill(int x, int y);
  void print(std::string filename);
};
