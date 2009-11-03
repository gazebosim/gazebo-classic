#include "Geom.hh"
#include "Shape.hh"

using namespace gazebo;

std::string Shape::TypeNames[Shape::TYPE_COUNT] = {"box", "cylinder", "heightmap", "map", "sphere", "plane", "ray", "trimesh", "multiray"};

////////////////////////////////////////////////////////////////////////////////
// Constructor
Shape::Shape(Geom *p)
  : parent(p) 
{
  this->parent->SetShape(this);
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
Shape::~Shape()
{
}

////////////////////////////////////////////////////////////////////////////////
// Get the type
Shape::Type Shape::GetType() const
{
  return this->type;
}
