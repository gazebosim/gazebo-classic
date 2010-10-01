#include "World.hh"
#include "Geom.hh"
#include "Shape.hh"

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
// Constructor
Shape::Shape(Geom *p)
  : Common(p) 
{
  this->type.push_back("shape");
  this->geomParent = p;
  this->geomParent->SetShape(this);
  this->physicsEngine = World::Instance()->GetPhysicsEngine();
  this->SetName("shape");
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
Shape::~Shape()
{
  if (this->geomParent)
    this->geomParent->SetShape(NULL);
}
