#include "Plane.hh"

using namespace gazebo;

Plane::Plane()
{
  this->d = 0.0;
}

Plane::~Plane()
{
}

void Plane::Set(Vector3 n, Vector2<double> s, double offset)
{
  this->normal = n;
  this->size = s;
  this->d = offset;
}

////////////////////////////////////////////////////////////////////////////////
/// Equal operator
const Plane &Plane::operator=(const Plane & p)
{
  this->normal = p.normal;
  this->size = p.size;
  this->d = p.d;

  return *this;
}
