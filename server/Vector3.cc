#include <math.h>

#include "Vector3.hh"

////////////////////////////////////////////////////////////////////////////////
// Constructor
Vector3::Vector3()
  : x(0), y(0), z(0)
{
}

////////////////////////////////////////////////////////////////////////////////
// Constructor
Vector3::Vector3( const double &x, const double &y, const double &z )
  : x(x), y(y), z(z)
{
}

////////////////////////////////////////////////////////////////////////////////
// Copy Constructor
Vector3::Vector3( const Vector3 &pt )
  : x(pt.x), y(pt.y), z(pt.z)
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
Vector3::~Vector3()
{
}

// Calc distance to the given point
double Vector3::Distance(const Vector3 &pt ) const
{
  return sqrt((this->x-pt.x)*(this->x-pt.x) + (this->y-pt.y)*(this->y-pt.y) + (this->z-pt.z)*(this->z-pt.z));
}

// Normalize the vector length
void Vector3::Normalize()
{
  double d = sqrt(this->x * this->x + this->y * this->y + this->z * this->z);

  this->x /= d;
  this->y /= d;
  this->z /= d;
}

// Set the contents of the vector
void Vector3::Set(double x, double y, double z)
{
  this->x = x;
  this->y = y;
  this->z = z;
}

Vector3 Vector3::GetCrossProd(const Vector3 &pt) const
{
  Vector3 c;

  c.x =  this->y * pt.z - this->z * pt.y;
  c.y = -this->x * pt.z + this->z * pt.x;
  c.z =  this->x * pt.y - this->y * pt.x;

  return c;
}

////////////////////////////////////////////////////////////////////////////////
// Equals operator
const Vector3 &Vector3::operator=( const Vector3 &pt )
{
  this->x = pt.x;
  this->y = pt.y;
  this->z = pt.z;

  return *this;
}


////////////////////////////////////////////////////////////////////////////////
// Addition operator
Vector3 Vector3::operator+( const Vector3 &pt ) const
{
  return Vector3(this->x + pt.x, this->y + pt.y, this->z + pt.z);
}

const Vector3 &Vector3::operator+=( const Vector3 &pt )
{
  this->x += pt.x;
  this->y += pt.y;
  this->z += pt.z;

  return *this;
}

////////////////////////////////////////////////////////////////////////////////
// Subtraction operators
Vector3 Vector3::operator-( const Vector3 &pt ) const
{
  return Vector3(this->x - pt.x, this->y - pt.y, this->z - pt.z);
}

const Vector3 &Vector3::operator-=( const Vector3 &pt )
{
  this->x -= pt.x;
  this->y -= pt.y;
  this->z -= pt.z;

  return *this;
}


////////////////////////////////////////////////////////////////////////////////
// Division operators

const Vector3 Vector3::operator/( const Vector3 &pt ) const
{
  return Vector3(this->x / pt.x, this->y / pt.y, this->z / pt.z);
}

const Vector3 &Vector3::operator/=( const Vector3 &pt )
{
  this->x /= pt.x;
  this->y /= pt.y;
  this->z /= pt.z;

  return *this;
}


////////////////////////////////////////////////////////////////////////////////
// Mulitplication operators
const Vector3 Vector3::operator*( const Vector3 &pt ) const
{
  return Vector3(this->x * pt.x, this->y * pt.y, this->z * pt.z);
}

const Vector3 &Vector3::operator*=( const Vector3 &pt )
{
  this->x *= pt.x;
  this->y *= pt.y;
  this->z *= pt.z;

  return *this;
}


////////////////////////////////////////////////////////////////////////////////
// Equality operator
bool Vector3::operator==( const Vector3 &pt ) const
{
  return this->x == pt.x && this->y == pt.y && this->z == pt.z;
}

////////////////////////////////////////////////////////////////////////////////
// Inequality operator
bool Vector3::operator!=( const Vector3 &pt ) const
{
  return !(*this == pt);
}

////////////////////////////////////////////////////////////////////////////////
// Ostream operator
std::ostream &operator<<( std::ostream &out, const Vector3 &pt )
{
  out << pt.x << " " << pt.y << " " << pt.z;

  return out;
}

// See if a point is finite (e.g., not nan)
bool Vector3::IsFinite() const
{
  return finite(this->x) && finite(this->y) && finite(this->z);
}


