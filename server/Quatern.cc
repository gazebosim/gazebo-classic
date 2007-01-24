#include <math.h>
#include "Quatern.hh"

// Constructors
Quatern::Quatern()
  : u(1), x(0), y(0), z(0)
{

}

Quatern::Quatern( const double &u, const double &x, const double &y, const double &z)
  : u(u), x(x), y(y), z(z)
{
}

Quatern::Quatern( const Quatern &qt )
{
  this->u = qt.u;
  this->x = qt.x;
  this->y = qt.y;
  this->z = qt.z;
}

// Destructor
Quatern::~Quatern()
{
}

// Equal operator
const Quatern &Quatern::operator=(const Quatern &qt)
{
  this->u = qt.u;
  this->x = qt.x;
  this->y = qt.y;
  this->z = qt.z;

  return *this;
}

void Quatern::SetToIdentity()
{
  this->u = 1.0;
  this->x = 0.0;
  this->y = 0.0;
  this->z = 0.0;
}


// Inver the quaternion
void Quatern::Invert()
{
  this->x = -this->x;
  this->y = -this->y;
  this->z = -this->z;
}

// Get the inverse of this quaternion
Quatern Quatern::GetInverse() const
{
  Quatern q;

  q.u = this->u;
  q.x = -this->x;
  q.y = -this->y;
  q.z = -this->z;

  return q;
}


// Normalize the quaternion
void Quatern::Normalize()
{
  double s;

  s = sqrt(this->u * this->u + this->x * this->x + this->y * this->y + this->z * this->z);

  this->u /= s;
  this->x /= s;
  this->y /= s;
  this->z /= s;
}

// Set the quaternion from an axis and angle
void Quatern::SetFromAxis(double ax, double ay, double az, double aa)
{
  double l;

  l = ax * ax + ay * ay + az * az;

  if (l > 0.0)
  {
    aa *= 0.5;
    l = sin(aa) / sqrt(l);
    this->u = cos(aa);
    this->x = ax * l;
    this->y = ay * l;
    this->z = az * l;
  }
  else
  {
    this->u = 1;
    this->x = 0;
    this->y = 0;
    this->z = 0;
  }

  this->Normalize();
}
       
// Set the quaternion from Euler angles
void Quatern::SetFromEuler(double roll, double pitch, double yaw)
{
  double phi, the, psi;

  phi = roll / 2;
  the = pitch / 2;
  psi = yaw / 2;

  this->u = cos(phi) * cos(the) * cos(psi) + sin(phi) * sin(the) * sin(psi);
  this->x = sin(phi) * cos(the) * cos(psi) - cos(phi) * sin(the) * sin(psi);
  this->y = cos(phi) * sin(the) * cos(psi) + sin(phi) * cos(the) * sin(psi);
  this->z = cos(phi) * cos(the) * sin(psi) - sin(phi) * sin(the) * cos(psi);

  this->Normalize();
}

// Return the rotation in Euler angles
Vector3 Quatern::GetAsEuler()
{
  double phi, the, psi;

  this->Normalize();

  phi = atan2(2 * (this->y*this->z + this->u*this->x), 
      (this->u*this->u - this->x*this->x - this->y*this->y + this->z*this->z));
  the = asin(-2 * (this->x*this->z - this->u * this->y));
  psi = atan2(2 * (this->x*this->y + this->u*this->z), 
      (this->u*this->u + this->x*this->x - this->y*this->y - this->z*this->z));

  return Vector3(phi, the, psi);
}

// Return rotation as axis and angle (x, y, y, rotation)
Quatern Quatern::GetAsAxis()
{
  return Quatern(acos(this->u)*2, this->x, this->y, this->z);
}

// Scale a Quaternion
void Quatern::Scale(double scale)
{
  Quatern b;

  // Convert to axis-and-angle
  b = this->GetAsAxis();
  b.u *= scale;

  this->SetFromAxis(b.x, b.y, b.z, b.u);
}

// Multiplication operator
Quatern Quatern::operator*( const Quatern &qt ) const
{
  Quatern c;

  c.u = this->u * qt.u - this->x * qt.x - this->y * qt.y - this->z * qt.z;
  c.x = this->u * qt.x + this->x * qt.u + this->y * qt.z - this->z * qt.y;
  c.y = this->u * qt.y - this->x * qt.z + this->y * qt.u + this->z * qt.x;
  c.z = this->u * qt.z + this->x * qt.y - this->y * qt.x + this->z * qt.u;

  return c;
}

// See if a quatern is finite (e.g., not nan)
bool Quatern::IsFinite()
{
  return finite(this->u) && finite(this->x) && finite(this->y) && finite(this->z);
}

////////////////////////////////////////////////////////////////////////////////
// Ostream operator
std::ostream &operator<<( std::ostream &out, const Quatern &q )
{
  out << q.u << " " << q.x << " " << q.y << " " << q.z;

  return out;
}
