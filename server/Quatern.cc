/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003  
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/* Desc: External interfaces for Gazebo
 * Author: Nate Koenig
 * Date: 03 Apr 2007
 * SVN: $Id$
 */
#include <math.h>
#include "Quatern.hh"

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
// Default Constructor
Quatern::Quatern()
  : u(1), x(0), y(0), z(0)
{

}

////////////////////////////////////////////////////////////////////////////////
// Constructor
Quatern::Quatern( const double &u, const double &x, const double &y, const double &z)
  : u(u), x(x), y(y), z(z)
{
}

////////////////////////////////////////////////////////////////////////////////
// Copy Constructor
Quatern::Quatern( const Quatern &qt )
{
  this->u = qt.u;
  this->x = qt.x;
  this->y = qt.y;
  this->z = qt.z;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
Quatern::~Quatern()
{
}

////////////////////////////////////////////////////////////////////////////////
// Equal operator
const Quatern &Quatern::operator=(const Quatern &qt)
{
  this->u = qt.u;
  this->x = qt.x;
  this->y = qt.y;
  this->z = qt.z;

  return *this;
}

////////////////////////////////////////////////////////////////////////////////
// Set quatern to identity
void Quatern::SetToIdentity()
{
  this->u = 1.0;
  this->x = 0.0;
  this->y = 0.0;
  this->z = 0.0;
}


////////////////////////////////////////////////////////////////////////////////
// Invert the quaternion
void Quatern::Invert()
{
  this->x = -this->x;
  this->y = -this->y;
  this->z = -this->z;
}

////////////////////////////////////////////////////////////////////////////////
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


////////////////////////////////////////////////////////////////////////////////
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

////////////////////////////////////////////////////////////////////////////////
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
       
////////////////////////////////////////////////////////////////////////////////
// Set the quaternion from Euler angles
void Quatern::SetFromEuler(const Vector3 &vec)
{
  double phi, the, psi;

  phi = vec.x / 2;
  the = vec.y / 2;
  psi = vec.z / 2;

  this->u = cos(phi) * cos(the) * cos(psi) + sin(phi) * sin(the) * sin(psi);
  this->x = sin(phi) * cos(the) * cos(psi) - cos(phi) * sin(the) * sin(psi);
  this->y = cos(phi) * sin(the) * cos(psi) + sin(phi) * cos(the) * sin(psi);
  this->z = cos(phi) * cos(the) * sin(psi) - sin(phi) * sin(the) * cos(psi);

  this->Normalize();
}

////////////////////////////////////////////////////////////////////////////////
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

////////////////////////////////////////////////////////////////////////////////
/// \brief Get the Euler roll angle in radians
double Quatern::GetRoll()
{
  return this->GetAsEuler().x;
}

////////////////////////////////////////////////////////////////////////////////
/// \brief Get the Euler pitch angle in radians
double Quatern::GetPitch()
{
  return this->GetAsEuler().y;
}

////////////////////////////////////////////////////////////////////////////////
/// \brief Get the Euler yaw angle in radians
double Quatern::GetYaw()
{
  return this->GetAsEuler().z;
}

////////////////////////////////////////////////////////////////////////////////
// Return rotation as axis and angle (x, y, y, rotation)
Quatern Quatern::GetAsAxis()
{
  return Quatern(acos(this->u)*2, this->x, this->y, this->z);
}

////////////////////////////////////////////////////////////////////////////////
// Scale a Quaternion
void Quatern::Scale(double scale)
{
  Quatern b;

  // Convert to axis-and-angle
  b = this->GetAsAxis();
  b.u *= scale;

  this->SetFromAxis(b.x, b.y, b.z, b.u);
}

////////////////////////////////////////////////////////////////////////////////
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

////////////////////////////////////////////////////////////////////////////////
// See if a quatern is finite (e.g., not nan)
bool Quatern::IsFinite() const
{
  return finite(this->u) && finite(this->x) && finite(this->y) && finite(this->z);
}

