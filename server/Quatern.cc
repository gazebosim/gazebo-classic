/*
 * Copyright 2011 Nate Koenig & Andrew Howard
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
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
  if ( u == 0 && x == 0 && y == 0 && z == 0 )
  {
    /// @todo: give user warning
    this->SetToIdentity();
  }
  this->Normalize();
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
  double norm = this->u*this->u+this->x*this->x+this->y*this->y+this->z*this->z;
  this->u = this->u/norm;
  this->x = -this->x/norm;
  this->y = -this->y/norm;
  this->z = -this->z/norm;
}

////////////////////////////////////////////////////////////////////////////////
// Get the inverse of this quaternion
Quatern Quatern::GetInverse() const 
{
  Quatern q;

  double norm = this->u*this->u+this->x*this->x+this->y*this->y+this->z*this->z;

  if (norm > 0.0)
  {
    q.u = this->u / norm;
    q.x = -this->x / norm;
    q.y = -this->y / norm;
    q.z = -this->z / norm;
  }

  return q;
}


////////////////////////////////////////////////////////////////////////////////
// Normalize the quaternion
void Quatern::Normalize()
{
  double s;

  s = sqrt(this->u * this->u + this->x * this->x + this->y * this->y + this->z * this->z);

  if (s == 0)
  {
    this->u = 0.0;
    this->x = 0.0;
    this->y = 0.0;
    this->z = 1.0;
  }
  else
  {
    this->u /= s;
    this->x /= s;
    this->y /= s;
    this->z /= s;
  }
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
/// Set this quaternion from another
void Quatern::Set(double u, double x, double y, double z)
{
  this->u = u;
  this->x = x;
  this->y = y;
  this->z = z;
}

////////////////////////////////////////////////////////////////////////////////
// Set the quaternion from Euler angles
void Quatern::SetFromEuler(const Vector3 &vec)
{
  double phi, the, psi;

  phi = vec.x / 2.0;
  the = vec.y / 2.0;
  psi = vec.z / 2.0;

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
  Vector3 vec;

  double squ;
  double sqx;
  double sqy;
  double sqz;

  this->Normalize();

  squ = this->u * this->u;
  sqx = this->x * this->x;
  sqy = this->y * this->y;
  sqz = this->z * this->z;

  this->Normalize();

  // Roll
  vec.x = atan2(2 * (this->y*this->z + this->u*this->x), squ - sqx - sqy + sqz);

  // Pitch
  double sarg = -2 * (this->x*this->z - this->u * this->y);
  vec.y = sarg <= -1.0 ? -0.5*M_PI : (sarg >= 1.0 ? 0.5*M_PI : asin(sarg));

  // Yaw
  vec.z = atan2(2 * (this->x*this->y + this->u*this->z), squ + sqx - sqy - sqz);

  return vec;
}

////////////////////////////////////////////////////////////////////////////////
/// Convert euler angles to quatern.
Quatern Quatern::EulerToQuatern( const Vector3 &vec )
{
  Quatern result;
  result.SetFromEuler(vec);
  return result;
}

////////////////////////////////////////////////////////////////////////////////
/// Convert euler angles to quatern.
Quatern Quatern::EulerToQuatern( double x, double y, double z)
{
  return EulerToQuatern( Vector3(x,y,z) );
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
// Return rotation as axis and angle
void Quatern::GetAsAxis(Vector3 &axis, double &angle) const
{
  double len = this->x*this->x + this->y*this->y + this->z*this->z;
  if (len > 0.0)
  {
    angle = 2.0 * acos(this->u);
    double invLen =  1.0 / sqrt(len);
    axis.Set( this->x*invLen, this->y*invLen, this->z*invLen);
  }
  else
  {
    angle = 0.0;
    axis.Set(1,0,0);
  }
}

////////////////////////////////////////////////////////////////////////////////
// Scale a Quaternion
void Quatern::Scale(double scale)
{
  Quatern b;
  Vector3 axis;
  double angle;

  // Convert to axis-and-angle
  this->GetAsAxis(axis, angle);
  angle *= scale;

  this->SetFromAxis(axis.x, axis.y, axis.z, angle);
}

////////////////////////////////////////////////////////////////////////////////
/// Addition operator
Quatern Quatern::operator+( const Quatern &qt ) const
{
  Quatern result(this->u + qt.u, this->x + qt.x, 
                 this->y + qt.y, this->z + qt.z);
  return result;
}

////////////////////////////////////////////////////////////////////////////////
/// Addition operator
Quatern Quatern::operator+=( const Quatern &qt ) 
{
  *this = *this + qt;

  return *this;
}

////////////////////////////////////////////////////////////////////////////////
/// Subtraction operator
Quatern Quatern::operator-=( const Quatern &qt )
{
  *this = *this - qt;
  return *this;
}

////////////////////////////////////////////////////////////////////////////////
/// Substraction operator
Quatern Quatern::operator-( const Quatern &qt ) const
{
  Quatern result(this->u - qt.u, this->x - qt.x, 
                 this->y - qt.y, this->z - qt.z);
  return result;
}

////////////////////////////////////////////////////////////////////////////////
// Multiplication operator
Quatern Quatern::operator*( const Quatern &qt ) const
{
  Quatern c;

  c.x = this->u * qt.x + this->x * qt.u + this->y * qt.z - this->z * qt.y;
  c.y = this->u * qt.y - this->x * qt.z + this->y * qt.u + this->z * qt.x;
  c.z = this->u * qt.z + this->x * qt.y - this->y * qt.x + this->z * qt.u;
  c.u = this->u * qt.u - this->x * qt.x - this->y * qt.y - this->z * qt.z;

  return c;
}

////////////////////////////////////////////////////////////////////////////////
// Multiplication operator
Quatern Quatern::operator*=( const Quatern &qt )
{
  *this = *this * qt;
  return *this;
}

////////////////////////////////////////////////////////////////////////////////
/// Rotate a vector 
Vector3 Quatern::operator*( const Vector3 &v ) const
{
/*  Quatern tmp(0.0, v.x, v.y, v.z);
  
  tmp = (*this) * (tmp * this->GetInverse());

  return Vector3(tmp.x, tmp.y, tmp.z);
  */

  Vector3 uv, uuv;
  Vector3 qvec(this->x, this->y, this->z);
  uv = qvec.GetCrossProd(v);
  uuv = qvec.GetCrossProd(uv);
  uv *= (2.0f * this->u);
  uuv *= 2.0f;

  return v + uv + uuv;
}

////////////////////////////////////////////////////////////////////////////////
/// Rotate a vector using the quaternion
Vector3 Quatern::RotateVector(Vector3 vec) const
{
  Quatern tmp;
  Vector3 result;

  tmp.u = 0.0;
  tmp.x = vec.x;
  tmp.y = vec.y;
  tmp.z = vec.z;

  tmp = (*this) * (tmp * this->GetInverse());

  result.x = tmp.x;
  result.y = tmp.y;
  result.z = tmp.z;

  return result;
}

////////////////////////////////////////////////////////////////////////////////
// Do the reverse rotation of a vector by this quaternion
Vector3 Quatern::RotateVectorReverse(Vector3 vec) const
{
  Quatern tmp;
  Vector3 result;

  tmp.u = 0.0;
  tmp.x = vec.x;
  tmp.y = vec.y;
  tmp.z = vec.z;

  tmp =  this->GetInverse() * (tmp * (*this));

  result.x = tmp.x;
  result.y = tmp.y;
  result.z = tmp.z;

  return result;
}


////////////////////////////////////////////////////////////////////////////////
// See if a quatern is finite (e.g., not nan)
bool Quatern::IsFinite() const
{
  return finite(this->u) && finite(this->x) && finite(this->y) && finite(this->z);
}

////////////////////////////////////////////////////////////////////////////////
/// Correct any nan
void Quatern::Correct()
{
  if (!finite(this->x))
    this->x = 0;
  if (!finite(this->y))
    this->y = 0;
  if (!finite(this->z))
    this->z = 0;
  if (!finite(this->u))
    this->u = 1;

  if (this->u == 0 && this->x == 0 && this->y == 0 && this->z == 0)
    this->u = 1;
}


Vector3 Quatern::GetXAxis() const
{
  double fTy  = 2.0f*this->y;
  double fTz  = 2.0f*this->z;

  double fTwy = fTy*this->u;
  double fTwz = fTz*this->u;
  double fTxy = fTy*this->x;
  double fTxz = fTz*this->x;
  double fTyy = fTy*this->y;
  double fTzz = fTz*this->z;

  return Vector3(1.0f-(fTyy+fTzz), fTxy+fTwz, fTxz-fTwy);

}

Vector3 Quatern::GetYAxis() const
{
  double fTx  = 2.0f*this->x;
  double fTy  = 2.0f*this->y;
  double fTz  = 2.0f*this->z;
  double fTwx = fTx*this->u;
  double fTwz = fTz*this->u;
  double fTxx = fTx*this->x;
  double fTxy = fTy*this->x;
  double fTyz = fTz*this->y;
  double fTzz = fTz*this->z;

  return Vector3(fTxy-fTwz, 1.0f-(fTxx+fTzz), fTyz+fTwx);
}

Vector3 Quatern::GetZAxis() const
{
  double fTx  = 2.0f*this->x;
  double fTy  = 2.0f*this->y;
  double fTz  = 2.0f*this->z;
  double fTwx = fTx*this->u;
  double fTwy = fTy*this->u;
  double fTxx = fTx*this->x;
  double fTxz = fTz*this->x;
  double fTyy = fTy*this->y;
  double fTyz = fTz*this->y;

  return Vector3(fTxz+fTwy, fTyz-fTwx, 1.0f-(fTxx+fTyy));
}

