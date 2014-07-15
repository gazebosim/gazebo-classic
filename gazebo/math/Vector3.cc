/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
/* Desc: Vector 3
 * Author: Andrew Howard and Nate Koenig
 * Date: 4 Apr 2007
 */

#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

#include "gazebo/math/Helpers.hh"
#include "gazebo/math/Vector3.hh"

using namespace gazebo;
using namespace math;

const Vector3 Vector3::Zero = math::Vector3(0, 0, 0);
const Vector3 Vector3::One = math::Vector3(1, 1, 1);
const Vector3 Vector3::UnitX = math::Vector3(1, 0, 0);
const Vector3 Vector3::UnitY = math::Vector3(0, 1, 0);
const Vector3 Vector3::UnitZ = math::Vector3(0, 0, 1);

//////////////////////////////////////////////////
Vector3::Vector3()
{
  this->data[0] = 0.0;
  this->data[1] = 0.0;
  this->data[2] = 0.0;
}

//////////////////////////////////////////////////
Vector3::Vector3(const double &_x, const double &_y, const double &_z)
{
  this->data[0] = _x;
  this->data[1] = _y;
  this->data[2] = _z;
}

//////////////////////////////////////////////////
Vector3::Vector3(const Vector3 &_pt)
{
  this->data[0] = _pt.X();
  this->data[1] = _pt.Y();
  this->data[2] = _pt.Z();
}

//////////////////////////////////////////////////
Vector3::~Vector3()
{
}

//////////////////////////////////////////////////
double Vector3::Distance(const Vector3 &_pt) const
{
  return sqrt((this->X()-_pt.X())*(this->X()-_pt.X()) +
              (this->Y()-_pt.Y())*(this->Y()-_pt.Y()) +
              (this->Z()-_pt.Z())*(this->Z()-_pt.Z()));
}

//////////////////////////////////////////////////
double Vector3::Distance(double _x, double _y, double _z) const
{
  return this->Distance(Vector3(_x, _y, _z));
}

//////////////////////////////////////////////////
double Vector3::GetSum() const
{
  return this->X() + this->Y() + this->Z();
}

//////////////////////////////////////////////////
double Vector3::GetLength() const
{
  return sqrt(this->X() * this->X() + this->Y() * this->Y() + this->Z() * this->Z());
}

//////////////////////////////////////////////////
double Vector3::GetSquaredLength() const
{
  return this->X() * this->X() + this->Y() * this->Y() + this->Z() * this->Z();
}

//////////////////////////////////////////////////
Vector3 Vector3::Normalize()
{
  double d = sqrt(this->X() * this->X() + this->Y() * this->Y() + this->Z() * this->Z());

  if (!math::equal(d, 0.0))
  {
    this->X() /= d;
    this->Y() /= d;
    this->Z() /= d;
  }

  return *this;
}

//////////////////////////////////////////////////
Vector3 Vector3::Round()
{
  this->X() = nearbyint(this->X());
  this->Y() = nearbyint(this->Y());
  this->Z() = nearbyint(this->Z());
  return *this;
}

//////////////////////////////////////////////////
Vector3 Vector3::GetRounded() const
{
  Vector3 result = *this;
  result.Round();
  return result;
}

//////////////////////////////////////////////////
Vector3 Vector3::Cross(const Vector3 &_pt) const
{
  Vector3 c(0, 0, 0);

  c.X() = this->Y() * _pt.Z() - this->Z() * _pt.Y();
  c.Y() = this->Z() * _pt.X() - this->X() * _pt.Z();
  c.Z() = this->X() * _pt.Y() - this->Y() * _pt.X();

  return c;
}

//////////////////////////////////////////////////
double Vector3::Dot(const Vector3 &_pt) const
{
  return this->X() * _pt.X() + this->Y() * _pt.Y() + this->Z() * _pt.Z();
}

//////////////////////////////////////////////////
Vector3 Vector3::GetAbs() const
{
  return Vector3(fabs(this->X()), fabs(this->Y()), fabs(this->Z()));
}

//////////////////////////////////////////////////
Vector3 Vector3::GetPerpendicular() const
{
  static const double sqrZero = 1e-06 * 1e-06;

  Vector3 perp = this->Cross(Vector3(1, 0, 0));

  // Check the length of the vector
  if (perp.GetSquaredLength() < sqrZero)
  {
    perp = this->Cross(Vector3(0, 1, 0));
  }

  return perp;
}

//////////////////////////////////////////////////
Vector3 Vector3::GetNormal(const Vector3 &v1, const Vector3 &v2,
                           const Vector3 &v3)
{
  Vector3 a = v2 - v1;
  Vector3 b = v3 - v1;
  Vector3 n = a.Cross(b);
  return n;
}

//////////////////////////////////////////////////
double Vector3::GetDistToLine(const Vector3 &_pt1, const Vector3 &_pt2)
{
  double d = ((*this) - _pt1).Cross((*this) - _pt2).GetLength();
  d = d / (_pt2 - _pt1).GetLength();
  return d;
}

//////////////////////////////////////////////////
void Vector3::SetToMax(const Vector3 & _v)
{
  if (_v.X() > this->X()) this->X() = _v.X();
  if (_v.Y() > this->Y()) this->Y() = _v.Y();
  if (_v.Z() > this->Z()) this->Z() = _v.Z();
}

//////////////////////////////////////////////////
void Vector3::SetToMin(const Vector3 & _v)
{
  if (_v.X() < this->X()) this->X() = _v.X();
  if (_v.Y() < this->Y()) this->Y() = _v.Y();
  if (_v.Z() < this->Z()) this->Z() = _v.Z();
}

//////////////////////////////////////////////////
double Vector3::GetMax() const
{
  return std::max(std::max(this->X(), this->Y()), this->Z());
}

//////////////////////////////////////////////////
double Vector3::GetMin() const
{
  return std::min(std::min(this->X(), this->Y()), this->Z());
}

//////////////////////////////////////////////////
Vector3 &Vector3::operator =(const Vector3 &_pt)
{
  this->X() = _pt.X();
  this->Y() = _pt.Y();
  this->Z() = _pt.Z();

  return *this;
}

//////////////////////////////////////////////////
Vector3 &Vector3::operator =(double value)
{
  this->X() = value;
  this->Y() = value;
  this->Z() = value;

  return *this;
}

//////////////////////////////////////////////////
Vector3 Vector3::operator+(const Vector3 &pt) const
{
  return Vector3(this->X() + pt.X(), this->Y() + pt.Y(), this->Z() + pt.Z());
}

//////////////////////////////////////////////////
const Vector3 &Vector3::operator+=(const Vector3 &pt)
{
  this->X() += pt.X();
  this->Y() += pt.Y();
  this->Z() += pt.Z();

  return *this;
}

//////////////////////////////////////////////////
const Vector3 &Vector3::operator-=(const Vector3 &pt)
{
  this->X() -= pt.X();
  this->Y() -= pt.Y();
  this->Z() -= pt.Z();

  return *this;
}

//////////////////////////////////////////////////
const Vector3 Vector3::operator/(const Vector3 &pt) const
{
  return Vector3(this->X() / pt.X(), this->Y() / pt.Y(), this->Z() / pt.Z());
}

//////////////////////////////////////////////////
const Vector3 &Vector3::operator/=(const Vector3 &pt)
{
  this->X() /= pt.X();
  this->Y() /= pt.Y();
  this->Z() /= pt.Z();

  return *this;
}

//////////////////////////////////////////////////
const Vector3 Vector3::operator/(double v) const
{
  return Vector3(this->X() / v, this->Y() / v, this->Z() / v);
}

//////////////////////////////////////////////////
const Vector3 &Vector3::operator/=(double v)
{
  this->X() /= v;
  this->Y() /= v;
  this->Z() /= v;

  return *this;
}

//////////////////////////////////////////////////
Vector3 Vector3::operator*(const Vector3 &pt) const
{
  return Vector3(this->X() * pt.X(), this->Y() * pt.Y(), this->Z() * pt.Z());
}

//////////////////////////////////////////////////
const Vector3 &Vector3::operator*=(const Vector3 &pt)
{
  this->X() *= pt.X();
  this->Y() *= pt.Y();
  this->Z() *= pt.Z();

  return *this;
}

//////////////////////////////////////////////////
Vector3 Vector3::operator*(double v) const
{
  return Vector3(this->X() * v, this->Y() * v, this->Z() * v);
}

//////////////////////////////////////////////////
const Vector3 &Vector3::operator*=(double v)
{
  this->X() *= v;
  this->Y() *= v;
  this->Z() *= v;

  return *this;
}

//////////////////////////////////////////////////
bool Vector3::operator ==(const Vector3 &_pt) const
{
  return equal(this->X(), _pt.X(), 0.001) &&
         equal(this->Y(), _pt.Y(), 0.001) &&
         equal(this->Z(), _pt.Z(), 0.001);
}

//////////////////////////////////////////////////
bool Vector3::operator!=(const Vector3 &_pt) const
{
  return !(*this == _pt);
}

//////////////////////////////////////////////////
bool Vector3::IsFinite() const
{
  return std::isfinite(this->X()) && std::isfinite(this->Y()) &&
         std::isfinite(this->Z());
}

//////////////////////////////////////////////////
double Vector3::operator[](unsigned int index) const
{
  switch (index)
  {
    case 0:
      return this->X();
    case 1:
      return this->Y();
    case 2:
      return this->Z();
    default:
      return 0;
  }
}

//////////////////////////////////////////////////
/// Round all values to _decimalPlaces
void Vector3::Round(int _precision)
{
  this->X() = precision(this->X(), _precision);
  this->Y() = precision(this->Y(), _precision);
  this->Z() = precision(this->Z(), _precision);
}

//////////////////////////////////////////////////
/// Returns true if the two vectors are exacatly equal
bool Vector3::Equal(const Vector3 &_v) const
{
  return math::equal(this->X(), _v.X()) &&
         math::equal(this->Y(), _v.Y()) &&
         math::equal(this->Z(), _v.Z());
}
