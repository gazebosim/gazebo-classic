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

#include "math/Pose.hh"

using namespace gazebo;
using namespace math;


////////////////////////////////////////////////////////////////////////////////
/// Default constructors
Pose::Pose()
{
}

////////////////////////////////////////////////////////////////////////////////
/// Constructor
Pose::Pose( const Vector3 &pos, const Quaternion &rot)
{
  this->pos = pos;
  this->rot = rot;
}

////////////////////////////////////////////////////////////////////////////////
/// Copy constructor
Pose::Pose( const Pose &pose )
{
  this->pos = pose.pos;
  this->rot = pose.rot;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
Pose::~Pose()
{
}

////////////////////////////////////////////////////////////////////////////////
// See if a pose is finite (e.g., not nan)
bool Pose::IsFinite() const
{
  return this->pos.IsFinite() && this->rot.IsFinite();
}

////////////////////////////////////////////////////////////////////////////////
/// Fix any nan values
void Pose::Correct()
{
  this->pos.Correct();
  this->rot.Correct();
}

////////////////////////////////////////////////////////////////////////////////
/// Get the inverse of this pose
Pose Pose::GetInverse() const
{
  Quaternion inv = this->rot.GetInverse();
  return Pose( inv * (this->pos*-1), inv );
}

////////////////////////////////////////////////////////////////////////////////
// Add two poses: result = this + obj
Pose Pose::operator+(const Pose &obj) const
{
  Pose result;

  result.pos = this->CoordPositionAdd(obj);
  result.rot = this->CoordRotationAdd(obj.rot);

  return result;
}

////////////////////////////////////////////////////////////////////////////////
/// Add-Equals operator
const Pose &Pose::operator+=(const Pose &obj)
{
  this->pos = this->CoordPositionAdd(obj);
  this->rot = this->CoordRotationAdd(obj.rot);

  return *this;
}

////////////////////////////////////////////////////////////////////////////////
// Add two poses: result = this - obj
Pose Pose::operator-(const Pose &obj) const
{
  Pose result;

  result.pos = this->CoordPositionSub(obj);
  result.rot = this->CoordRotationSub(obj.rot);

  return result;
}

////////////////////////////////////////////////////////////////////////////////
// Add two poses: result = this - obj
const Pose &Pose::operator-=(const Pose &_obj)
{
  this->pos = this->CoordPositionSub(_obj);
  this->rot = this->CoordRotationSub(_obj.rot);

  return *this;
}

////////////////////////////////////////////////////////////////////////////////
bool Pose::operator==(const Pose &_pose) const
{
  return this->pos == _pose.pos && this->rot == _pose.rot;
}

////////////////////////////////////////////////////////////////////////////////
bool Pose::operator!=(const Pose &_pose) const
{
  return this->pos != _pose.pos || this->rot != _pose.rot;
}
 
////////////////////////////////////////////////////////////////////////////////
/// Multiplication operator
Pose Pose::operator*(const Pose &pose)
{
  return Pose(this->CoordPositionAdd(pose), this->rot * pose.rot);
}

////////////////////////////////////////////////////////////////////////////////
// Add one point to a vector: result = this + pos
Vector3 Pose::CoordPositionAdd(const Vector3 &pos) const
{
  Quaternion tmp;
  Vector3 result;

  // result = pose.rot + pose.rot * this->pos * pose.rot!
  tmp.w = 0.0;
  tmp.x = pos.x;
  tmp.y = pos.y;
  tmp.z = pos.z;

  tmp = this->rot * (tmp * this->rot.GetInverse());
  result.x = this->pos.x + tmp.x;
  result.y = this->pos.y + tmp.y;
  result.z = this->pos.z + tmp.z;

  return result;
}

////////////////////////////////////////////////////////////////////////////////
// Add one point to another: result = this + pose
Vector3 Pose::CoordPositionAdd(const Pose &pose) const
{
  Quaternion tmp;
  Vector3 result;

  // result = pose.rot + pose.rot * this->pos * pose.rot!
  tmp.w = 0.0;
  tmp.x = this->pos.x;
  tmp.y = this->pos.y;
  tmp.z = this->pos.z;

  //tmp = (tmp * pose.rot) * pose.rot.GetInverse();
  tmp = pose.rot * (tmp * pose.rot.GetInverse());

  result.x = pose.pos.x + tmp.x;
  result.y = pose.pos.y + tmp.y;
  result.z = pose.pos.z + tmp.z;

  return result;
}

////////////////////////////////////////////////////////////////////////////////
// Subtract one position from another: result = this - pose
Vector3 Pose::CoordPositionSub(const Pose &pose) const
{
  Quaternion tmp;
  Vector3 result;

  // result = pose.rot! * (this->pos - pose.pos) * pose.rot
  tmp.x = (this->pos - pose.pos).x;
  tmp.y = (this->pos - pose.pos).y;
  tmp.z = (this->pos - pose.pos).z;
  tmp.w = 0.0;

  tmp = pose.rot.GetInverse() * (tmp * pose.rot);

  result.x = tmp.x;
  result.y = tmp.y;
  result.z = tmp.z;

  return result;
}

////////////////////////////////////////////////////////////////////////////////
// Add one rotation to another: result =  this->rot + rot
Quaternion Pose::CoordRotationAdd(const Quaternion &rot) const
{
  return Quaternion(rot * this->rot);
}

////////////////////////////////////////////////////////////////////////////////
// Subtract one rotation from another: result = this->rot - rot
Quaternion Pose::CoordRotationSub(const Quaternion &rot) const
{
  Quaternion result(rot.GetInverse() * this->rot);
  result.Normalize();
  return result;
}

////////////////////////////////////////////////////////////////////////////////
/// Reset the pose
void Pose::Reset()
{
  // set the position to zero
  this->pos.Set();
  this->rot.SetToIdentity();
}


//////////////////////////////////////////////////////////////////////////////
// Find the inverse of a pose; i.e., if b = this + a, given b and this,
// find a
Pose Pose::CoordPoseSolve(const Pose &b) const
{
  Quaternion q;
  Pose a;

  a.rot = this->rot.GetInverse() * b.rot;
  q = a.rot * Quaternion(0, this->pos.x, this->pos.y, this->pos.z);
  q = q * a.rot.GetInverse();
  a.pos = b.pos - Vector3(q.x, q.y, q.z);

  return a;
}

//////////////////////////////////////////////////////////////////////////////
// Rotate a vector by a quaternion, added for computing CG location for the Body
Pose Pose::RotatePositionAboutOrigin(const Quaternion &rot) const
{
  Pose a = *this;
  a.pos.x =  (1.0 - 2.0*rot.y*rot.y - 2.0*rot.z*rot.z) * this->pos.x
            +(2.0*(rot.x*rot.y+rot.w*rot.z)          ) * this->pos.y
            +(2.0*(rot.x*rot.z-rot.w*rot.y)          ) * this->pos.z;
  a.pos.y =  (2.0*(rot.x*rot.y-rot.w*rot.z)          ) * this->pos.x
            +(1.0 - 2.0*rot.x*rot.x - 2.0*rot.z*rot.z) * this->pos.y
            +(2.0*(rot.y*rot.z+rot.w*rot.x)          ) * this->pos.z;
  a.pos.z =  (2.0*(rot.x*rot.z+rot.w*rot.y)          ) * this->pos.x
            +(2.0*(rot.y*rot.z-rot.w*rot.x)          ) * this->pos.y
            +(1.0 - 2.0*rot.x*rot.x - 2.0*rot.y*rot.y) * this->pos.z;
  return a;
}
