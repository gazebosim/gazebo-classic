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

#include "Pose3d.hh"

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
/// Default constructors
Pose3d::Pose3d()
{
}

////////////////////////////////////////////////////////////////////////////////
/// Constructor
Pose3d::Pose3d( const Vector3 &pos, const Quatern &rot)
{
  this->pos = pos;
  this->rot = rot;
}

////////////////////////////////////////////////////////////////////////////////
/// Copy constructor
Pose3d::Pose3d( const Pose3d &pose )
{
  this->pos = pose.pos;
  this->rot = pose.rot;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
Pose3d::~Pose3d()
{
}

////////////////////////////////////////////////////////////////////////////////
// See if a pose is finite (e.g., not nan)
bool Pose3d::IsFinite() const
{
  return this->pos.IsFinite() && this->rot.IsFinite();
}

////////////////////////////////////////////////////////////////////////////////
/// Fix any nan values
void Pose3d::Correct()
{
  this->pos.Correct();
  this->rot.Correct();
}

////////////////////////////////////////////////////////////////////////////////
/// Get the inverse of this pose
Pose3d Pose3d::GetInverse() const
{
  Quatern inv = this->rot.GetInverse();
  return Pose3d( inv * (this->pos*-1), inv );
}

////////////////////////////////////////////////////////////////////////////////
// Add two poses: result = this + obj
Pose3d Pose3d::operator+(const Pose3d &obj) const
{
  Pose3d result;

  result.pos = this->CoordPositionAdd(obj);
  result.rot = this->CoordRotationAdd(obj.rot);

  return result;
}

////////////////////////////////////////////////////////////////////////////////
/// Add-Equals operator
const Pose3d &Pose3d::operator+=(const Pose3d &obj)
{
  this->pos = this->CoordPositionAdd(obj);
  this->rot = this->CoordRotationAdd(obj.rot);

  return *this;
}

////////////////////////////////////////////////////////////////////////////////
// Add two poses: result = this - obj
Pose3d Pose3d::operator-(const Pose3d &obj) const
{
  Pose3d result;

  result.pos = this->CoordPositionSub(obj);
  result.rot = this->CoordRotationSub(obj.rot);

  return result;
}

////////////////////////////////////////////////////////////////////////////////
// Add two poses: result = this - obj
const Pose3d &Pose3d::operator-=(const Pose3d &obj)
{
  this->pos = this->CoordPositionSub(obj);
  this->rot = this->CoordRotationSub(obj.rot);

  return *this;
}

////////////////////////////////////////////////////////////////////////////////
/// Multiplication operator
Pose3d Pose3d::operator*(const Pose3d &pose)
{
  return Pose3d(this->CoordPositionAdd(pose), this->rot * pose.rot);
}

////////////////////////////////////////////////////////////////////////////////
// Add one point to a vector: result = this + pos
Vector3 Pose3d::CoordPositionAdd(const Vector3 &pos) const
{
  Quatern tmp;
  Vector3 result;

  // result = pose.rot + pose.rot * this->pos * pose.rot!
  tmp.u = 0.0;
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
Vector3 Pose3d::CoordPositionAdd(const Pose3d &pose) const
{
  Quatern tmp;
  Vector3 result;

  // result = pose.rot + pose.rot * this->pos * pose.rot!
  tmp.u = 0.0;
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
Vector3 Pose3d::CoordPositionSub(const Pose3d &pose) const
{
  Quatern tmp;
  Vector3 result;

  // result = pose.rot! * (this->pos - pose.pos) * pose.rot
  tmp.x = (this->pos - pose.pos).x;
  tmp.y = (this->pos - pose.pos).y;
  tmp.z = (this->pos - pose.pos).z;
  tmp.u = 0.0;

  tmp = pose.rot.GetInverse() * (tmp * pose.rot);

  result.x = tmp.x;
  result.y = tmp.y;
  result.z = tmp.z;

  return result;
}

////////////////////////////////////////////////////////////////////////////////
// Add one rotation to another: result =  this->rot + rot
Quatern Pose3d::CoordRotationAdd(const Quatern &rot) const
{
  return Quatern(rot * this->rot);
}

////////////////////////////////////////////////////////////////////////////////
// Subtract one rotation from another: result = this->rot - rot
Quatern Pose3d::CoordRotationSub(const Quatern &rot) const
{
  Quatern result(rot.GetInverse() * this->rot);
  result.Normalize();
  return result;
}

////////////////////////////////////////////////////////////////////////////////
/// Reset the pose
void Pose3d::Reset()
{
  // set the position to zero
  this->pos.Set();
  this->rot.SetToIdentity();
}


//////////////////////////////////////////////////////////////////////////////
// Find the inverse of a pose; i.e., if b = this + a, given b and this,
// find a
Pose3d Pose3d::CoordPoseSolve(const Pose3d &b) const
{
  Quatern q;
  Pose3d a;

  a.rot = this->rot.GetInverse() * b.rot;
  q = a.rot * Quatern(0, this->pos.x, this->pos.y, this->pos.z);
  q = q * a.rot.GetInverse();
  a.pos = b.pos - Vector3(q.x, q.y, q.z);

  return a;
}

//////////////////////////////////////////////////////////////////////////////
// Rotate a vector by a quaternion, added for computing CG location for the Body
Pose3d Pose3d::RotatePositionAboutOrigin(const Quatern &rot) const
{
  Pose3d a = *this;
  a.pos.x =  (1.0 - 2.0*rot.y*rot.y - 2.0*rot.z*rot.z) * this->pos.x
            +(2.0*(rot.x*rot.y+rot.u*rot.z)          ) * this->pos.y
            +(2.0*(rot.x*rot.z-rot.u*rot.y)          ) * this->pos.z;
  a.pos.y =  (2.0*(rot.x*rot.y-rot.u*rot.z)          ) * this->pos.x
            +(1.0 - 2.0*rot.x*rot.x - 2.0*rot.z*rot.z) * this->pos.y
            +(2.0*(rot.y*rot.z+rot.u*rot.x)          ) * this->pos.z;
  a.pos.z =  (2.0*(rot.x*rot.z+rot.u*rot.y)          ) * this->pos.x
            +(2.0*(rot.y*rot.z-rot.u*rot.x)          ) * this->pos.y
            +(1.0 - 2.0*rot.x*rot.x - 2.0*rot.y*rot.y) * this->pos.z;
  return a;
}
