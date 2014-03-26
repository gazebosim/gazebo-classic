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
/* Desc: A ray shape
 * Author: Nate Koenig
 * Date: 14 Oct 2009
 */

#include <vector>
#include <list>
#include <string>
#include <boost/enable_shared_from_this.hpp>
#include <boost/shared_ptr.hpp>

#include <sdf/sdf.hh>

#include "gazebo/transport/TransportTypes.hh"

#include "gazebo/math/Helpers.hh"

#include "gazebo/common/CommonTypes.hh"
#include "gazebo/common/Event.hh"

#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/physics/Collision.hh"
#include "gazebo/physics/RayShape.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
RayShape::RayShape(PhysicsEnginePtr /*_physicsEngine*/)
  : Shape(CollisionPtr())
{
  this->AddType(RAY_SHAPE);
  this->SetName("Ray");

  this->contactLen = GZ_DBL_MAX;
  this->contactRetro = 0.0;
  this->contactFiducial = -1;
}

//////////////////////////////////////////////////
RayShape::RayShape(CollisionPtr _parent)
  : Shape(_parent)
{
  this->AddType(RAY_SHAPE);
  this->SetName("Ray");

  this->contactLen = GZ_DBL_MAX;
  this->contactRetro = 0.0;
  this->contactFiducial = -1;

  if (collisionParent)
    this->collisionParent->SetSaveable(false);
}

//////////////////////////////////////////////////
RayShape::~RayShape()
{
}

//////////////////////////////////////////////////
void RayShape::SetPoints(const math::Vector3 &_posStart,
                         const math::Vector3 &_posEnd)
{
  math::Vector3 dir;

  this->relativeStartPos = _posStart;
  this->relativeEndPos = _posEnd;

  if (this->collisionParent)
  {
    this->globalStartPos =
      this->collisionParent->GetWorldPose().CoordPositionAdd(
        this->relativeStartPos);
    this->globalEndPos =
      this->collisionParent->GetWorldPose().CoordPositionAdd(
        this->relativeEndPos);
  }
  else
  {
    this->globalStartPos = this->relativeStartPos;
    this->globalEndPos = this->relativeEndPos;
  }

  // Compute the direction of the ray
  dir = this->globalEndPos - this->globalStartPos;
  dir.Normalize();
}

//////////////////////////////////////////////////
void RayShape::GetRelativePoints(math::Vector3 &_posA, math::Vector3 &_posB)
{
  _posA = this->relativeStartPos;
  _posB = this->relativeEndPos;
}

//////////////////////////////////////////////////
void RayShape::GetGlobalPoints(math::Vector3 &_posA, math::Vector3 &_posB)
{
  _posA = this->globalStartPos;
  _posB = this->globalEndPos;
}

//////////////////////////////////////////////////
void RayShape::SetLength(double _len)
{
  this->contactLen = _len;

  math::Vector3 dir = this->relativeEndPos - this->relativeStartPos;
  dir.Normalize();

  this->relativeEndPos = dir * _len + this->relativeStartPos;
}

//////////////////////////////////////////////////
void RayShape::SetScale(const math::Vector3 &_scale)
{
  if (this->scale == _scale)
    return;

  this->scale = _scale;

  /// TODO RayShape::SetScale not yet implemented.
}

//////////////////////////////////////////////////
double RayShape::GetLength() const
{
  return this->contactLen;
}

//////////////////////////////////////////////////
void RayShape::SetRetro(float _retro)
{
  this->contactRetro = _retro;
}

//////////////////////////////////////////////////
float RayShape::GetRetro() const
{
  return this->contactRetro;
}

//////////////////////////////////////////////////
void RayShape::SetFiducial(int _fid)
{
  this->contactFiducial = _fid;
}

//////////////////////////////////////////////////
int RayShape::GetFiducial() const
{
  return this->contactFiducial;
}

//////////////////////////////////////////////////
void RayShape::Init()
{
}

//////////////////////////////////////////////////
void RayShape::FillMsg(msgs::Geometry &/*_msg*/)
{
}

//////////////////////////////////////////////////
void RayShape::ProcessMsg(const msgs::Geometry &/*_msg*/)
{
}
