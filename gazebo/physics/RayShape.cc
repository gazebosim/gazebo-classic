/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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
#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
  #include <Winsock2.h>
#endif
#include "gazebo/physics/Collision.hh"

#include "gazebo/physics/RayShapePrivate.hh"
#include "gazebo/physics/RayShape.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
RayShape::RayShape(PhysicsEnginePtr /*_physicsEngine*/)
: Shape(*new RayShapePrivate, CollisionPtr()),
  rayShapeDPtr(static_cast<RayShapePrivate*>(this->shapeDPtr))
{
  this->AddType(RAY_SHAPE);
  this->SetName("Ray");

  this->rayShapeDPtr->contactLen = GZ_DBL_MAX;
  this->rayShapeDPtr->contactRetro = 0.0;
  this->rayShapeDPtr->contactFiducial = -1;
}

//////////////////////////////////////////////////
RayShape::RayShape(CollisionPtr _parent)
  : Shape(_parent)
{
  this->AddType(RAY_SHAPE);
  this->SetName("Ray");

  this->rayShapeDPtr->contactLen = GZ_DBL_MAX;
  this->rayShapeDPtr->contactRetro = 0.0;
  this->rayShapeDPtr->contactFiducial = -1;

  if (this->rayShapeDPtr->collisionParent)
    this->rayShapeDPtr->collisionParent->SetSaveable(false);
}

//////////////////////////////////////////////////
RayShape::~RayShape()
{
}

//////////////////////////////////////////////////
void RayShape::SetPoints(const math::Vector3 &_posStart,
                         const math::Vector3 &_posEnd)
{
  this->SetPoints(_posStart.Ign(), _posEnd.Ign());
}

//////////////////////////////////////////////////
void RayShape::SetPoints(const ignition::math::Vector3d &_posStart,
                         const ignition::math::Vector3d &_posEnd)
{
  ignition::math::Vector3d dir;

  this->rayShapeDPtr->relativeStartPos = _posStart;
  this->rayShapeDPtr->relativeEndPos = _posEnd;

  if (this->rayShapeDPtr->collisionParent)
  {
    this->rayShapeDPtr->globalStartPos =
      this->rayShapeDPtr->collisionParent->WorldPose().CoordPositionAdd(
        this->rayShapeDPtr->relativeStartPos);
    this->rayShapeDPtr->globalEndPos =
      this->rayShapeDPtr->collisionParent->WorldPose().CoordPositionAdd(
        this->rayShapeDPtr->relativeEndPos);
  }
  else
  {
    this->rayShapeDPtr->globalStartPos = this->rayShapeDPtr->relativeStartPos;
    this->rayShapeDPtr->globalEndPos = this->rayShapeDPtr->relativeEndPos;
  }

  // Compute the direction of the ray
  dir = this->rayShapeDPtr->globalEndPos - this->rayShapeDPtr->globalStartPos;
  dir.Normalize();
}

//////////////////////////////////////////////////
void RayShape::GetRelativePoints(math::Vector3 &_posA, math::Vector3 &_posB)
{
  ignition::math::Vector3d a, b;
  this->RelativePoints(a, b);
  _posA = a;
  _posB = b;
}

//////////////////////////////////////////////////
void RayShape::RelativePoints(ignition::math::Vector3d &_posA,
    ignition::math::Vector3d &_posB)
{
  _posA = this->rayShapeDPtr->relativeStartPos;
  _posB = this->rayShapeDPtr->relativeEndPos;
}

//////////////////////////////////////////////////
void RayShape::GetGlobalPoints(math::Vector3 &_posA, math::Vector3 &_posB)
{
  ignition::math::Vector3d a, b;
  this->GlobalPoints(a, b);
  _posA = a;
  _posB = b;
}

//////////////////////////////////////////////////
void RayShape::GlobalPoints(ignition::math::Vector3d &_posA,
                            ignition::math::Vector3d &_posB)
{
  _posA = this->rayShapeDPtr->globalStartPos;
  _posB = this->rayShapeDPtr->globalEndPos;
}

//////////////////////////////////////////////////
void RayShape::SetLength(const double _len)
{
  this->rayShapeDPtr->contactLen = _len;

  ignition::math::Vector3d dir = this->rayShapeDPtr->relativeEndPos -
    this->rayShapeDPtr->relativeStartPos;
  dir.Normalize();

  this->rayShapeDPtr->relativeEndPos = dir * _len +
    this->rayShapeDPtr->relativeStartPos;
}

//////////////////////////////////////////////////
void RayShape::SetScale(const math::Vector3 &_scale)
{
  this->SetScale(_scale.Ign());
}

//////////////////////////////////////////////////
void RayShape::SetScale(const ignition::math::Vector3d &_scale)
{
  if (this->rayShapeDPtr->scale == _scale)
    return;

  this->rayShapeDPtr->scale = _scale;

  /// TODO RayShape::SetScale not yet implemented.
}

//////////////////////////////////////////////////
double RayShape::GetLength() const
{
  return this->Length();
}

//////////////////////////////////////////////////
double RayShape::Length() const
{
  return this->rayShapeDPtr->contactLen;
}

//////////////////////////////////////////////////
void RayShape::SetRetro(const float _retro)
{
  this->rayShapeDPtr->contactRetro = _retro;
}

//////////////////////////////////////////////////
float RayShape::GetRetro() const
{
  return this->Retro();
}

//////////////////////////////////////////////////
float RayShape::Retro() const
{
  return this->rayShapeDPtr->contactRetro;
}

//////////////////////////////////////////////////
void RayShape::SetFiducial(const int _fid)
{
  this->rayShapeDPtr->contactFiducial = _fid;
}

//////////////////////////////////////////////////
int RayShape::GetFiducial() const
{
  return this->Fiducial();
}

//////////////////////////////////////////////////
int RayShape::Fiducial() const
{
  return this->rayShapeDPtr->contactFiducial;
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

//////////////////////////////////////////////////
double RayShape::ComputeVolume() const
{
  return 0;
}

//////////////////////////////////////////////////
void RayShape::GetIntersection(double &_dist, std::string &_entity)
{
  this->Intersection(_dist, _entity);
}

//////////////////////////////////////////////////
ignition::math::Vector3d RayShape::Start() const
{
  return this->rayShapeDPtr->relativeStartPos;
}

//////////////////////////////////////////////////
ignition::math::Vector3d RayShape::End() const
{
  return this->rayShapeDPtr->relativeEndPos;
}

//////////////////////////////////////////////////
void RayShape::SetCollisionName(const std::string &_name)
{
  this->rayShapeDPtr->collisionName = _name;
}

//////////////////////////////////////////////////
std::string RayShape::CollisionName() const
{
  return this->rayShapeDPtr->collisionName;
}
