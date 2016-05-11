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

#include <sstream>

#include "gazebo/common/Assert.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/math/Box.hh"

#include "gazebo/physics/Inertial.hh"
#include "gazebo/physics/ode/ODESurfaceParams.hh"
#include "gazebo/physics/ode/ODEPhysics.hh"
#include "gazebo/physics/ode/ODELink.hh"

#include "gazebo/physics/ode/ODECollisionPrivate.hh"
#include "gazebo/physics/ode/ODECollision.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
ODECollision::ODECollision(LinkPtr _link)
: Collision(*new ODECollisionPrivate, _link),
  odeCollisionDPtr(static_cast<ODECollisionPrivate*>(this->collDPtr))
{
  this->SetName("ODE_Collision");
  this->odeCollisionDPtr->collisionId = NULL;
  this->odeCollisionDPtr->onPoseChangeFunc = &ODECollision::OnPoseChangeNull;

  this->SetSpaceId(
      std::static_pointer_cast<ODELink>(this->odeCollisionDPtr->link)->SpaceId());

  this->odeCollisionDPtr->surface.reset(new ODESurfaceParams());
}

//////////////////////////////////////////////////
ODECollision::~ODECollision()
{
  if (this->odeCollisionDPtr->collisionId)
    dGeomDestroy(this->odeCollisionDPtr->collisionId);
  this->odeCollisionDPtr->collisionId = NULL;
}

//////////////////////////////////////////////////
void ODECollision::Load(sdf::ElementPtr _sdf)
{
  Collision::Load(_sdf);

  if (this->IsStatic())
  {
    this->SetCategoryBits(GZ_FIXED_COLLIDE);
    this->SetCollideBits(~GZ_FIXED_COLLIDE);
  }

  // Force max correcting velocity to zero for certain collision entities
  if (this->IsStatic() ||
      this->odeCollisionDPtr->shape->HasType(Base::HEIGHTMAP_SHAPE) ||
      this->odeCollisionDPtr->shape->HasType(Base::MAP_SHAPE))
  {
    this->ODESurface()->maxVel = 0.0;
  }
}

//////////////////////////////////////////////////
void ODECollision::Fini()
{
  /*
     if (this->odeCollisionDPtr->collisionId)
     dGeomDestroy(this->odeCollisionDPtr->collisionId);
     this->odeCollisionDPtr->collisionId = NULL;

     if (this->odeCollisionDPtr->spaceId)
     dSpaceDestroy(this->odeCollisionDPtr->spaceId);
     this->odeCollisionDPtr->spaceId = NULL;
     */

  Collision::Fini();
}

//////////////////////////////////////////////////
void ODECollision::OnPoseChange()
{
  // Update all the models
  // (*this.*onPoseChangeFunc)();

  if (this->IsStatic() && this->odeCollisionDPtr->collisionId &&
      this->odeCollisionDPtr->placeable)
  {
    this->OnPoseChangeGlobal();
  }
  else if (this->odeCollisionDPtr->collisionId && this->odeCollisionDPtr->placeable)
  {
    this->OnPoseChangeRelative();
  }
}

//////////////////////////////////////////////////
void ODECollision::SetCollision(dGeomID _collisionId, const bool _placeable)
{
  // Must go first in this function
  this->odeCollisionDPtr->collisionId = _collisionId;

  Collision::SetCollision(_placeable);

  if (dGeomGetSpace(this->odeCollisionDPtr->collisionId) == 0)
  {
    dSpaceAdd(this->odeCollisionDPtr->spaceId, this->odeCollisionDPtr->collisionId);
    GZ_ASSERT(dGeomGetSpace(this->odeCollisionDPtr->collisionId) != 0,
        "Collision ID is NULL");
  }

  if (this->odeCollisionDPtr->collisionId && this->odeCollisionDPtr->placeable)
  {
    if (this->IsStatic())
      this->odeCollisionDPtr->onPoseChangeFunc = &ODECollision::OnPoseChangeGlobal;
    else
      this->odeCollisionDPtr->onPoseChangeFunc = &ODECollision::OnPoseChangeRelative;
  }
  else
  {
    this->odeCollisionDPtr->onPoseChangeFunc = &ODECollision::OnPoseChangeNull;
  }

  dGeomSetData(this->odeCollisionDPtr->collisionId, this);
}

//////////////////////////////////////////////////
dGeomID ODECollision::GetCollisionId() const
{
  return this->CollisionId();
}

//////////////////////////////////////////////////
dGeomID ODECollision::CollisionId() const
{
  return this->odeCollisionDPtr->collisionId;
}

//////////////////////////////////////////////////
int ODECollision::GetCollisionClass() const
{
  return this->CollisionClass();
}

//////////////////////////////////////////////////
int ODECollision::CollisionClass() const
{
  int result = 0;

  if (this->odeCollisionDPtr->collisionId)
  {
    result = dGeomGetClass(this->odeCollisionDPtr->collisionId);
  }

  return result;
}

//////////////////////////////////////////////////
void ODECollision::SetCategoryBits(const unsigned int _bits)
{
  if (this->odeCollisionDPtr->collisionId)
    dGeomSetCategoryBits(this->odeCollisionDPtr->collisionId, _bits);
  if (this->odeCollisionDPtr->spaceId)
    dGeomSetCategoryBits((dGeomID)this->odeCollisionDPtr->spaceId, _bits);
}

//////////////////////////////////////////////////
void ODECollision::SetCollideBits(const unsigned int _bits)
{
  if (this->odeCollisionDPtr->collisionId)
    dGeomSetCollideBits(this->odeCollisionDPtr->collisionId, _bits);
  if (this->odeCollisionDPtr->spaceId)
    dGeomSetCollideBits((dGeomID)this->odeCollisionDPtr->spaceId, _bits);
}

//////////////////////////////////////////////////
ignition::math::Box ODECollision::BoundingBox() const
{
  ignition::math::Box box;
  dReal aabb[6];

  memset(aabb, 0, 6 * sizeof(dReal));

  // if (this->odeCollisionDPtr->collisionId && this->type != Shape::PLANE)
  dGeomGetAABB(this->odeCollisionDPtr->collisionId, aabb);

  box.Min().Set(aabb[0], aabb[2], aabb[4]);
  box.Max().Set(aabb[1], aabb[3], aabb[5]);

  return box;
}

//////////////////////////////////////////////////
dSpaceID ODECollision::GetSpaceId() const
{
  return this->SpaceId();
}

//////////////////////////////////////////////////
dSpaceID ODECollision::SpaceId() const
{
  return this->odeCollisionDPtr->spaceId;
}

//////////////////////////////////////////////////
void ODECollision::SetSpaceId(const dSpaceID _spaceid)
{
  this->odeCollisionDPtr->spaceId = _spaceid;
}

/////////////////////////////////////////////////
ODESurfaceParamsPtr ODECollision::GetODESurface() const
{
  return this->ODESurface();
}

/////////////////////////////////////////////////
ODESurfaceParamsPtr ODECollision::ODESurface() const
{
  return std::dynamic_pointer_cast<ODESurfaceParams>(
      this->odeCollisionDPtr->surface);
}

/////////////////////////////////////////////////
void ODECollision::OnPoseChangeGlobal()
{
  // A collision is not guaranteed to have a link (such as a standalone ray)
  if (!this->odeCollisionDPtr->link)
    return;

  dQuaternion q;

  // Transform into global pose since a static collision does not have a link
  ignition::math::Pose3d localPose = this->WorldPose();

  // un-offset cog location
  ignition::math::Vector3d cogVec =
    this->odeCollisionDPtr->link->Inertia().CoG();
  localPose.Pos() = localPose.Pos() - cogVec;

  q[0] = localPose.Rot().W();
  q[1] = localPose.Rot().X();
  q[2] = localPose.Rot().Y();
  q[3] = localPose.Rot().Z();

  dGeomSetPosition(this->odeCollisionDPtr->collisionId,
      localPose.Pos().X(), localPose.Pos().Y(), localPose.Pos().Z());
  dGeomSetQuaternion(this->odeCollisionDPtr->collisionId, q);
}

/////////////////////////////////////////////////
void ODECollision::OnPoseChangeRelative()
{
  // A collision is not guaranteed to have a link (such as a standalone ray)
  if (!this->odeCollisionDPtr->link)
    return;

  dQuaternion q;
  // Transform into CoM relative Pose
  ignition::math::Pose3d localPose = this->RelativePose();

  // un-offset cog location
  ignition::math::Vector3d cog_vec =
    this->odeCollisionDPtr->link->Inertia().CoG();
  localPose.Pos() = localPose.Pos() - cog_vec;

  q[0] = localPose.Rot().W();
  q[1] = localPose.Rot().X();
  q[2] = localPose.Rot().Y();
  q[3] = localPose.Rot().Z();

  // Set the pose of the encapsulated collision; this is always relative
  // to the CoM
  dGeomSetOffsetPosition(this->odeCollisionDPtr->collisionId,
      localPose.Pos().X(), localPose.Pos().Y(), localPose.Pos().Z());
  dGeomSetOffsetQuaternion(this->odeCollisionDPtr->collisionId, q);
}

/////////////////////////////////////////////////
void ODECollision::OnPoseChangeNull()
{
}
