/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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
  dataPtr(std::static_pointer_cast<ODECollisionPrivate>(this->baseDPtr))
{
  this->SetName("ODE_Collision");
  this->dataPtr->collisionId = NULL;
  this->dataPtr->onPoseChangeFunc = &ODECollision::OnPoseChangeNull;

  this->SetSpaceId(
      boost::static_pointer_cast<ODELink>(this->dataPtr->link)->GetSpaceId());

  this->dataPtr->surface.reset(new ODESurfaceParams());
}

//////////////////////////////////////////////////
ODECollision::~ODECollision()
{
  if (this->dataPtr->collisionId)
    dGeomDestroy(this->dataPtr->collisionId);
  this->dataPtr->collisionId = NULL;
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
      this->dataPtr->shape->HasType(Base::HEIGHTMAP_SHAPE) ||
      this->dataPtr->shape->HasType(Base::MAP_SHAPE))
  {
    this->ODESurface()->maxVel = 0.0;
  }
}

//////////////////////////////////////////////////
void ODECollision::Fini()
{
  /*
     if (this->dataPtr->collisionId)
     dGeomDestroy(this->dataPtr->collisionId);
     this->dataPtr->collisionId = NULL;

     if (this->dataPtr->spaceId)
     dSpaceDestroy(this->dataPtr->spaceId);
     this->dataPtr->spaceId = NULL;
     */

  Collision::Fini();
}

//////////////////////////////////////////////////
void ODECollision::OnPoseChange()
{
  // Update all the models
  // (*this.*onPoseChangeFunc)();

  if (this->IsStatic() && this->dataPtr->collisionId &&
      this->dataPtr->placeable)
  {
    this->OnPoseChangeGlobal();
  }
  else if (this->dataPtr->collisionId && this->dataPtr->placeable)
  {
    this->OnPoseChangeRelative();
  }
}

//////////////////////////////////////////////////
void ODECollision::SetCollision(dGeomID _collisionId, const bool _placeable)
{
  // Must go first in this function
  this->dataPtr->collisionId = _collisionId;

  Collision::SetCollision(_placeable);

  if (dGeomGetSpace(this->dataPtr->collisionId) == 0)
  {
    dSpaceAdd(this->dataPtr->spaceId, this->dataPtr->collisionId);
    GZ_ASSERT(dGeomGetSpace(this->dataPtr->collisionId) != 0,
        "Collision ID is NULL");
  }

  if (this->dataPtr->collisionId && this->dataPtr->placeable)
  {
    if (this->IsStatic())
      this->dataPtr->onPoseChangeFunc = &ODECollision::OnPoseChangeGlobal;
    else
      this->dataPtr->onPoseChangeFunc = &ODECollision::OnPoseChangeRelative;
  }
  else
  {
    this->dataPtr->onPoseChangeFunc = &ODECollision::OnPoseChangeNull;
  }

  dGeomSetData(this->dataPtr->collisionId, this);
}

//////////////////////////////////////////////////
dGeomID ODECollision::GetCollisionId() const
{
  return this->CollisionId();
}

//////////////////////////////////////////////////
dGeomID ODECollision::CollisionId() const
{
  return this->dataPtr->collisionId;
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

  if (this->dataPtr->collisionId)
  {
    result = dGeomGetClass(this->dataPtr->collisionId);
  }

  return result;
}

//////////////////////////////////////////////////
void ODECollision::SetCategoryBits(const unsigned int _bits)
{
  if (this->dataPtr->collisionId)
    dGeomSetCategoryBits(this->dataPtr->collisionId, _bits);
  if (this->dataPtr->spaceId)
    dGeomSetCategoryBits((dGeomID)this->dataPtr->spaceId, _bits);
}

//////////////////////////////////////////////////
void ODECollision::SetCollideBits(const unsigned int _bits)
{
  if (this->dataPtr->collisionId)
    dGeomSetCollideBits(this->dataPtr->collisionId, _bits);
  if (this->dataPtr->spaceId)
    dGeomSetCollideBits((dGeomID)this->dataPtr->spaceId, _bits);
}

//////////////////////////////////////////////////
ignition::math::Box ODECollision::BoundingBox() const
{
  ignition::math::Box box;
  dReal aabb[6];

  memset(aabb, 0, 6 * sizeof(dReal));

  // if (this->dataPtr->collisionId && this->type != Shape::PLANE)
  dGeomGetAABB(this->dataPtr->collisionId, aabb);

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
  return this->dataPtr->spaceId;
}

//////////////////////////////////////////////////
void ODECollision::SetSpaceId(const dSpaceID _spaceid)
{
  this->dataPtr->spaceId = _spaceid;
}

/////////////////////////////////////////////////
ODESurfaceParamsPtr ODECollision::GetODESurface() const
{
  return this->ODESurface();
}

/////////////////////////////////////////////////
ODESurfaceParamsPtr ODECollision::ODESurface() const
{
  return boost::dynamic_pointer_cast<ODESurfaceParams>(this->dataPtr->surface);
}

/////////////////////////////////////////////////
void ODECollision::OnPoseChangeGlobal()
{
  dQuaternion q;

  // Transform into global pose since a static collision does not have a link
  ignition::math::Pose3d localPose = this->WorldPose();

  // un-offset cog location
  ignition::math::Vector3d cogVec =
    this->dataPtr->link->Inertial()->GetCoG().Ign();
  localPose.Pos() = localPose.Pos() - cogVec;

  q[0] = localPose.Rot().W();
  q[1] = localPose.Rot().X();
  q[2] = localPose.Rot().Y();
  q[3] = localPose.Rot().Z();

  dGeomSetPosition(this->dataPtr->collisionId,
      localPose.Pos().X(), localPose.Pos().Y(), localPose.Pos().Z());
  dGeomSetQuaternion(this->dataPtr->collisionId, q);
}

/////////////////////////////////////////////////
void ODECollision::OnPoseChangeRelative()
{
  dQuaternion q;
  // Transform into CoM relative Pose
  ignition::math::Pose3d localPose = this->RelativePose();

  // un-offset cog location
  ignition::math::Vector3d cog_vec =
    this->dataPtr->link->Inertial()->GetCoG().Ign();
  localPose.Pos() = localPose.Pos() - cog_vec;

  q[0] = localPose.Rot().W();
  q[1] = localPose.Rot().X();
  q[2] = localPose.Rot().Y();
  q[3] = localPose.Rot().Z();

  // Set the pose of the encapsulated collision; this is always relative
  // to the CoM
  dGeomSetOffsetPosition(this->dataPtr->collisionId,
      localPose.Pos().X(), localPose.Pos().Y(), localPose.Pos().Z());
  dGeomSetOffsetQuaternion(this->dataPtr->collisionId, q);
}

/////////////////////////////////////////////////
void ODECollision::OnPoseChangeNull()
{
}
