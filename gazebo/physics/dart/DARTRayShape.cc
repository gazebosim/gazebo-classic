/*
 * Copyright 2012-2013 Open Source Robotics Foundation
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

#include "gazebo/common/Assert.hh"

#include "gazebo/physics/World.hh"
#include "gazebo/physics/Link.hh"
#include "gazebo/physics/dart/DARTPhysics.hh"
#include "gazebo/physics/dart/DARTTypes.hh"
#include "gazebo/physics/dart/DARTLink.hh"
#include "gazebo/physics/dart/DARTCollision.hh"
#include "gazebo/physics/dart/DARTRayShape.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
DARTRayShape::DARTRayShape(PhysicsEnginePtr _physicsEngine)
  : RayShape(_physicsEngine)
{
  this->SetName("DART Ray Shape");

  this->physicsEngine =
    boost::static_pointer_cast<DARTPhysics>(_physicsEngine);
}

//////////////////////////////////////////////////
DARTRayShape::DARTRayShape(CollisionPtr _parent)
    : RayShape(_parent)
{
}

//////////////////////////////////////////////////
DARTRayShape::~DARTRayShape()
{
}

//////////////////////////////////////////////////
void DARTRayShape::Update()
{
  if (this->collisionParent)
  {
    DARTCollisionPtr collision =
        boost::static_pointer_cast<DARTCollision>(this->collisionParent);

    LinkPtr link = this->collisionParent->GetLink();
    GZ_ASSERT(link != NULL, "DART link is NULL");

    this->globalStartPos = link->GetWorldPose().CoordPositionAdd(
          this->relativeStartPos);

    this->globalEndPos = link->GetWorldPose().CoordPositionAdd(
          this->relativeEndPos);
  }

  Eigen::Vector3d start(this->globalStartPos.x, this->globalStartPos.y,
      this->globalStartPos.z);
  Eigen::Vector3d end(this->globalEndPos.x, this->globalEndPos.y,
      this->globalEndPos.z);

//  btCollisionWorld::ClosestRayResultCallback rayCallback(start, end);
//  rayCallback.m_collisionFilterGroup = GZ_SENSOR_COLLIDE;
//  rayCallback.m_collisionFilterMask = ~GZ_SENSOR_COLLIDE;

  boost::recursive_mutex::scoped_lock lock(
      *this->physicsEngine->GetPhysicsUpdateMutex());

//  this->physicsEngine->GetDynamicsWorld()->rayTest(
//      start, end, rayCallback);

//  if (rayCallback.hasHit())
//  {
//    math::Vector3 result(rayCallback.m_hitPointWorld.getX(),
//                         rayCallback.m_hitPointWorld.getY(),
//                         rayCallback.m_hitPointWorld.getZ());
//    this->SetLength(this->globalStartPos.Distance(result));
//  }
}

//////////////////////////////////////////////////
void DARTRayShape::GetIntersection(double& _dist, std::string& _entity)
{
  _dist = 0;
  _entity = "";

  if (this->physicsEngine)
  {
    Eigen::Vector3d start(this->globalStartPos.x, this->globalStartPos.y,
        this->globalStartPos.z);
    Eigen::Vector3d end(this->globalEndPos.x, this->globalEndPos.y,
        this->globalEndPos.z);

    // btCollisionWorld::ClosestRayResultCallback rayCallback(start, end);
    // rayCallback.m_collisionFilterGroup = GZ_SENSOR_COLLIDE;
    // rayCallback.m_collisionFilterMask = ~GZ_SENSOR_COLLIDE;
    // this->physicsEngine->GetDynamicsWorld()->rayTest(
    //    start, end, rayCallback);
//    if (rayCallback.hasHit())
//    {
//      math::Vector3 result(rayCallback.m_hitPointWorld.getX(),
//                           rayCallback.m_hitPointWorld.getY(),
//                           rayCallback.m_hitPointWorld.getZ());
//      _dist = this->globalStartPos.Distance(result);

//      DARTLink* link = static_cast<DARTLink*>(
//          rayCallback.m_collisionObject->getUserPointer());
//      GZ_ASSERT(link != NULL, "DART link is NULL");
//      _entity = link->GetScopedName();
//    }
  }
}

//////////////////////////////////////////////////
void DARTRayShape::SetPoints(const math::Vector3& _posStart,
                             const math::Vector3& _posEnd)
{
  RayShape::SetPoints(_posStart, _posEnd);
}
