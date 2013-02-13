/*
 * Copyright 2012 Open Source Robotics Foundation
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
/* Desc: A ray
 * Author: Nate Koenig
 * Date: 24 May 2009
 */

#include "gazebo/physics/World.hh"
#include "gazebo/physics/bullet/BulletLink.hh"
#include "gazebo/physics/bullet/BulletPhysics.hh"
#include "gazebo/physics/bullet/BulletTypes.hh"
#include "gazebo/physics/bullet/BulletCollision.hh"
#include "gazebo/physics/bullet/BulletRayShape.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
BulletRayShape::BulletRayShape(PhysicsEnginePtr _physicsEngine)
  : RayShape(_physicsEngine)
{
  this->SetName("Bullet Ray Shape");

  this->physicsEngine =
    boost::shared_static_cast<BulletPhysics>(_physicsEngine);
}

//////////////////////////////////////////////////
BulletRayShape::BulletRayShape(CollisionPtr _parent)
    : RayShape(_parent)
{
  this->SetName("Bullet Ray Shape");
  this->physicsEngine = boost::shared_static_cast<BulletPhysics>(
      this->collisionParent->GetWorld()->GetPhysicsEngine());
}

//////////////////////////////////////////////////
BulletRayShape::~BulletRayShape()
{
}

//////////////////////////////////////////////////
void BulletRayShape::Update()
{
  if (this->collisionParent)
  {
    BulletCollisionPtr collision =
      boost::shared_static_cast<BulletCollision>(this->collisionParent);

    this->globalStartPos =
      this->collisionParent->GetLink()->GetWorldPose().CoordPositionAdd(
          this->relativeStartPos);

    this->globalEndPos =
      this->collisionParent->GetLink()->GetWorldPose().CoordPositionAdd(
          this->relativeEndPos);
  }

  btVector3 start(this->globalStartPos.x, this->globalStartPos.y,
      this->globalStartPos.z);
  btVector3 end(this->globalEndPos.x, this->globalEndPos.y,
      this->globalEndPos.z);


//  gzerr << "start " << globalStartPos.x <<  " " << this->globalStartPos.y << " " << this->globalStartPos.z <<std::endl;
//    gzerr << "end " << globalEndPos.x <<  " " << this->globalEndPos.y << " " << this->globalEndPos.z <<std::endl;

  btCollisionWorld::ClosestRayResultCallback rayCallback(start, end);
  this->physicsEngine->GetPhysicsUpdateMutex()->lock();
  this->physicsEngine->GetDynamicsWorld()->rayTest(
      start, end, rayCallback);

  if (rayCallback.hasHit())
  {
    math::Vector3 result(rayCallback.m_hitPointWorld.getX(),
                         rayCallback.m_hitPointWorld.getY(),
                         rayCallback.m_hitPointWorld.getZ());
    this->SetLength(this->globalStartPos.Distance(result));
  }
  this->physicsEngine->GetPhysicsUpdateMutex()->unlock();
}

//////////////////////////////////////////////////
void BulletRayShape::GetIntersection(double &_dist, std::string &_entity)
{
  _dist = 0;
  _entity = "";

  if (this->physicsEngine && this->collisionParent)
  {
    btVector3 start(this->globalStartPos.x, this->globalStartPos.y,
        this->globalStartPos.z);
    btVector3 end(this->globalEndPos.x, this->globalEndPos.y,
        this->globalEndPos.z);

    btCollisionWorld::ClosestRayResultCallback rayCallback(start, end);
    this->physicsEngine->GetDynamicsWorld()->rayTest(
        start, end, rayCallback);
    if (rayCallback.hasHit())
    {
      math::Vector3 result(rayCallback.m_hitPointWorld.getX(),
                           rayCallback.m_hitPointWorld.getY(),
                           rayCallback.m_hitPointWorld.getZ());
      _dist = this->globalStartPos.Distance(result);
      _entity = static_cast<BulletLink*>(
          rayCallback.m_collisionObject->getUserPointer())->GetName();
    }
  }
}

//////////////////////////////////////////////////
void BulletRayShape::SetPoints(const math::Vector3 &_posStart,
                                   const math::Vector3 &_posEnd)
{
  RayShape::SetPoints(_posStart, _posEnd);
}
