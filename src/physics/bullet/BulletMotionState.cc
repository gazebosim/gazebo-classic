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
/* Desc: Bullet motion state class.
 * Author: Nate Koenig 
 * Date: 25 May 2009
 */

#include "BulletPhysics.hh"
#include "rendering/Visual.hh"
#include "Body.hh"
#include "BulletMotionState.hh"

using namespace gazebo;
using namespace physics;

using namespace physics;

using namespace physics;


////////////////////////////////////////////////////////////////////////////////
// Constructor
BulletMotionState::BulletMotionState(Body *body)
  : btMotionState()
{
  this->body = body;
  this->visual = NULL;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
BulletMotionState::~BulletMotionState()
{
}

////////////////////////////////////////////////////////////////////////////////
// Set the visual node
void BulletMotionState::SetVisual(Visual *vis) 
{
  this->visual = vis;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the pose
common::Pose3d BulletMotionState::GetWorldPose() const
{
  return this->worldPose;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the position of the body
void BulletMotionState::SetWorldPosition(const common::Vector3 &pos)
{
  this->worldPose.pos = pos;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the rotation of the body
void BulletMotionState::SetWorldRotation(const common::Quatern &rot)
{
  this->worldPose.rot = rot;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the pose
void BulletMotionState::SetWorldPose(const common::Pose3d &pose)
{
  this->worldPose = pose;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the center of mass offset
void BulletMotionState::SetCoMOffset( const common::Pose3d &com )
{
  this->comOffset = com;
}

////////////////////////////////////////////////////////////////////////////////
// Get the world transform
void BulletMotionState::getWorldTransform(btTransform &worldTrans) const 
{
  common::Pose3d result = this->worldPose;
  result.pos += this->comOffset.pos;

  worldTrans = BulletPhysics::ConvertPose(result);
}

////////////////////////////////////////////////////////////////////////////////
// Set the world transform
void BulletMotionState::setWorldTransform(const btTransform &worldTrans) 
{
  if (this->visual == NULL)
    return;

  this->worldPose = BulletPhysics::ConvertPose( worldTrans );

  //std::cout << "New Pose[" << this->pose << "]\n";

  this->body->SetWorldPose(this->worldPose, false);
  //this->visual->SetDirty(true, this->worldPose - this->body->GetWorldPose());
}
