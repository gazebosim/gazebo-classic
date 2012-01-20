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
/*
#include "BulletPhysics.hh"
#include "rendering/Visual.hh"
#include "Link.hh"
#include "BulletMotionState.hh"
*/

using namespace gazebo;
using namespace physics;


//////////////////////////////////////////////////
BulletMotionState::BulletMotionState(Link *_body)
  : btMotionState()
{
  this->body = _body;
  this->visual = NULL;
}

//////////////////////////////////////////////////
BulletMotionState::~BulletMotionState()
{
}

//////////////////////////////////////////////////
void BulletMotionState::SetVisual(Visual *_vis)
{
  this->visual = _vis;
}

//////////////////////////////////////////////////
math::Pose BulletMotionState::GetWorldPose() const
{
  return this->worldPose;
}

//////////////////////////////////////////////////
void BulletMotionState::SetWorldPosition(const math::Vector3 &_pos)
{
  this->worldPose._pos = _pos;
}

//////////////////////////////////////////////////
void BulletMotionState::SetWorldRotation(const common::Quatern &_rot)
{
  this->worldPose._rot = _rot;
}

//////////////////////////////////////////////////
void BulletMotionState::SetWorldPose(const math::Pose &_pose)
{
  this->worldPose = _pose;
}

//////////////////////////////////////////////////
void BulletMotionState::SetCoMOffset(const math::Pose &_com)
{
  this->comOffset = _com;
}

//////////////////////////////////////////////////
void BulletMotionState::getWorldTransform(btTransform &_worldTrans) const
{
  math::Pose result = this->worldPose;
  result.pos += this->comOffset.pos;

  _worldTrans = BulletPhysics::ConvertPose(result);
}

//////////////////////////////////////////////////
void BulletMotionState::setWorldTransform(const btTransform &_worldTrans)
{
  if (this->visual == NULL)
    return;

  this->worldPose = BulletPhysics::ConvertPose(_worldTrans);

  this->body->SetWorldPose(this->worldPose, false);
  // this->visual->SetDirty(true, this->worldPose - this->body->GetWorldPose());
}


