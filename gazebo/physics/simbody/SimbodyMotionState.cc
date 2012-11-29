/*
 * Copyright 2011 Nate Koenig
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
/* Desc: Simbody motion state class.
 * Author: Nate Koenig
 * Date: 25 May 2009
 */

#include "physics/Link.hh"
#include "physics/simbody/SimbodyPhysics.hh"
#include "physics/simbody/SimbodyMotionState.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
SimbodyMotionState::SimbodyMotionState(Link *_link)
  : btMotionState()
{
  this->link = _link;
}

//////////////////////////////////////////////////
SimbodyMotionState::~SimbodyMotionState()
{
}

//////////////////////////////////////////////////
math::Pose SimbodyMotionState::GetWorldPose() const
{
  return this->worldPose;
}

//////////////////////////////////////////////////
void SimbodyMotionState::SetWorldPosition(const math::Vector3 &_pos)
{
  this->worldPose.pos = _pos;
}

//////////////////////////////////////////////////
void SimbodyMotionState::SetWorldRotation(const math::Quaternion &_rot)
{
  this->worldPose.rot = _rot;
}

//////////////////////////////////////////////////
void SimbodyMotionState::SetWorldPose(const math::Pose &_pose)
{
  this->worldPose = _pose;
}

//////////////////////////////////////////////////
void SimbodyMotionState::SetCoG(const math::Vector3 &_cog)
{
  this->cog = _cog;
  math::Vector3 cg = this->worldPose.rot.RotateVector(this->cog);
  this->worldPose.pos += cg;
}

//////////////////////////////////////////////////
void SimbodyMotionState::getWorldTransform(btTransform &_worldTrans) const
{
  math::Pose result = this->worldPose;
  _worldTrans = SimbodyPhysics::ConvertPose(result);
}

//////////////////////////////////////////////////
void SimbodyMotionState::setWorldTransform(const btTransform &_worldTrans)
{
  this->worldPose = SimbodyPhysics::ConvertPose(_worldTrans);

  math::Vector3 cg = this->worldPose.rot.RotateVector(this->cog);
  this->worldPose.pos -= cg;

  this->link->SetWorldPose(this->worldPose, false);
}
