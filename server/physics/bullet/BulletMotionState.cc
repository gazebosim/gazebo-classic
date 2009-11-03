/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/* Desc: Bullet motion state class.
 * Author: Nate Koenig 
 * Date: 25 May 2009
 */

#include "Simulator.hh"
#include "BulletPhysics.hh"
#include "OgreVisual.hh"
#include "Body.hh"
#include "BulletMotionState.hh"

using namespace gazebo;

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
void BulletMotionState::SetVisual(OgreVisual *vis) 
{
  this->visual = vis;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the pose
Pose3d BulletMotionState::GetAbsPose() const
{
  std::cout << "MotionState: Get Abs Pose[" << this->absPose << "]\n";
  return this->absPose;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the position of the body
void BulletMotionState::SetAbsPosition(const Vector3 &pos)
{
  this->absPose.pos = pos;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the rotation of the body
void BulletMotionState::SetAbsRotation(const Quatern &rot)
{
  this->absPose.rot = rot;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the pose
void BulletMotionState::SetAbsPose(const Pose3d &pose)
{
  this->absPose = pose;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the center of mass offset
void BulletMotionState::SetCoMOffset( const Pose3d &com )
{
  this->comOffset = com;
}

////////////////////////////////////////////////////////////////////////////////
// Get the world transform
void BulletMotionState::getWorldTransform(btTransform &worldTrans) const 
{
  Pose3d result = this->absPose;
  result.pos += this->comOffset.pos;

  worldTrans = BulletPhysics::ConvertPose(result);
}

////////////////////////////////////////////////////////////////////////////////
// Set the world transform
void BulletMotionState::setWorldTransform(const btTransform &worldTrans) 
{
  if (this->visual == NULL)
    return;

  this->absPose = BulletPhysics::ConvertPose( worldTrans );

  //std::cout << "New Pose[" << this->pose << "]\n";

  this->body->SetAbsPose(this->absPose, false);
  //this->visual->SetDirty(true, this->absPose - this->body->GetAbsPose());
}
