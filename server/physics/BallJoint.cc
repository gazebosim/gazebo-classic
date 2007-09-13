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
/* Desc: A ball joint
 * Author: Nate Keonig, Andrew Howard
 * Date: 21 May 2003
 * CVS: $Id$
 */

#include "World.hh"
#include "BallJoint.hh"

using namespace gazebo;

//////////////////////////////////////////////////////////////////////////////
// Constructor
BallJoint::BallJoint(dWorldID worldId)
  : Joint()
{
  this->jointId = dJointCreateBall(worldId, NULL);
}


//////////////////////////////////////////////////////////////////////////////
// Destructor
BallJoint::~BallJoint()
{
}

//////////////////////////////////////////////////////////////////////////////
/// Load the joint
void BallJoint::LoadChild(XMLConfigNode * /*node*/)
{
}

//////////////////////////////////////////////////////////////////////////////
// Get the joints anchor point
Vector3 BallJoint::GetAnchor() const
{
  dVector3 result;
  dJointGetBallAnchor( jointId, result );
  return Vector3(result[0], result[1], result[2]);
}


//////////////////////////////////////////////////////////////////////////////
// Set the joints anchor point
void BallJoint::SetAnchor(const Vector3 &anchor) 
{
  dJointSetBallAnchor( jointId, anchor.x, anchor.y, anchor.z );
}

