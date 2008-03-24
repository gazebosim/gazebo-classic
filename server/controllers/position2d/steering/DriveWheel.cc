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
/*
 * Desc: Wheel that can not be steered
 * Author: Jordi Polo
 * Date: 18 Dec 2007
 */

#include "Global.hh"
#include "XMLConfig.hh"
#include "Model.hh"
#include "HingeJoint.hh"
#include "Hinge2Joint.hh"
#include "World.hh"
#include "gazebo.h"
#include "GazeboError.hh"
#include "ControllerFactory.hh"
#include "Steering_Position2d.hh"
#include "DriveWheel.hh"
#include <string>

using namespace gazebo;


////////////////////////////////////////////////////////////////////////////////
// Constructor
DriveWheel::DriveWheel( )
{

}

////////////////////////////////////////////////////////////////////////////////
// Destructor
DriveWheel::~DriveWheel( )
{

}


void DriveWheel::Connect(Joint *joint, int type)
{
  this->joint= dynamic_cast<HingeJoint*>(joint);
  this->type=type;
  if (!joint)
  {
    std::ostringstream stream;
    stream << "The controller couldn't get the joint " << joint->GetName();
    gzthrow(stream.str());
  }
  //avoid an initial impulse to the joints that would make the vehicle flip
  this->joint->SetParam(dParamFudgeFactor, 0.1);
}

void DriveWheel::Stop()
{
  this->joint->SetParam(dParamVel, 0);
  this->joint->SetParam(dParamFMax, 0);
}

void DriveWheel::SetTorque( float newTorque)
{
  this->torque=newTorque;
  this->joint->SetParam(dParamFMax, this->torque);
}


void DriveWheel::SetSteerTorque(float newTorque)
{

}



void DriveWheel::Update(float speed, float steer)
{
  this->cmdSpeed=speed;

  // TODO: proper acceleration model
  this->joint->SetParam(dParamFMax, this->torque);
  this->joint->SetParam(dParamVel, this->cmdSpeed);
}

void DriveWheel::SetSuspension(float spring, float damping, float step)
{
  joint->SetParam(dParamSuspensionERP, step*spring/(step*spring+damping));
  joint->SetParam(dParamSuspensionCFM, 1.0/(step*spring+damping));
}

