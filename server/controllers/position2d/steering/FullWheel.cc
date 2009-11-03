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
 * Desc: Full (steering and drive) or Steer (steering only) wheel
 * Author: Jordi Polo
 * Date: 18 Dec 2007
 */

#include "Global.hh"
#include "XMLConfig.hh"
#include "Model.hh"
#include "Joint.hh"
#include "World.hh"
#include "gazebo.h"
#include "GazeboError.hh"
#include "ControllerFactory.hh"
#include "Steering_Position2d.hh"
#include "FullWheel.hh"
#include <string>

using namespace gazebo;


////////////////////////////////////////////////////////////////////////////////
// Constructor
FullWheel::FullWheel( )
{

}

////////////////////////////////////////////////////////////////////////////////
// Destructor
FullWheel::~FullWheel( )
{

}

////////////////////////////////////////////////////////////////////////////////
// Connects the wheel to a given Joint
void FullWheel::Connect(Joint *joint, int type)
{
  this->joint= joint;
  this->type=type;
  if (!this->joint)
  {
    std::ostringstream stream;
    stream << "The controller couldn't get the joint " <<this->joint->GetName();
    gzthrow(stream.str());
  }

  //avoid an initial impulse to the joints that would make the vehicle flip
  this->joint->SetAttribute(Joint::FUDGE_FACTOR,0, 0.1);
}


////////////////////////////////////////////////////////////////////////////////
// Stops the wheel
void FullWheel::Stop()
{
  this->joint->SetVelocity(0, 0);
  this->joint->SetMaxForce(0, 0);
  this->joint->SetVelocity(1, 0);
  this->joint->SetMaxForce(1, 0);
}

////////////////////////////////////////////////////////////////////////////////
// Set the torque
void FullWheel::SetTorque( float newTorque)
{
  this->torque=newTorque;
  this->joint->SetMaxForce(1, this->torque);
}

////////////////////////////////////////////////////////////////////////////////
// Set the steering torque
void FullWheel::SetSteerTorque(float newTorque)
{
  this->steerTorque=newTorque;
  this->joint->SetMaxForce(0, this->steerTorque);
}

////////////////////////////////////////////////////////////////////////////////
// Set the steering parameters
void FullWheel::SetSteerPD(float kp, float kd)
{
  this->steerKp=kp;
  this->steerKd=kd;
}

////////////////////////////////////////////////////////////////////////////////
// Set the steering max angle
void FullWheel::SetSteerMaxAngle(float maxAngle)
{
  this->steerMaxAngle=maxAngle;
}

////////////////////////////////////////////////////////////////////////////////
// Updates
void FullWheel::Update(float speed, float steer)
{
  double v;
  double kp, kd;

  this->cmdSpeed=speed;
  this->cmdSteer=steer;

  if (this->cmdSteer > +this->steerMaxAngle)
    this->cmdSteer = +this->steerMaxAngle;
  else if (this->cmdSteer < -this->steerMaxAngle)
    this->cmdSteer = -this->steerMaxAngle;


  // TODO: proper acceleration model
  // Normalize the gain factors using the step time (dont what the PD
  // to depend on sim cycle rate)
  kp = this->steerKp;
  kd = this->steerKd;

  // Set the turn angle; PD control
  v = kp * (this->cmdSteer - this->joint->GetAngle(0).GetAsRadian())
      - kd * this->joint->GetVelocity(0);
  this->joint->SetVelocity(0, v);

  this->joint->SetMaxForce(0, this->steerTorque);

  if (type==FULL)
  {
    this->joint->SetMaxForce(1, this->torque);
    this->joint->SetVelocity(1, this->cmdSpeed);
  }

}

////////////////////////////////////////////////////////////////////////////////
// Set the suspension
void FullWheel::SetSuspension(float spring, float damping, float step)
{
  joint->SetAttribute(Joint::SUSPENSION_ERP,0, 
      step*spring/(step*spring+damping));
  joint->SetAttribute(Joint::SUSPENSION_CFM,0, 1.0/(step*spring+damping));
}
