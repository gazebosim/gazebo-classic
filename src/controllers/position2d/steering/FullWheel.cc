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
