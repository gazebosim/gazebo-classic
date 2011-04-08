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
 * Desc: Wheel that can not be steered
 * Author: Jordi Polo
 * Date: 18 Dec 2007
 */

#include "common/Global.hh"
#include "common/XMLConfig.hh"
#include "Model.hh"
#include "Joint.hh"
#include "World.hh"
#include "common/Exception.hh"
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
  this->joint= joint;
  this->type=type;
  if (!this->joint)
  {
    std::ostringstream stream;
    stream << "The controller couldn't get the joint " <<this->joint->GetName();
    gzthrow(stream.str());
  }

  //avoid an initial impulse to the joints that would make the vehicle flip
  this->joint->SetAttribute(Joint::FUDGE_FACTOR,0 , 0.1);
}

void DriveWheel::Stop()
{
  this->joint->SetVelocity(0, 0);
  this->joint->SetMaxForce(0, 0);
}

void DriveWheel::SetTorque( float newTorque)
{
  this->torque=newTorque;
  this->joint->SetMaxForce(0, this->torque);
}

void DriveWheel::SetSteerTorque(float newTorque)
{
}

void DriveWheel::Update(float speed, float steer)
{
  this->cmdSpeed=speed;

  // TODO: proper acceleration model
  this->joint->SetMaxForce(0, this->torque);
  this->joint->SetVelocity(0, this->cmdSpeed);
}

void DriveWheel::SetSuspension(float spring, float damping, float step)
{
  joint->SetAttribute(Joint::SUSPENSION_ERP,0, 
      step*spring/(step*spring+damping));
  joint->SetAttribute(Joint::SUSPENSION_CFM,0, 1.0/(step*spring+damping));
}

