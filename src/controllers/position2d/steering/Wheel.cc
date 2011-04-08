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
 * Desc: General Wheel
 * Author: Jordi Polo
 * Date: 18 Dec 2007
 */

#include "common/Global.hh"
#include "common/XMLConfig.hh"
#include "Model.hh"
#include "HingeJoint.hh"
#include "Hinge2Joint.hh"
#include "World.hh"
#include "common/Exception.hh"
#include "ControllerFactory.hh"
#include "Steering_Position2d.hh"
#include "Wheel.hh"
#include <string>

using namespace gazebo;


enum {DRIVE, STEER, FULL};


////////////////////////////////////////////////////////////////////////////////
// Constructor
Wheel::Wheel( )
{

}

////////////////////////////////////////////////////////////////////////////////
// Destructor
Wheel::~Wheel( )
{

}


void Wheel::SetSuspension(float spring, float damping, float step)
{
}


void Wheel::Connect(Joint *joint, int type)
{

}

void Wheel::Stop()
{

}


void Wheel::SetTorque( float newTorque)
{
}

void Wheel::Update(float speed, float steer)
{

}



