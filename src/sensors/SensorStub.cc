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

/* Desc: A stubbed out sensor
 * Author: Nate Koenig
 * Date: 05 Aug 2007
 * SVn: $Id$
 */

#include <sstream>
#include "Global.hh"
#include "GazeboError.hh"
#include "Body.hh"

#include "SensorFactory.hh"
#include "SensorStub.hh"

using namespace gazebo;

GZ_REGISTER_STATIC_SENSOR("stub", SensorStub);

//////////////////////////////////////////////////////////////////////////////
// Constructor
SensorStub::SensorStub(Body *body)
    : Sensor(body)
{
}


//////////////////////////////////////////////////////////////////////////////
// Destructor
SensorStub::~SensorStub()
{
}

//////////////////////////////////////////////////////////////////////////////
// Load the camera
void SensorStub::LoadChild( XMLConfigNode *node )
{
}

//////////////////////////////////////////////////////////////////////////////
// Initialize the camera
void SensorStub::InitChild()
{
}

//////////////////////////////////////////////////////////////////////////////
// Finalize the camera
void SensorStub::FiniChild()
{
}

//////////////////////////////////////////////////////////////////////////////
// Update the drawing
void SensorStub::UpdateChild(UpdateParams &params)
{
}
