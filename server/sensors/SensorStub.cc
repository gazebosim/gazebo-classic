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
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
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
