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
/* Desc: Base class for all sensors
 * Author: Nathan Koenig
 * Date: 25 May 2007
 * SVN: $Id$
 */

#include "XMLConfig.hh"
#include "Sensor.hh"

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
// Constructor
Sensor::Sensor()
  : Entity()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
Sensor::~Sensor()
{
}

////////////////////////////////////////////////////////////////////////////////
// Load the sensor
void Sensor::Load(XMLConfigNode *node)
{
  this->LoadChild(node);
}

////////////////////////////////////////////////////////////////////////////////
/// Initialize the sensor
void Sensor::Init()
{
  this->InitChild();
}

////////////////////////////////////////////////////////////////////////////////
/// Update the sensor
void Sensor::Update(UpdateParams &params)
{
  this->UpdateChild(params);
}

////////////////////////////////////////////////////////////////////////////////
/// Finalize the sensor
void Sensor::Fini()
{
  this->FiniChild();
}

