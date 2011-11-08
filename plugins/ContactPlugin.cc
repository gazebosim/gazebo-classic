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
 * Desc: Contact plugin
 * Author: Nate Koenig mod by John Hsu
 */

#include "physics/physics.h"
#include "ContactPlugin.hh"

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(ContactPlugin)

////////////////////////////////////////////////////////////////////////////////
// Constructor
ContactPlugin::ContactPlugin()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
ContactPlugin::~ContactPlugin()
{
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void ContactPlugin::Load(sensors::SensorPtr &_parent, sdf::ElementPtr &_sdf)
{
  // Get then name of the parent sensor
  this->parentSensor = 
    boost::shared_dynamic_cast<sensors::ContactSensor>(_parent);

  this->world = physics::get_world(_sdf->GetWorldName());

  if (!this->parentSensor)
    gzthrow("ContactPlugin requires a Contact Sensor as its parent");
}
