/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
 * Desc: Contact plugin
 * Author: Nate Koenig mod by John Hsu
 */

#include "gazebo/physics/physics.hh"
#include "RayPlugin.hh"

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(RayPlugin)

/////////////////////////////////////////////////
RayPlugin::RayPlugin()
{
}

/////////////////////////////////////////////////
RayPlugin::~RayPlugin()
{
  this->parentSensor->GetLaserShape()->DisconnectNewLaserScans(
      this->newLaserScansConnection);
  this->newLaserScansConnection.reset();

  this->parentSensor.reset();
  this->world.reset();
}

/////////////////////////////////////////////////
void RayPlugin::Load(sensors::SensorPtr _parent, sdf::ElementPtr /*_sdf*/)
{
  // Get then name of the parent sensor
  this->parentSensor =
    boost::dynamic_pointer_cast<sensors::RaySensor>(_parent);

  if (!this->parentSensor)
    gzthrow("RayPlugin requires a Ray Sensor as its parent");

  this->world = physics::get_world(this->parentSensor->GetWorldName());

  this->newLaserScansConnection =
    this->parentSensor->GetLaserShape()->ConnectNewLaserScans(
      boost::bind(&RayPlugin::OnNewLaserScans, this));
}

/////////////////////////////////////////////////
void RayPlugin::OnNewLaserScans()
{
  /* overload with useful callback here */
}
