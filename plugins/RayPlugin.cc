/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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
#include <functional>

#include "gazebo/physics/physics.hh"
#include "RayPlugin.hh"

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(RayPlugin)

/////////////////////////////////////////////////
RayPlugin::RayPlugin()
{
gzdbg << "RayPlugin" << std::endl;
}

/////////////////////////////////////////////////
RayPlugin::~RayPlugin()
{
gzdbg << "~RayPlugin" << std::endl;
  if (this->newLaserScansConnection)
  {
gzdbg << "~RayPlugin " << this->newLaserScansConnection << std::endl;
}
  this->newLaserScansConnection.reset();
gzdbg << "~RayPlugin" << std::endl;
  this->parentSensor.reset();
gzdbg << "~RayPlugin" << std::endl;
  this->world.reset();
gzdbg << "~RayPlugin" << std::endl;
}

/////////////////////////////////////////////////
void RayPlugin::Load(sensors::SensorPtr _parent, sdf::ElementPtr /*_sdf*/)
{
gzdbg << "RayPlugin" << std::endl;
  // Get then name of the parent sensor
  this->parentSensor =
    std::dynamic_pointer_cast<sensors::RaySensor>(_parent);

  if (!this->parentSensor)
    gzthrow("RayPlugin requires a Ray Sensor as its parent");

  this->world = physics::get_world(this->parentSensor->WorldName());

  this->newLaserScansConnection =
    this->parentSensor->LaserShape()->ConnectNewLaserScans(
      std::bind(&RayPlugin::OnNewLaserScans, this));
gzdbg << "RayPlugin" << std::endl;
}

/////////////////////////////////////////////////
void RayPlugin::OnNewLaserScans()
{
gzdbg << "RayPlugin" << std::endl;
  /* overload with useful callback here */
}
