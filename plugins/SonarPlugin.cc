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

#include "gazebo/physics/physics.hh"
#include "SonarPlugin.hh"

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(SonarPlugin)

/////////////////////////////////////////////////
SonarPlugin::SonarPlugin()
{
}

/////////////////////////////////////////////////
SonarPlugin::~SonarPlugin()
{
  this->parentSensor->DisconnectUpdate(this->connection);
  this->parentSensor.reset();
}

/////////////////////////////////////////////////
void SonarPlugin::Load(sensors::SensorPtr _parent, sdf::ElementPtr /*_sdf*/)
{
  // Get then name of the parent sensor
  this->parentSensor =
    boost::dynamic_pointer_cast<sensors::SonarSensor>(_parent);

  if (!this->parentSensor)
    gzthrow("SonarPlugin requires a sonar sensor as its parent.");

  this->connection =
    this->parentSensor->ConnectUpdate(
      boost::bind(&SonarPlugin::OnUpdate, this, _1));
}

/////////////////////////////////////////////////
void SonarPlugin::OnUpdate(msgs::SonarStamped /*_msg*/)
{
  // overload with useful callback here
}
