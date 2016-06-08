/*
 * Copyright (C) 2014-2016 Open Source Robotics Foundation
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
#include "ImuSensorPlugin.hh"

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(ImuSensorPlugin)

/////////////////////////////////////////////////
ImuSensorPlugin::ImuSensorPlugin()
{
}

/////////////////////////////////////////////////
ImuSensorPlugin::~ImuSensorPlugin()
{
  this->connection.reset();
  this->parentSensor.reset();
}

/////////////////////////////////////////////////
void ImuSensorPlugin::Load(sensors::SensorPtr _parent,
    sdf::ElementPtr /*_sdf*/)
{
  this->parentSensor =
    std::dynamic_pointer_cast<sensors::ImuSensor>(_parent);

  this->world = physics::get_world(_parent->WorldName());
  physics::EntityPtr entity = this->world->GetEntity(_parent->ParentName());
  this->link = boost::dynamic_pointer_cast<physics::Link>(entity);

  if (!this->link)
    gzthrow("Imu sensor parent is not a Link.");

  if (!this->parentSensor)
    gzthrow("ImuSensorPlugin requires a imu sensor as its parent.");

  this->connection = this->parentSensor->ConnectUpdated(
        std::bind(&ImuSensorPlugin::OnUpdate, this, this->parentSensor));
}

/////////////////////////////////////////////////
void ImuSensorPlugin::OnUpdate(sensors::ImuSensorPtr /*_sensor*/)
{
  // overload with useful callback here
  // gzdbg << _sensor->GetName() << " :\n"
  //       << "  sensor linear accel [" << _sensor->GetLinearAcceleration()
  //       << "]\n  Link::GetRelativeLinearAccel() ["
  //       << this->link->GetRelativeLinearAccel()
  //       << "]\n  Link::GetWorldLinearAccel() ["
  //       << this->link->GetWorldLinearAccel()
  //       << "]\n";
}
