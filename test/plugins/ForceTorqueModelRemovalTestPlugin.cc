/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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

#include "plugins/ForceTorqueModelRemovalTestPlugin.hh"

#include "gazebo/sensors/ForceTorqueSensor.hh"


using namespace gazebo;

GZ_REGISTER_SENSOR_PLUGIN(ForceTorqueModelRemovalTestPlugin)

/////////////////////////////////////////////////
ForceTorqueModelRemovalTestPlugin::ForceTorqueModelRemovalTestPlugin()
  : SensorPlugin(), parentSensor(0)
{
}

/////////////////////////////////////////////////
void ForceTorqueModelRemovalTestPlugin::Init()
{
}

/////////////////////////////////////////////////
ForceTorqueModelRemovalTestPlugin::~ForceTorqueModelRemovalTestPlugin()
{
  this->updateConnection.reset();

  parentSensor = 0;
}

void ForceTorqueModelRemovalTestPlugin::Load(sensors::SensorPtr _sensor,
                                             sdf::ElementPtr /*_sdf*/)
{
  _sensor->SetActive(true);

  parentSensor =
    (gazebo::sensors::ForceTorqueSensor*) boost::get_pointer(_sensor);

  // Create connection
  this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
       std::bind(&ForceTorqueModelRemovalTestPlugin::onUpdate, this,
                 std::placeholders::_1));
}

void ForceTorqueModelRemovalTestPlugin::onUpdate(
    const gazebo::common::UpdateInfo & /*_info*/)
{
  ignition::math::Vector3d force;
  ignition::math::Vector3d torque;

  if ( parentSensor )
  {
    force = this->parentSensor->Force();
    torque = this->parentSensor->Torque();

    int i = 0;

    for (i = 0; i < 3; ++i)
    {
      this->forcetorque_data[0+i] = force[i];
    }

    for (i = 0; i < 3; ++i)
    {
      this->forcetorque_data[3+i] = torque[i];
    }
  }

  return;
}
