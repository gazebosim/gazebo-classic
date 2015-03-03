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
  if (this->updateConnection.get())
  {
    event::Events::DisconnectWorldUpdateBegin(this->updateConnection);
    this->updateConnection = gazebo::event::ConnectionPtr();
  }

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
       boost::bind(&ForceTorqueModelRemovalTestPlugin::onUpdate, this, _1));
}

void ForceTorqueModelRemovalTestPlugin::onUpdate(
    const gazebo::common::UpdateInfo & /*_info*/)
{
  gazebo::math::Vector3 force;
  gazebo::math::Vector3 torque;

  if ( parentSensor )
  {
    force = this->parentSensor->GetForce();
    torque = this->parentSensor->GetTorque();

    int i = 0;

    for (i = 0; i < 3; ++i)
    {
      forcetorque_data[0+i] = force[i];
    }

    for (i = 0; i < 3; ++i)
    {
      forcetorque_data[3+i] = torque[i];
    }
  }

  return;
}
