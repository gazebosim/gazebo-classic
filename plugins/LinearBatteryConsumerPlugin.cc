/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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

#include "gazebo/common/Assert.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/common/Battery.hh"
#include "LinearBatteryConsumerPlugin.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(LinearBatteryConsumerPlugin)

/////////////////////////////////////////////////
LinearBatteryConsumerPlugin::LinearBatteryConsumerPlugin() : consumerId(-1)
{
}

/////////////////////////////////////////////////
LinearBatteryConsumerPlugin::~LinearBatteryConsumerPlugin()
{
  if (this->battery && this->consumerId != -1)
    this->battery->RemoveConsumer(this->consumerId);
}

/////////////////////////////////////////////////
void LinearBatteryConsumerPlugin::Load(physics::ModelPtr _parent,
                                       sdf::ElementPtr _sdf)
{
  sdf::ElementPtr elem = _sdf->GetElement("link_name");
  GZ_ASSERT(elem, "Element link_name doesn't exist!");
  physics::LinkPtr link = _parent->GetLink(elem->Get<std::string>());
  GZ_ASSERT(link, "Link was NULL");

  elem = _sdf->GetElement("battery_name");
  GZ_ASSERT(elem, "Element battery_name doesn't exist!");
  this->battery = link->Battery(elem->Get<std::string>());
  GZ_ASSERT(this->battery, "Battery was NULL");

  if (_sdf->HasElement("power_load"))
  {
    double powerLoad = _sdf->Get<double>("power_load");
    this->consumerId = this->battery->AddConsumer();
    bool success = this->battery->SetPowerLoad(this->consumerId, powerLoad);
    GZ_ASSERT(success, "Failed to set consumer power load.");
  }
  else
  {
    gzwarn << "Required attribute power_load missing "
           << "in LinearBatteryConsumerPlugin SDF" << std::endl;
  }
}
