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
#include <string>
#include <functional>

#include "gazebo/common/Assert.hh"
#include "gazebo/physics/physics.hh"
#include "plugins/LinearBatteryPlugin.hh"

static const double SECOND_TO_HOUR = 1.0/3600;

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(LinearBatteryPlugin)

/////////////////////////////////////////////////
LinearBatteryPlugin::LinearBatteryPlugin()
{
  this->iraw = 0.0;
  this->ismooth = 0.0;

  this->e0 = 0.0;
  this->e1 = 0.0;

  this->q0 = 0.0;
  this->q = this->q0;

  this->c = 0.0;
  this->r = 0.0;
  this->tau = 0.0;
}

/////////////////////////////////////////////////
void LinearBatteryPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  // Store the pointer to the model
  this->model = _parent;
  this->world = _parent->GetWorld();

  if (_sdf->HasElement("link_name"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("link_name");
    GZ_ASSERT(elem, "Element link_name doesn't exist!");
    std::string linkName = elem->Get<std::string>();
    this->link = this->model->GetLink(linkName);
    GZ_ASSERT(link, "Link was NULL");
  }
  else
  {
    gzerr << "link_name not supplied, ignoring LinearBatteryPlugin.\n";
    return;
  }

  if (_sdf->HasElement("open_circuit_voltage_constant_coef"))
    this->e0 = _sdf->Get<double>("open_circuit_voltage_constant_coef");

  if (_sdf->HasElement("open_circuit_voltage_linear_coef"))
    this->e1 = _sdf->Get<double>("open_circuit_voltage_linear_coef");

  if (_sdf->HasElement("initial_charge"))
    this->q0 = _sdf->Get<double>("initial_charge");

  if (_sdf->HasElement("capacity"))
    this->c = _sdf->Get<double>("capacity");

  if (_sdf->HasElement("resistance"))
    this->r = _sdf->Get<double>("resistance");

  if (_sdf->HasElement("smooth_current_tau"))
    this->tau = _sdf->Get<double>("smooth_current_tau");

  if (_sdf->HasElement("battery_name"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("battery_name");
    GZ_ASSERT(elem, "Element battery_name doesn't exist!");
    std::string batteryName = elem->Get<std::string>();
    this->battery = this->link->Battery(batteryName);
    GZ_ASSERT(this->battery, "Battery was NULL");

    if (!this->battery)
    {
      gzerr << "Battery with name[" << batteryName << "] not found. "
            << "The LinearBatteryPlugin will not update its voltage\n";
    }
    else
    {
      this->battery->SetUpdateFunc(
        std::bind(&LinearBatteryPlugin::OnUpdateVoltage, this, _1, _2));
    }
  }
}

/////////////////////////////////////////////////
void LinearBatteryPlugin::Init()
{
  this->q = this->q0;
}

/////////////////////////////////////////////////
void LinearBatteryPlugin::Reset()
{
  this->iraw = 0.0;
  this->ismooth = 0.0;
  this->Init();
}

/////////////////////////////////////////////////
double LinearBatteryPlugin::OnUpdateVoltage(double _voltage,
                               const std::map<uint32_t, double> &_powerLoads)
{
  double dt = this->world->GetPhysicsEngine()->GetMaxStepSize();
  double totalpower = 0.0;
  double k = dt / this->tau;

  if (fabs(_voltage) < 1e-3)
    return 0.0;

  for (auto powerLoad : _powerLoads)
    totalpower += powerLoad.second;

  this->iraw = totalpower / _voltage;

  this->ismooth = this->ismooth + k * (this->iraw - this->ismooth);

  this->q = this->q - SECOND_TO_HOUR * dt * this->ismooth;

  _voltage = this->e0 + this->e1 * (1 - this->q / this->c)
      - this->r * this->ismooth;

  return _voltage;
}
