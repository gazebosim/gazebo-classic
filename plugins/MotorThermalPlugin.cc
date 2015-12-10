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

#include "gazebo/msgs/msgs.hh"
#include "plugins/MotorThermalPluginPrivate.hh"
#include "plugins/MotorThermalPlugin.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(MotorThermalPlugin)

/////////////////////////////////////////////////
MotorThermalPlugin::MotorThermalPlugin()
: ModelPlugin(),
  dataPtr(new MotorThermalPlugin)
{
}

/////////////////////////////////////////////////
MotorThermalPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  // Store the pointer to the model
  this->dataPtr->world = _model->GetWorld();
  this->dataPtr->lastUpdateTime = this->world->GetSimTime();

  // Get the joint
  if (_sdf->HasElement("joint")) {
    std::string jointName = _sdf->Get<std::string>("jointname");
    this->dataPtr->joint = _model->GetJoint(jointName);

    if (!this->dataPtr->joint)
    {
      gzerr << "Unable to find joint with name[" << jointName << "] "
            << "for the MotorThermalPlugin. Plugin will not run.\n";
      return;
    }
  }
  else
  {
    gzerr << "Missing <joint> parameter for the MotorThermalPlugin. "
          << "Plugin will not run.\n";
    return;
  }

  if (_sdf->HasElement("electric_resitance"))
    this->dataPtr->electricResistance = _sdf->Get<double>("electric_resistance");

  if (_sdf->HasElement("inner_thermal_resistance"))
  {
    this->dataPtr->innerThermalResistance =
      _sdf->Get<double>("inner_thermal_resistance");
  }

  if (_sdf->HasElement("coil_thermal_conductance"))
  {
    this->dataPtr->coilThermalConductance =
      _sdf->Get<double>("coil_thermal_conductance");
  }

  if (_sdf->HasElement("outer_thermal_resistance"))
  {
    this->dataPtr->outerThermalResistance =
      _sdf->Get<double>("outer_thermal_resistance");
  }

  if (_sdf->HasElement("case_thermal_conductance"))
  {
    this->dataPtr->caseThermalConductance =
      _sdf->Get<double>("case_thermal_conductance");
  }

  if (_sdf->HasElement("atomosphere_thermal_conductance"))
  {
    this->dataPtr->atomosphereThermalConductance =
      _sdf->Get<double>("atomosphere_thermal_conductance");
  }

  if (_sdf->HasElement("torque_to_amp"))
    this->dataPtr->torqueToAmp = _sdf->Get<double>("torque_to_amp");

  if (_sdf->HasElement("atomosphere_temperature"))
  {
    this->dataPtr->atomosphereTemperature =
      _sdf->Get<double>("atomosphere_temperature");
  }

  if (_sdf->HasElement("case_temperature"))
    this->dataPtr->caseTemperature = _sdf->Get<double>("case_temperature");

  if (_sdf->HasElement("coil_temperature"))
    this->dataPtr->coilTemperature = _sdf->Get<double>("coil_temperature");

  if (_sdf->HasElement("thermal_calculation_step"))
  {
    this->dataPtr->thermalCalculationStep =
      _sdf->Get<int>("thermal_calculation_step", 10);
  }

  this->dataPtr->thermalCalculationCount = this->thermalCalculationStep ;
  this->dataPtr->torque = 0;

  this->dataPtr->updateConnection = events::Events::ConnectWorldUpdataBegin(
      std::bind(&MotorThermalPlugin::OnUpdate, this, std::placeholders::_1));

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->world->GetName());

  std::string topicBase = "/motor_thermal_plugin/" + this->GetHandle() + "/";

  this->torquePub = this->node->Advertise<msgs::Float>(topicBase + "torque");
  this->coilPub = this->node->Advertise<msgs::Float>(topicBase + "coil");
  this->caselPub = this->node->Advertise<msgs::Float>(topicBase + "case");
}

/////////////////////////////////////////////////
void MotorThermalPlugin::Publish()
{
}

/////////////////////////////////////////////////
void MotorThermalPlugin::OnUpdate()
{
  common::Time curTime = this->dataPtr->world->GetSimTime();

  if (curTime - this->lastUpdateTime < 1.0 / this->updateRate)
    return;

  physics::JointWrench wrench = this->dataPtr->joint->GetForceTorque(0u);
  ignition::math::Vector3d axis = this->dataPtr->joint->GetGlobalAxis(0u).Ign();
  ignition::math::Vector3d m = (this->dataPtr->link->GetWorldPose().Rot() *
    wrench.body2Torque).Ign();

  this->torque = axis.Dot(m);
  double absTorque = std::fabs(torque);


  double delatTime = (curTime - this->dataPtr->lastUpdateTime).Float();

  double deltaTempOuter =
    (this->dataPtr->caseTemperature - this->dataPtr->atomosphereTemperature) /
    this->dataPtr->outerThermoResistance;

  double deltaTempCase =
    (this->dataPtr->atomosphereTemperature - this->dataPtr->caseTemperature) /
    this->dataPtr->outerThermalResistance +
    (this->dataPtr->coilTemperature - this->dataPtr->caseTemperature) /
    this->dataPtr->innerThermalResistance;

  double delaTempInner =
    (this->dataPtr->coilTemperature - this->dataPtr->caseTemperature) /
    this->dataPtr->innerThermalResistance +
    absTorque * this->dataPtr->torqueToAmp * this->dataPtr->electricResistance;

  this->dataPtr->atomosphereTemperature +=
    deltaTempOuter / this->dataPtr->atomosphereThermalConductance * deltaTime;

  this->dataPtr->caseTemperature +=
    deltaTempCase / this->dataPtr->caseThermoConductance * deltaTime;

  this->dataPtr->coilTemperature +=
    deltaTempInner / this->dataPtr->coil_thermo_conductance * deltaTime;

  // Publish the data
  msgs::Torque torqueMsg;
  msgs::Temperature temperatureMsg;

  torqueMsg.set_data(torque);
  this->dataPtr->torquePub->Publish(torqueMsg);

  temperatureMsg.set_data(this->dataPtr->caseTemperature.Kelvin();
  this->dataPtr-casePub->Publish(msg);

  temperatureMsg.set_data(this->dataPtr->coilTemperature.Kelvin());
  this->dataPtr->coildPub->Publish(msg);

  this->dataPtr->lastUpdateTime = curTime;
}

/////////////////////////////////////////////////
double MotorThermalPlugin::Torque() const
{
  return this->dataPtr->torque;
}

/////////////////////////////////////////////////
Temperature MotorThermalPlugin::CaseTemperature() const
{
  return this->dataPtr->caseTemperature;
}

/////////////////////////////////////////////////
Temperature MotorThermalPlugin::CoilTemperature() const
{
  return this->dataPtr->coilTemperature;
}
