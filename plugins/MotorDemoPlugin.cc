/*
 * Copyright (C) 2014 Open Source Robotics Foundation
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

#include <boost/unordered_map.hpp>
#include <boost/unordered_set.hpp>

#include "gazebo/common/Assert.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/sensors/SensorManager.hh"
#include "gazebo/transport/transport.hh"
#include "plugins/MotorDemoPlugin.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(MotorDemoPlugin)

/////////////////////////////////////////////////
MotorDemoPlugin::MotorDemoPlugin() : maxTorque(0.0), torqueSpeedSlope(-1.0)
{
}

/////////////////////////////////////////////////
void MotorDemoPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model, "MotorDemoPlugin _model pointer is NULL");
  this->model = _model;
  this->modelName = _model->GetName();
  this->sdf = _sdf;

  this->world = this->model->GetWorld();
  GZ_ASSERT(this->world, "MotorDemoPlugin world pointer is NULL");

  this->physics = this->world->GetPhysicsEngine();
  GZ_ASSERT(this->physics, "MotorDemoPlugin physics pointer is NULL");

  GZ_ASSERT(_sdf, "MotorDemoPlugin _sdf pointer is NULL");

  if (_sdf->HasElement("max_torque"))
    this->maxTorque = _sdf->Get<double>("max_torque");

  // torque speed curve?
  // how to define this curve?  See example:
  // http://lancet.mit.edu/motors/motors3.html
  if (_sdf->HasElement("torque_speed_slope"))
    this->torqueSpeedSlope = _sdf->Get<double>("torque_speed_slope");

  if (_sdf->HasElement("motor_shaft_joint_name"))
  {
    this->motorShaftJointName =
      _sdf->Get<std::string>("motor_shaft_joint_name");
  }
  else
  {
    gzerr << "motor_shaft_joint_name not specified, cannot load plugin.\n";
    return;
  }

  if (_sdf->HasElement("encoder_joint_name"))
  {
    this->encoderJointName =
      _sdf->Get<std::string>("encoder_joint_name");
  }
  else
  {
    gzerr << "encoder_joint_name not specified, cannot load plugin.\n";
    return;
  }

  if (_sdf->HasElement("force_torque_sensor_joint_name"))
  {
    this->forceTorqueSensorJointName =
      _sdf->Get<std::string>("force_torque_sensor_joint_name");
  }
  else
  {
    gzerr << "force_torque_sensor_joint_name not specified,"
          << " cannot load plugin.\n";
    return;
  }
}

/////////////////////////////////////////////////
void MotorDemoPlugin::Init()
{
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&MotorDemoPlugin::OnUpdate, this));
}

/////////////////////////////////////////////////
void MotorDemoPlugin::OnUpdate()
{
}
