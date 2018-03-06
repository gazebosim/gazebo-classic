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

#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"
#include "plugins/VehiclePlugin.hh"

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(VehiclePlugin)

/////////////////////////////////////////////////
VehiclePlugin::VehiclePlugin()
{
  this->joints.resize(4);

  this->aeroLoad = 0.1;
  this->swayForce = 10;

  this->maxSpeed = 10;
  this->frontPower = 50;
  this->rearPower = 50;
  this->wheelRadius = 0.3;
  this->maxBrake = 0.0;
  this->maxGas = 0.0;
  this->steeringRatio = 1.0;
  this->tireAngleRange = 1.0;
}

/////////////////////////////////////////////////
void VehiclePlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  this->model = _model;
  // this->physics = this->model->GetWorld()->Physics();

  this->joints[0] = this->model->GetJoint(_sdf->Get<std::string>("front_left"));
  if (!this->joints[0])
  {
    gzerr << "Unable to find joint: front_left\n";
    return;
  }

  this->joints[1] = this->model->GetJoint(
      _sdf->Get<std::string>("front_right"));

  if (!this->joints[1])
  {
    gzerr << "Unable to find joint: front_right\n";
    return;
  }

  this->joints[2] = this->model->GetJoint(_sdf->Get<std::string>("back_left"));
  if (!this->joints[2])
  {
    gzerr << "Unable to find joint: back_left\n";
    return;
  }


  this->joints[3] = this->model->GetJoint(_sdf->Get<std::string>("back_right"));
  if (!this->joints[3])
  {
    gzerr << "Unable to find joint: back_right\n";
    return;
  }

  this->joints[0]->SetParam("suspension_erp", 0, 0.15);
  this->joints[0]->SetParam("suspension_cfm", 0, 0.04);

  this->joints[1]->SetParam("suspension_erp", 0, 0.15);
  this->joints[1]->SetParam("suspension_cfm", 0, 0.04);

  this->joints[2]->SetParam("suspension_erp", 0, 0.15);
  this->joints[2]->SetParam("suspension_cfm", 0, 0.04);

  this->joints[3]->SetParam("suspension_erp", 0, 0.15);
  this->joints[3]->SetParam("suspension_cfm", 0, 0.04);

  this->gasJoint = this->model->GetJoint(_sdf->Get<std::string>("gas"));
  this->brakeJoint = this->model->GetJoint(_sdf->Get<std::string>("brake"));
  this->steeringJoint = this->model->GetJoint(
      _sdf->Get<std::string>("steering"));

  if (!this->gasJoint)
  {
    gzerr << "Unable to find gas joint["
          << _sdf->Get<std::string>("gas") << "]\n";
    return;
  }

  if (!this->steeringJoint)
  {
    gzerr << "Unable to find steering joint["
          << _sdf->Get<std::string>("steering") << "]\n";
    return;
  }

  if (!this->joints[0])
  {
    gzerr << "Unable to find front_left joint["
          << _sdf->GetElement("front_left") << "]\n";
    return;
  }

  if (!this->joints[1])
  {
    gzerr << "Unable to find front_right joint["
          << _sdf->GetElement("front_right") << "]\n";
    return;
  }

  if (!this->joints[2])
  {
    gzerr << "Unable to find back_left joint["
          << _sdf->GetElement("back_left") << "]\n";
    return;
  }

  if (!this->joints[3])
  {
    gzerr << "Unable to find back_right joint["
          << _sdf->GetElement("back_right") << "]\n";
    return;
  }

  this->maxSpeed = _sdf->Get<double>("max_speed");
  this->aeroLoad = _sdf->Get<double>("aero_load");
  this->tireAngleRange = _sdf->Get<double>("tire_angle_range");
  this->frontPower = _sdf->Get<double>("front_power");
  this->rearPower = _sdf->Get<double>("rear_power");

  this->connections.push_back(event::Events::ConnectWorldUpdateBegin(
          std::bind(&VehiclePlugin::OnUpdate, this)));

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->model->GetWorld()->Name());

  this->velSub = this->node->Subscribe(std::string("~/") +
      this->model->GetName() + "/vel_cmd", &VehiclePlugin::OnVelMsg, this);
}

/////////////////////////////////////////////////
void VehiclePlugin::Init()
{
  this->chassis = this->joints[0]->GetParent();

  // This assumes that the largest dimension of the wheel is the diameter
  physics::EntityPtr parent = boost::dynamic_pointer_cast<physics::Entity>(
      this->joints[0]->GetChild());
  ignition::math::Box bb = parent->BoundingBox();
  this->wheelRadius = bb.Size().Max() * 0.5;

  // The total range the steering wheel can rotate
  double steeringRange = this->steeringJoint->UpperLimit(0) -
                         this->steeringJoint->LowerLimit(0);

  // Compute the angle ratio between the steering wheel and the tires
  this->steeringRatio = steeringRange / this->tireAngleRange;

  // Maximum gas is the upper limit of the gas joint
  this->maxGas = this->gasJoint->UpperLimit(0);

  // Maximum brake is the upper limit of the gas joint
  this->maxBrake = this->gasJoint->UpperLimit(0);

  printf("SteeringRation[%f] MaxGa[%f]\n", this->steeringRatio, this->maxGas);
}

/////////////////////////////////////////////////
void VehiclePlugin::OnUpdate()
{
  // Get the normalized gas and brake amount
  double gas = this->gasJoint->Position(0) / this->maxGas;
  double brake = this->brakeJoint->Position(0) / this->maxBrake;

  // A little force to push back on the pedals
  this->gasJoint->SetForce(0, -0.1);
  this->brakeJoint->SetForce(0, -0.1);

  // Get the steering angle
  double steeringAngle = this->steeringJoint->Position(0);

  // Compute the angle of the front wheels.
  double wheelAngle = steeringAngle / this->steeringRatio;

  // double idleSpeed = 0.5;

  // Compute the rotational velocity of the wheels
  double jointVel = (std::max(0.0, gas-brake) * this->maxSpeed) /
                    this->wheelRadius;

  // Set velocity and max force for each wheel
  this->joints[0]->SetVelocityLimit(1, -jointVel);
  this->joints[0]->SetForce(1, (gas + brake) * this->frontPower);

  this->joints[1]->SetVelocityLimit(1, -jointVel);
  this->joints[1]->SetForce(1, (gas + brake) * this->frontPower);

  this->joints[2]->SetVelocityLimit(1, -jointVel);
  this->joints[2]->SetForce(1, (gas + brake) * this->rearPower);

  this->joints[3]->SetVelocityLimit(1, -jointVel);
  this->joints[3]->SetForce(1, (gas + brake) * this->rearPower);

  // Set the front-left wheel angle
  this->joints[0]->SetLowerLimit(0, wheelAngle);
  this->joints[0]->SetUpperLimit(0, wheelAngle);
  this->joints[0]->SetLowerLimit(0, wheelAngle);
  this->joints[0]->SetUpperLimit(0, wheelAngle);

  // Set the front-right wheel angle
  this->joints[1]->SetUpperLimit(0, wheelAngle);
  this->joints[1]->SetLowerLimit(0, wheelAngle);
  this->joints[1]->SetUpperLimit(0, wheelAngle);
  this->joints[1]->SetLowerLimit(0, wheelAngle);

  // Get the current velocity of the car
  this->velocity = this->chassis->WorldLinearVel();

  //  aerodynamics
  this->chassis->AddForce(
      ignition::math::Vector3d(0, 0,
      this->aeroLoad * this->velocity.SquaredLength()));

  // Sway bars
  ignition::math::Vector3d bodyPoint;
  ignition::math::Vector3d hingePoint;
  ignition::math::Vector3d axis;

  for (int ix = 0; ix < 4; ++ix)
  {
    hingePoint = this->joints[ix]->Anchor(0);
    bodyPoint = this->joints[ix]->Anchor(1);

    axis = this->joints[ix]->GlobalAxis(0).Round();
    double displacement = (bodyPoint - hingePoint).Dot(axis);

    float amt = displacement * this->swayForce;
    if (displacement > 0)
    {
      if (amt > 15)
        amt = 15;

      ignition::math::Pose3d p =
          this->joints[ix]->GetChild()->WorldPose();
      this->joints[ix]->GetChild()->AddForce(axis * -amt);
      this->chassis->AddForceAtWorldPosition(axis * amt, p.Pos());

      p = this->joints[ix^1]->GetChild()->WorldPose();
      this->joints[ix^1]->GetChild()->AddForce(axis * amt);
      this->chassis->AddForceAtWorldPosition(axis * -amt, p.Pos());
    }
  }
}

/////////////////////////////////////////////////
void VehiclePlugin::OnVelMsg(ConstPosePtr &/*_msg*/)
{
}
