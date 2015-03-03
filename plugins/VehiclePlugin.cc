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
}

/////////////////////////////////////////////////
void VehiclePlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  this->model = _model;
  // this->physics = this->model->GetWorld()->GetPhysicsEngine();

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
          boost::bind(&VehiclePlugin::OnUpdate, this)));

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->model->GetWorld()->GetName());

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
  math::Box bb = parent->GetBoundingBox();
  this->wheelRadius = bb.GetSize().GetMax() * 0.5;

  // The total range the steering wheel can rotate
  double steeringRange = this->steeringJoint->GetHighStop(0).Radian() -
                         this->steeringJoint->GetLowStop(0).Radian();

  // Compute the angle ratio between the steering wheel and the tires
  this->steeringRatio = steeringRange / this->tireAngleRange;

  // Maximum gas is the upper limit of the gas joint
  this->maxGas = this->gasJoint->GetHighStop(0).Radian();

  // Maximum brake is the upper limit of the gas joint
  this->maxBrake = this->gasJoint->GetHighStop(0).Radian();

  printf("SteeringRation[%f] MaxGa[%f]\n", this->steeringRatio, this->maxGas);
}

/////////////////////////////////////////////////
void VehiclePlugin::OnUpdate()
{
  // Get the normalized gas and brake amount
  double gas = this->gasJoint->GetAngle(0).Radian() / this->maxGas;
  double brake = this->brakeJoint->GetAngle(0).Radian() / this->maxBrake;

  // A little force to push back on the pedals
  this->gasJoint->SetForce(0, -0.1);
  this->brakeJoint->SetForce(0, -0.1);

  // Get the steering angle
  double steeringAngle = this->steeringJoint->GetAngle(0).Radian();

  // Compute the angle of the front wheels.
  double wheelAngle = steeringAngle / this->steeringRatio;

  // double idleSpeed = 0.5;

  // Compute the rotational velocity of the wheels
  double jointVel = (std::max(0.0, gas-brake) * this->maxSpeed) /
                    this->wheelRadius;

  // Set velocity and max force for each wheel
  this->joints[0]->SetVelocity(1, -jointVel);
  this->joints[0]->SetMaxForce(1, (gas + brake) * this->frontPower);

  this->joints[1]->SetVelocity(1, -jointVel);
  this->joints[1]->SetMaxForce(1, (gas + brake) * this->frontPower);

  this->joints[2]->SetVelocity(1, -jointVel);
  this->joints[2]->SetMaxForce(1, (gas + brake) * this->rearPower);

  this->joints[3]->SetVelocity(1, -jointVel);
  this->joints[3]->SetMaxForce(1, (gas + brake) * this->rearPower);

  // Set the front-left wheel angle
  this->joints[0]->SetLowStop(0, wheelAngle);
  this->joints[0]->SetHighStop(0, wheelAngle);
  this->joints[0]->SetLowStop(0, wheelAngle);
  this->joints[0]->SetHighStop(0, wheelAngle);

  // Set the front-right wheel angle
  this->joints[1]->SetHighStop(0, wheelAngle);
  this->joints[1]->SetLowStop(0, wheelAngle);
  this->joints[1]->SetHighStop(0, wheelAngle);
  this->joints[1]->SetLowStop(0, wheelAngle);

  // Get the current velocity of the car
  this->velocity = this->chassis->GetWorldLinearVel();

  //  aerodynamics
  this->chassis->AddForce(
      math::Vector3(0, 0, this->aeroLoad * this->velocity.GetSquaredLength()));

  // Sway bars
  math::Vector3 bodyPoint;
  math::Vector3 hingePoint;
  math::Vector3 axis;

  for (int ix = 0; ix < 4; ++ix)
  {
    hingePoint = this->joints[ix]->GetAnchor(0);
    bodyPoint = this->joints[ix]->GetAnchor(1);

    axis = this->joints[ix]->GetGlobalAxis(0).Round();
    double displacement = (bodyPoint - hingePoint).Dot(axis);

    float amt = displacement * this->swayForce;
    if (displacement > 0)
    {
      if (amt > 15)
        amt = 15;

      math::Pose p = this->joints[ix]->GetChild()->GetWorldPose();
      this->joints[ix]->GetChild()->AddForce(axis * -amt);
      this->chassis->AddForceAtWorldPosition(axis * amt, p.pos);

      p = this->joints[ix^1]->GetChild()->GetWorldPose();
      this->joints[ix^1]->GetChild()->AddForce(axis * amt);
      this->chassis->AddForceAtWorldPosition(axis * -amt, p.pos);
    }
  }
}

/////////////////////////////////////////////////
void VehiclePlugin::OnVelMsg(ConstPosePtr &/*_msg*/)
{
}
