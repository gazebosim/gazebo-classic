/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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

#include "physics/physics.h"
#include "transport/transport.h"
#include "plugins/JointPIDControl.hh"

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(JointPIDControlPlugin)

enum {RIGHT, LEFT};

/////////////////////////////////////////////////
JointPIDControlPlugin::JointPIDControlPlugin()
  : leftPID(0.01, 0.0, 0.001), rightPID(0.01, 0.0, 0.001)
{
  this->wheelSpeed[LEFT] = this->wheelSpeed[RIGHT] = 0;
}

/////////////////////////////////////////////////
void JointPIDControlPlugin::Load(physics::ModelPtr _model,
                           sdf::ElementPtr _sdf)
{
  this->model = _model;

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->model->GetWorld()->GetName());

  this->velSub = this->node->Subscribe(std::string("~/") +
      this->model->GetName() + "/vel_cmd", &JointPIDControlPlugin::OnVelMsg, this);

  if (!_sdf->HasElement("left_joint"))
    gzerr << "DiffDrive plugin missing <left_joint> element\n";

  if (!_sdf->HasElement("right_joint"))
    gzerr << "DiffDrive plugin missing <right_joint> element\n";

  this->leftJoint = _model->GetJoint(
      _sdf->GetElement("left_joint")->GetValueString());
  this->rightJoint = _model->GetJoint(
      _sdf->GetElement("right_joint")->GetValueString());

  if (_sdf->HasElement("torque"))
    this->torque = _sdf->GetElement("torque")->GetValueDouble();
  else
  {
    gzwarn << "No torque value set for the DiffDrive plugin.\n";
    this->torque = 5.0;
  }

  if (!this->leftJoint)
    gzerr << "Unable to find left joint["
          << _sdf->GetElement("left_joint")->GetValueString() << "]\n";
  if (!this->rightJoint)
    gzerr << "Unable to find right joint["
          << _sdf->GetElement("right_joint")->GetValueString() << "]\n";

  this->updateConnection = event::Events::ConnectWorldUpdateStart(
          boost::bind(&JointPIDControlPlugin::OnUpdate, this));
}

/////////////////////////////////////////////////
void JointPIDControlPlugin::Init()
{
  this->wheelSeparation = this->leftJoint->GetAnchor(0).Distance(
      this->rightJoint->GetAnchor(0));

  physics::EntityPtr parent = boost::shared_dynamic_cast<physics::Entity>(
      this->leftJoint->GetChild());

  math::Box bb = parent->GetBoundingBox();
  math::Vector3 size = bb.GetSize() * this->leftJoint->GetLocalAxis(0);

  this->wheelRadius = (bb.GetSize().GetSum() - size.GetSum()) * 0.5;

  // this->wheelSpeed[LEFT] = 0.2;
  // this->wheelSpeed[RIGHT] = 0.2;
}

/////////////////////////////////////////////////
void JointPIDControlPlugin::OnVelMsg(ConstPosePtr &_msg)
{
  double vr, va;

  vr = _msg->position().x();
  va =  msgs::Convert(_msg->orientation()).GetAsEuler().z;

  this->wheelSpeed[LEFT] = vr + va * this->wheelSeparation / 2;
  this->wheelSpeed[RIGHT] = vr - va * this->wheelSeparation / 2;
}

/////////////////////////////////////////////////
void JointPIDControlPlugin::OnUpdate()
{
  /* double d1, d2;
  double dr, da;

  this->prevUpdateTime = currTime; 

  // Distance travelled by front wheels
  d1 = stepTime.Double() * this->wheelRadius * this->leftJoint->GetVelocity(0);
  d2 = stepTime.Double() * this->wheelRadius * this->rightJoint->GetVelocity(0);

  dr = (d1 + d2) / 2;
  da = (d1 - d2) / this->wheelSeparation;
  */
  common::Time currTime = this->model->GetWorld()->GetSimTime();
  common::Time stepTime = currTime - this->prevUpdateTime;

  double leftVel = this->leftJoint->GetVelocity(0);
  double rightVel = this->rightJoint->GetVelocity(0);

  double leftVelDesired = (this->wheelSpeed[LEFT] / this->wheelRadius);
  double rightVelDesired = (this->wheelSpeed[RIGHT] / this->wheelRadius);

  double leftErr = leftVel - leftVelDesired;
  double rightErr = rightVel - rightVelDesired;

  double leftForce = this->leftPID.Update(leftErr, stepTime);
  double rightForce = this->rightPID.Update(rightErr, stepTime);

  if (leftForce < -this->torque)
    leftForce = -this->torque;
  if (leftForce > this->torque)
    leftForce = this->torque;

  if (rightForce < -this->torque)
    rightForce = -this->torque;
  if (rightForce > this->torque)
    rightForce = this->torque;

  // printf("LV[%7.4f] LD[%7.4f] LF[%7.4f] RV[%7.4f] RD[%7.4f] RF[%7.4f]\n",
  // leftVel, leftVelDesired, leftForce, rightVel, rightVelDesired, rightForce);
  this->leftJoint->SetForce(0, leftForce);
  this->rightJoint->SetForce(0, rightForce);
}
