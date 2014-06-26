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
#include "plugins/DiffDrivePlugin.hh"

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(DiffDrivePlugin)

enum {RIGHT, LEFT};

/////////////////////////////////////////////////
DiffDrivePlugin::DiffDrivePlugin()
{
  this->wheelSpeed[LEFT] = this->wheelSpeed[RIGHT] = 0;
  // initialize with some default values
  this->leftPID.Init( 20, 0.001, 0.002, 0.5, -0.5, 5, -5);
  this->rightPID.Init(20, 0.001, 0.002, 0.5, -0.5, 5, -5);
  this->leftForce = 0;
  this->rightForce = 0;
}

/////////////////////////////////////////////////
void DiffDrivePlugin::Load(physics::ModelPtr _model,
                           sdf::ElementPtr _sdf)
{
  this->model = _model;

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->model->GetWorld()->GetName());

  this->velSub = this->node->Subscribe(std::string("~/") +
      this->model->GetName() + "/vel_cmd", &DiffDrivePlugin::OnVelMsg, this);

  if (!_sdf->HasElement("left_joint"))
    gzerr << "DiffDrive plugin missing <left_joint> element\n";

  if (!_sdf->HasElement("right_joint"))
    gzerr << "DiffDrive plugin missing <right_joint> element\n";

  this->leftJoint = _model->GetJoint(
      _sdf->GetElement("left_joint")->Get<std::string>());
  this->rightJoint = _model->GetJoint(
      _sdf->GetElement("right_joint")->Get<std::string>());

  if (_sdf->HasElement("torque"))
    this->torque = _sdf->GetElement("torque")->Get<double>();
  else
  {
    gzwarn << "No torque value set for the DiffDrive plugin.\n";
    this->torque = 5.0;
  }

  if (!this->leftJoint)
    gzerr << "Unable to find left joint["
          << _sdf->GetElement("left_joint")->Get<std::string>() << "]\n";
  if (!this->rightJoint)
    gzerr << "Unable to find right joint["
          << _sdf->GetElement("right_joint")->Get<std::string>() << "]\n";

  this->prevUpdateTime = this->model->GetWorld()->GetSimTime();

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&DiffDrivePlugin::OnUpdate, this));
}

/////////////////////////////////////////////////
void DiffDrivePlugin::Init()
{
  this->wheelSeparation = this->leftJoint->GetAnchor(0).Distance(
      this->rightJoint->GetAnchor(0));

  physics::EntityPtr parent = boost::dynamic_pointer_cast<physics::Entity>(
      this->leftJoint->GetChild());

  // get wheel radius
  this->wheelRadius = 0.0;

  // assume only one collision
  physics::CollisionPtr coll = this->leftJoint->GetChild()->GetCollisions()[0];

  if (coll)
  {
    physics::ShapePtr shape = coll->GetShape();

    if (shape)
    {
      if (shape->HasType(gazebo::physics::Base::CYLINDER_SHAPE))
      {
        physics::CylinderShape *cyl =
            static_cast<physics::CylinderShape*>(shape.get());
        this->wheelRadius = cyl->GetRadius();
      }
      else if (shape->HasType(physics::Base::SPHERE_SHAPE))
      {
        physics::SphereShape *sph =
            static_cast<physics::SphereShape*>(shape.get());
        this->wheelRadius = sph->GetRadius();
      }
      else
        gzerr << "wheel shape is neither cylinder nor sphere,"
              << " what's radius here?\n";
    }
    else
      gzerr << "wheel collision GetShape failed\n";
  }
  else
    gzerr << "wheel link GetCollision failed\n";

  gzdbg << "wheel radius [" << this->wheelRadius << "]\n";

}

/////////////////////////////////////////////////
void DiffDrivePlugin::OnVelMsg(ConstPosePtr &_msg)
{
  double vr, va;

  vr = _msg->position().x();
  va =  msgs::Convert(_msg->orientation()).GetAsEuler().z;

  this->wheelSpeed[LEFT] = vr + va * this->wheelSeparation / 2.0;
  this->wheelSpeed[RIGHT] = vr - va * this->wheelSeparation / 2.0;
}

/////////////////////////////////////////////////
void DiffDrivePlugin::OnUpdate()
{
  /* double d1, d2;
  double dr, da;

  this->prevUpdateTime = currTime;

  // Distance travelled by front wheels
  d1 = stepTime.Double() * this->wheelRadius * this->leftJoint->GetVelocity(0);
  d2 = stepTime.Double() * this->wheelRadius * this->rightJoint->GetVelocity(0);

  dr = (d1 + d2) / 2;
  da = (d1 - d2) / this->wheelSeparation;
  common::Time currTime = this->model->GetWorld()->GetSimTime();
  common::Time stepTime = currTime - this->prevUpdateTime;
  */

  double dt = (this->model->GetWorld()->GetSimTime()
    - this->prevUpdateTime).Double();
  if (dt > 0)
  {
    double leftVelDesired = (this->wheelSpeed[LEFT] / this->wheelRadius);
    double rightVelDesired = (this->wheelSpeed[RIGHT] / this->wheelRadius);

    double leftError = this->leftJoint->GetVelocity(0) - leftVelDesired;
    double rightError = this->rightJoint->GetVelocity(0) - rightVelDesired;

    this->leftForce = math::clamp(
      this->leftPID.Update(leftError, dt), -this->torque, this->torque);
    this->rightForce = math::clamp(
      this->rightPID.Update(rightError, dt), -this->torque, this->torque);

    this->prevUpdateTime = this->model->GetWorld()->GetSimTime();

    // gzdbg << this->leftJoint->GetVelocity(0)
    //       << " " << this->rightJoint->GetVelocity(0)
    //       << " " << leftVelDesired
    //       << " " << rightVelDesired
    //       << " " << this->leftForce
    //       << " " << this->rightForce
    //       << "\n";
  }

  // switch to set force based control
  this->leftJoint->SetForce(0, this->leftForce);
  this->rightJoint->SetForce(0, this->rightForce);
}
