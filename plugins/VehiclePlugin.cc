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
#include "plugins/VehiclePlugin.hh"

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(VehiclePlugin)

/////////////////////////////////////////////////
VehiclePlugin::VehiclePlugin()
{
  this->joints.resize(4);
}

/////////////////////////////////////////////////
void VehiclePlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  this->model = _model;
  // this->physics = this->model->GetWorld()->GetPhysicsEngine();

  this->chassis = this->model->GetLink(
      _sdf->GetElement("chassis")->GetValueString());
  this->joints[0] = this->model->GetJoint(
      _sdf->GetElement("front_left")->GetValueString());
  this->joints[1] = this->model->GetJoint(
      _sdf->GetElement("front_right")->GetValueString());
  this->joints[2] = this->model->GetJoint(
      _sdf->GetElement("back_left")->GetValueString());
  this->joints[3] = this->model->GetJoint(
      _sdf->GetElement("back_right")->GetValueString());

  if (!this->chassis)
  {
    gzerr << "Unable to find chassis[" << _sdf->GetElement("chassis") << "]\n";
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

  std::cout << "Chassis[" << this->chassis->GetScopedName() << "]\n";
  for (int i = 0; i < 4; ++i)
    std::cout << "Joint[" << this->joints[i]->GetScopedName() << "]\n";

  this->connections.push_back(event::Events::ConnectWorldUpdateStart(
          boost::bind(&VehiclePlugin::OnUpdate, this)));

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->model->GetWorld()->GetName());

  this->velSub = this->node->Subscribe(std::string("~/") +
      this->model->GetName() + "/vel_cmd", &VehiclePlugin::OnVelMsg, this);
}

/////////////////////////////////////////////////
void VehiclePlugin::Init()
{
}

/////////////////////////////////////////////////
void VehiclePlugin::OnUpdate()
{
  this->joints[0]->SetVelocity(0, 1.0);
  this->joints[0]->SetMaxForce(0, 10);

  this->joints[1]->SetVelocity(0, 1.0);
  this->joints[1]->SetMaxForce(0, 10);

  this->joints[2]->SetVelocity(0, 1.0);
  this->joints[2]->SetMaxForce(0, 10);

  this->joints[3]->SetVelocity(0, 1.0);
  this->joints[3]->SetMaxForce(0, 10);

  this->velocity = this->chassis->GetWorldLinearVel();
  // std::cout << "Velocity[" << this->velocity << "]\n";
  // double AERO_LOAD = -0.1;
  // double SWAY_FORCE = 10;

  // std::cout << "Aero[" << AERO_LOAD * this->velocity.GetSquaredLength() << "]\n";

  //  aerodynamics
  /*this->chassis->AddForce(math::Vector3(0, 0,
        AERO_LOAD * this->velocity.GetSquaredLength()));

  // Sway bars
  math::Vector3 bodyPoint;
  math::Vector3 hingePoint;
  math::Vector3 axis;

  double displacement;

  printf("Sway bars\n");
  for (int ix = 0; ix < 4; ++ix)
  {
    hingePoint = this->joints[ix]->GetAnchor(0);
    // dJointGetHinge2Anchor(hinges_[ix], &hingePoint.x);

    bodyPoint = this->joints[ix]->GetAnchor(1);
    // dJointGetHinge2Anchor2(hinges_[ix], &bodyPoint.x);

    axis = this->joints[ix]->GetGlobalAxis(0);
    //dJointGetHinge2Axis1(hinges_[ix], &axis.x);

    displacement = (bodyPoint - hingePoint).GetDotProd(axis);

    std::cout << "Displacement[" << displacement << "]\n";

    float amt = displacement * SWAY_FORCE;
    if( displacement > 0 )
    {
      if( amt > 15 )
        amt = 15;

      this->joints[ix]->GetChild()->AddForce(axis * (amt * -1));
      this->joints[ix^1]->GetChild()->AddForce(axis * amt);

      //dBodyAddForce( wheelBody_[ix], -axis.x * amt, -axis.y * amt, -axis.z * amt );

      //math::Pose p = this->joints[ix]->GetChild()->GetWorldPose();      
      // dReal const * wp = dBodyGetPosition( wheelBody_[ix] );
      
      //this->chassis->AddForceAtRelativePosition(amt*axis, p.pos);
      //dBodyAddForceAtPos( chassisBody_, axis.x*amt, axis.y*amt, axis.z*amt, wp[0], wp[1], wp[2] );
      
      //dBodyAddForce( wheelBody_[ix^1], axis.x * amt, axis.y * amt, axis.z * amt );
      
      //wp = dBodyGetPosition( wheelBody_[ix] );

      //this->chassis->AddForceAtRelativePosition(-amt*axis, p.pos);
      //dBodyAddForceAtPos( chassisBody_, -axis.x*amt, -axis.y*amt, -axis.z*amt, wp[0], wp[1], wp[2] );
    }
  }
  */
}

/////////////////////////////////////////////////
void VehiclePlugin::OnVelMsg(ConstPosePtr &_msg)
{
}
