/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Inertial.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <ignition/math/Pose3.hh>

#include "MoveCoGPlugin.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(MoveCoGPlugin)

/////////////////////////////////////////////////
void MoveCoGPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
{
  GZ_ASSERT(_model, "Model is null");
  this->model = _model;
}

/////////////////////////////////////////////////
void MoveCoGPlugin::Init()
{
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&MoveCoGPlugin::OnUpdate, this, std::placeholders::_1));
}

/////////////////////////////////////////////////
void MoveCoGPlugin::OnUpdate(const common::UpdateInfo &_info)
{
  if (_info.simTime < 10)
    return;

  physics::LinkPtr link = this->model->GetLink("link");
  GZ_ASSERT(link, "Link is null");
  physics::InertialPtr inertial = link->GetInertial();
  GZ_ASSERT(inertial, "Inertial is null");
  ignition::math::Pose3d pose = inertial->Pose();
  pose.Pos().Y() = -2.0;
  inertial->SetCoG(pose);
  link->UpdateMass();

  this->updateConnection.reset();
}
