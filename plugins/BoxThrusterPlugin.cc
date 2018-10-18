/*
 * Copyright (C) 2014-2015 Open Source Robotics Foundation
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
#include "gazebo/common/Events.hh"
#include "gazebo/physics/Link.hh"
#include "gazebo/physics/Model.hh"
#include "gazebo/physics/World.hh"
#include "plugins/BoxThrusterPlugin.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(BoxThrusterPlugin)

/////////////////////////////////////////////////
BoxThrusterPlugin::BoxThrusterPlugin()
  : thrustMagnitude(0.0)
{
}

/////////////////////////////////////////////////
BoxThrusterPlugin::~BoxThrusterPlugin()
{
}

/////////////////////////////////////////////////
void BoxThrusterPlugin::Load(physics::ModelPtr _model,
                     sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model, "_model pointer is NULL");
  this->model = _model;
  this->sdf = _sdf;

  this->world = _model->GetWorld();
  this->link = this->model->GetLink();

  if (this->sdf->HasElement("thrust_magnitude"))
  {
    this->thrustMagnitude = this->sdf->Get<double>("thrust_magnitude");
  }

  this->Reset();
}

/////////////////////////////////////////////////
void BoxThrusterPlugin::Init()
{
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&BoxThrusterPlugin::OnUpdate, this));
}

/////////////////////////////////////////////////
void BoxThrusterPlugin::OnUpdate()
{
  if (!this->world)
  {
    gzerr << "Invalid world pointer" << std::endl;
    return;
  }
  double t = this->world->GetSimTime().Double();
  double thrust = this->thrustMagnitude * 0.5*(1 - cos( M_PI * t));

  if (!this->link)
  {
    gzerr << "Invalid link pointer" << std::endl;
    return;
  }
  link->AddLinkForce(math::Vector3(0, 0, thrust));
}

/////////////////////////////////////////////////
void BoxThrusterPlugin::Reset()
{
  if (this->sdf->HasElement("linear"))
  {
    math::Vector3 linear = this->sdf->Get<math::Vector3>("linear");
    this->model->SetLinearVel(linear);
  }
  if (this->sdf->HasElement("angular"))
  {
    math::Vector3 angular = this->sdf->Get<math::Vector3>("angular");
    this->model->SetAngularVel(angular);
  }
}
