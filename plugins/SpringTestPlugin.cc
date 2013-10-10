/*
 * Copyright 2012 Open Source Robotics Foundation
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
#include "plugins/SpringTestPlugin.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(SpringTestPlugin)

/////////////////////////////////////////////////
SpringTestPlugin::SpringTestPlugin()
{
}

/////////////////////////////////////////////////
void SpringTestPlugin::Load(physics::ModelPtr _model,
                           sdf::ElementPtr _sdf)
{
  this->model = _model;

  // hardcoded params for this test
  this->jointExplicit = this->model->GetJoint(
    _sdf->Get<std::string>("joint_explicit"));

  this->kpExplicit =
    _sdf->GetElement("kp_explicit")->Get<double>();

  this->kdExplicit =
    _sdf->GetElement("kd_explicit")->Get<double>();

  this->jointImplicit = this->model->GetJoint(
    _sdf->Get<std::string>("joint_implicit"));

  this->kpImplicit =
    _sdf->GetElement("kp_implicit")->Get<double>();

  this->kdImplicit =
    _sdf->GetElement("kd_implicit")->Get<double>();

  this->jointImplicit->SetStiffnessDamping(0, this->kpImplicit,
    this->kdImplicit);

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&SpringTestPlugin::ExplicitUpdate, this));
}

/////////////////////////////////////////////////
void SpringTestPlugin::Init()
{
}

/////////////////////////////////////////////////
void SpringTestPlugin::ExplicitUpdate()
{
  common::Time currTime = this->model->GetWorld()->GetSimTime();
  common::Time stepTime = currTime - this->prevUpdateTime;
  this->prevUpdateTime = currTime;

  double pos = this->jointExplicit->GetAngle(0).Radian();
  double vel = this->jointExplicit->GetVelocity(0);
  double force = -this->kpExplicit * pos
                 -this->kdExplicit * vel;
  this->jointExplicit->SetForce(0, force);
}
