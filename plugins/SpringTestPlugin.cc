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

  this->k =
    _sdf->GetElement("k")->Get<double>();

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&SpringTestPlugin::OnUpdate, this));
}

/////////////////////////////////////////////////
void SpringTestPlugin::Init()
{
}

/////////////////////////////////////////////////
void SpringTestPlugin::OnUpdate()
{
  common::Time currTime = this->model->GetWorld()->GetSimTime();
  common::Time stepTime = currTime - this->prevUpdateTime;
  this->prevUpdateTime = currTime;

  physics::JointPtr joint = this->model->GetJoint("joint_0");
  {
    double pos = joint->GetAngle(0).Radian();
    double force = -this->k * pos;
    gzdbg << "joint pos [" << pos << "] force [" << force << "]\n";
    joint->SetForce(0, force);
  }
}
