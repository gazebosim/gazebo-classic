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
#include "plugins/ModelTrajectoryTestPlugin.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(ModelTrajectoryTestPlugin)

/////////////////////////////////////////////////
ModelTrajectoryTestPlugin::ModelTrajectoryTestPlugin()
{
}

/////////////////////////////////////////////////
void ModelTrajectoryTestPlugin::Load(physics::ModelPtr _model,
                           sdf::ElementPtr /*_sdf*/)
{
  this->model = _model;

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->model->GetWorld()->GetName());

  this->trajSub = this->node->Subscribe(std::string("~/") +
      this->model->GetName() + "/pose_trajectory",
      &ModelTrajectoryTestPlugin::OnPoseTrajectoryMsg, this);

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&ModelTrajectoryTestPlugin::OnUpdate, this));
}

/////////////////////////////////////////////////
void ModelTrajectoryTestPlugin::Init()
{
}

/////////////////////////////////////////////////
void ModelTrajectoryTestPlugin::OnPoseTrajectoryMsg(
    ConstPoseTrajectoryPtr &/*_msg*/)
{
}

/////////////////////////////////////////////////
void ModelTrajectoryTestPlugin::OnUpdate()
{
  common::Time currTime = this->model->GetWorld()->GetSimTime();
  common::Time stepTime = currTime - this->prevUpdateTime;
  this->prevUpdateTime = currTime;

  {
    // ignore everything else, get position and force only
    math::Pose pose;
    this->model->SetWorldPose(pose);
  }
}
