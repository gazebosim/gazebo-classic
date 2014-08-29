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

#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"
#include "plugins/GazeboPlugin.hh"

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(GazeboPlugin)

/////////////////////////////////////////////////
GazeboPlugin::GazeboPlugin()
{
  std::vector<std::string> v;
  v.push_back("--verbose");

  // use a different socket/port
  setenv("GAZEBO_MASTER_URI", "http://localhost:11346", 1);

  // Initialize gazebo.
  gazebo::setupServer(v);

  // Load a world
  this->world = gazebo::loadWorld("worlds/simple_arm.world");
}

/////////////////////////////////////////////////
GazeboPlugin::~GazeboPlugin()
{
  // Close everything.
  gazebo::shutdown();
}

/////////////////////////////////////////////////
void GazeboPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
{
  this->model = _model;

  this->connections.push_back(event::Events::ConnectWorldUpdateBegin(
          boost::bind(&GazeboPlugin::OnUpdate, this)));

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->model->GetWorld()->GetName());

  this->sub = this->node->Subscribe(std::string("~/") +
      this->model->GetName() + "/cmd", &GazeboPlugin::OnMsg, this);
}

/////////////////////////////////////////////////
void GazeboPlugin::Init()
{
}

/////////////////////////////////////////////////
void GazeboPlugin::OnUpdate()
{
  // Run simulation for 100 steps.
  gazebo::runWorld(this->world, 100);
}

/////////////////////////////////////////////////
void GazeboPlugin::OnMsg(ConstPosePtr &/*_msg*/)
{
}
