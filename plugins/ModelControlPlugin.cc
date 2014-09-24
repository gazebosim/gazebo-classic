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

#include <boost/unordered_map.hpp>
#include <boost/unordered_set.hpp>

#include "gazebo/common/Assert.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/sensors/SensorManager.hh"
#include "gazebo/sensors/ContactSensor.hh"
#include "gazebo/transport/transport.hh"
#include "plugins/ModelControlPlugin.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(ModelControlPlugin)

/////////////////////////////////////////////////
ModelControlPlugin::ModelControlPlugin()
{
}

/////////////////////////////////////////////////
void ModelControlPlugin::Load(physics::ModelPtr _model,
                     sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model, "ModelControlPlugin _model pointer is NULL");
  this->model = _model;
  this->modelName = _model->GetName();
  this->sdf = _sdf;

  this->world = this->model->GetWorld();
  GZ_ASSERT(this->world, "ModelControlPlugin world pointer is NULL");

  this->physics = this->world->GetPhysicsEngine();
  GZ_ASSERT(this->physics, "ModelControlPlugin physics pointer is NULL");

  GZ_ASSERT(_sdf, "ModelControlPlugin _sdf pointer is NULL");

  // get a list of links
  if (_sdf->HasElement("link_name"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("link_name");
    while (elem)
    {
      // create a vector of empty links and joints
      links.push_back(this->model->GetLink(elem->Get<std::string>()));
      // joints.push_back(physics::JointPtr());
      elem = elem->GetNextElement("link_name");
    }
  }

  // get a list of joints
  if (_sdf->HasElement("joint_name"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("joint_name");
    while (elem)
    {
      // create a vector of empty joints and joints
      joints.push_back(this->model->GetJoint(elem->Get<std::string>()));
      // joints.push_back(physics::JointPtr());
      elem = elem->GetNextElement("joint_name");
    }
  }
}

/////////////////////////////////////////////////
void ModelControlPlugin::Init()
{
  this->node.reset(new transport::Node());
  this->node->Init(this->world->GetName());

  // subscribe to a topic
  std::string topic = std::string("~/") + this->modelName + "/control_request";
  this->pubControlRequest =
    this->node->Advertise<msgs::ControlRequest>(topic);

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&ModelControlPlugin::OnUpdate, this));
}

/////////////////////////////////////////////////
void ModelControlPlugin::PubControlRequest()
{
  // boost::mutex::scoped_lock lock(this->mutex); // for threading this...

  // do something with control and publish state and wait for command
}

/////////////////////////////////////////////////
void ModelControlPlugin::OnUpdate()
{
  double dt = this->physics->GetMaxStepSize();
  if (dt < 1e-6)
    dt = 1e-6;

  {
    boost::mutex::scoped_lock lock(this->mutex);

    // call every simulation step
    this->PubControlRequest();
  }
}
