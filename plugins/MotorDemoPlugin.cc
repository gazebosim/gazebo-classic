/*
 * Copyright (C) 2012-2013 Open Source Robotics Foundation
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
#include "gazebo/transport/transport.hh"
#include "plugins/MotorDemoPlugin.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(MotorDemoPlugin)

/////////////////////////////////////////////////
MotorDemoPlugin::MotorDemoPlugin() : stiffness(0.0), damping(100.0)
{
}

/////////////////////////////////////////////////
void MotorDemoPlugin::Load(physics::ModelPtr _model,
                     sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model, "MotorDemoPlugin _model pointer is NULL");
  this->model = _model;
  this->modelName = _model->GetName();
  this->sdf = _sdf;

  this->world = this->model->GetWorld();
  GZ_ASSERT(this->world, "MotorDemoPlugin world pointer is NULL");

  this->physics = this->world->GetPhysicsEngine();
  GZ_ASSERT(this->physics, "MotorDemoPlugin physics pointer is NULL");

  GZ_ASSERT(_sdf, "MotorDemoPlugin _sdf pointer is NULL");

  if (_sdf->HasElement("stiffness"))
    this->stiffness = _sdf->Get<double>("stiffness");

  if (_sdf->HasElement("damping"))
    this->damping = _sdf->Get<double>("damping");

  if (_sdf->HasElement("link_name"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("link_name");
    while (elem)
    {
      linkNames.push_back(elem->Get<std::string>());
      links.push_back(physics::LinkPtr());
      joints.push_back(physics::JointPtr());
      elem = elem->GetNextElement("link_name");
    }
  }
  GZ_ASSERT(linkNames.size() == links.size(),
    "Length of links data structure doesn't match linkNames");
  GZ_ASSERT(linkNames.size() == joints.size(),
    "Length of joints data structure doesn't match linkNames");
}

/////////////////////////////////////////////////
void MotorDemoPlugin::Init()
{
  for (unsigned int i = 0; i < this->linkNames.size(); ++i)
  {
    physics::LinkPtr link = boost::dynamic_pointer_cast<physics::Link>(
        this->world->GetEntity(this->linkNames[i]));

    if (!link)
      continue;

    // do something with this link?
  }

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&MotorDemoPlugin::OnUpdate, this));
}

/////////////////////////////////////////////////
void MotorDemoPlugin::OnUpdate()
{
  // Iterate through the list of allowed links
  std::vector<std::string>::iterator iterLinkName =
    this->linkNames.begin();
  std::vector<physics::LinkPtr>::iterator iterLink = this->links.begin();
  std::vector<physics::JointPtr>::iterator iterJoint = this->joints.begin();
  unsigned int countIters = 0;
  // Only check the length of the first iterator since we used a GZ_ASSERT
  // in the Load function to confirm the vectors have the same length
  while (iterLinkName != this->linkNames.end())
  {
    // do something to iterLink

    // Increment
    ++countIters;
    ++iterJoint;
    ++iterLink;
    ++iterLinkName;
  }
}
