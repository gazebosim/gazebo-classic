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

#include <functional>
#include <string>
#include <gazebo/common/Assert.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/sensors/SensorManager.hh>
#include <sdf/sdf.hh>

#include "TouchPlugin.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(TouchPlugin)

/////////////////////////////////////////////////
TouchPlugin::TouchPlugin()
{
}

/////////////////////////////////////////////////
void TouchPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model, "Model pointer is null");
  GZ_ASSERT(_sdf, "SDF pointer is null");

  // Get sensors
  if (_sdf->HasElement("sensor"))
  {
    auto elem = _sdf->GetElement("sensor");
    while (elem)
    {
      auto sensorName = elem->Get<std::string>();

      auto sensor = std::dynamic_pointer_cast<sensors::ContactSensor>(
          sensors::SensorManager::Instance()->GetSensor(sensorName));

      if (!sensor)
      {
        gzerr << "Couldn't find sensor [" << sensorName << "]" << std::endl;
        return;
      }

      this->contactSensors.push_back(sensor);

      elem = elem->GetNextElement("sensor");
    }
  }
  else
  {
    gzerr << "Missing required parameter <sensor>." << std::endl;
    return;
  }

  // Get target substring
  if (!_sdf->HasElement("target"))
  {
    gzerr << "Missing required parameter <target>." << std::endl;
    return;
  }
  this->target = _sdf->GetElement("target")->Get<std::string>();
  this->modelName = _model->GetName();

  // Namespace
  if (!_sdf->HasElement("namespace"))
  {
    gzerr << "Missing required parameter <namespace>" << std::endl;
    return;
  }
  this->ns = _sdf->Get<std::string>("namespace");

  // Target time
  if (!_sdf->HasElement("time"))
  {
    gzerr << "Missing required parameter <time>" << std::endl;
    return;
  }
  this->targetTime = common::Time(_sdf->Get<double>("time"));

  // Start/stop "service"
  this->gzNode = transport::NodePtr(new transport::Node());
  this->gzNode->Init();
  this->enableSub = this->gzNode->Subscribe("/" + this->ns + "/enable",
      &TouchPlugin::Enable, this);

  // Start enabled or not
  auto enabled = _sdf->HasElement("enabled") && _sdf->Get<bool>("enabled");
  if (enabled)
  {
    boost::shared_ptr<msgs::Int> msg(new msgs::Int());
    msg->set_data(1);
    this->Enable(msg);
  }
}

//////////////////////////////////////////////////
void TouchPlugin::Enable(ConstIntPtr &_msg)
{
  // Start
  if (_msg->data() == 1 && !this->touchedPub)
  {
    // Start update
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&TouchPlugin::OnUpdate, this, std::placeholders::_1));

    this->touchedPub = this->gzNode->Advertise<msgs::Int>(
        "/" + this->ns + "/touched");

    for (auto s : this->contactSensors)
      s->SetActive(true);

    this->touchStart = common::Time::Zero;

    gzmsg << "Started touch plugin [" << this->ns << "]" << std::endl;
  }
  // Stop
  else if (_msg->data() == 0 && this->touchedPub)
  {
    this->updateConnection.reset();
    this->touchedPub->Fini();
    this->touchedPub.reset();

    for (auto s : this->contactSensors)
      s->SetActive(false);

    gzmsg << "Stopped touch plugin [" << this->ns << "]" << std::endl;
  }
  // Do nothing if not changing state
}

/////////////////////////////////////////////////
void TouchPlugin::OnUpdate(const common::UpdateInfo &_info)
{
  // Get all contacts across all sensors
  msgs::Contacts contacts;

  for (const auto &s : this->contactSensors)
    contacts.MergeFrom(s->Contacts());

  bool touching = false;

  for (int i = 0; i < contacts.contact_size(); ++i)
  {
    // Check for the target
    bool col1Target = contacts.contact(i).collision1().find(this->target) !=
        std::string::npos;
    bool col2Target = contacts.contact(i).collision2().find(this->target) !=
        std::string::npos;

    if (col1Target || col2Target)
      touching = true;

    // Check for this model
    bool col1Model = contacts.contact(i).collision1().find(this->modelName) !=
        std::string::npos;
    bool col2Model = contacts.contact(i).collision2().find(this->modelName) !=
        std::string::npos;

    // If the collisions are not target-model or model-target, we're touching
    // something else
    if (!((col1Target && col2Model) || (col1Model && col2Target)))
    {
      if (touchStart != common::Time::Zero)
      {
        gzmsg << "Touched something besides [" << this->target << "]"
              << std::endl;
      }
      this->touchStart = common::Time::Zero;
      return;
    }
  }

  // Free-fall is not touching
  if (!touching)
  {
    // Sanity check
    if (contacts.contact_size() > 0)
    {
      gzerr << "Not touching target, but touching something? "
            << "We shouldn't reach this point!" << std::endl;
    }

    if (this->touchStart != common::Time::Zero)
    {
      gzmsg << "Not touching anything" << std::endl;
    }
    this->touchStart = common::Time::Zero;
    return;
  }

  // Start touching
  if (this->touchStart == common::Time::Zero)
  {
    this->touchStart = _info.simTime;
    gzmsg << "Model [" << this->modelName << "] started touching ["
          << this->target << "] at " << this->touchStart << " seconds"
          << std::endl;
  }

  // Check if it has been touched for long enough
  auto completed = _info.simTime - this->touchStart > this->targetTime;

  // This is a single-use plugin. After touched, publish a message
  // and stop updating
  if (completed)
  {
    gzmsg << "Model [" << this->modelName << "] touched [" << this->target
          << "] exclusively for " << this->targetTime << " seconds"<< std::endl;

    gazebo::msgs::Int msg;
    msg.set_data(1);
    this->touchedPub->Publish(msg);

    // Disable
    boost::shared_ptr<msgs::Int> m(new msgs::Int());
    m->set_data(0);
    this->Enable(m);
  }
}

