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

#include <list>
#include <mutex>

#include <ignition/math/Plane.hh>
#include <ignition/math/Vector3.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/PhysicsIface.hh>
#include <gazebo/common/Events.hh>

#include "plugins/SASCPlugin.hh"

using namespace gazebo;

GZ_REGISTER_WORLD_PLUGIN(SASCPlugin)

class gazebo::SASCPluginPrivate
{
  /// \brief Pointer to the update event connection.
  public: event::ConnectionPtr updateConnection;

  /// \brief All the blue vehicles that need to be launched
  public: std::list<physics::ModelPtr> blueLaunchQueue;
  public: std::list<physics::JointPtr> blueLaunchJoints;
  public: physics::ModelPtr blueLaunched;
  public: gazebo::common::Time blueLaunchTime;

  /// \brief All the gold vehicles that need to be launched
  public: std::list<physics::ModelPtr> goldLaunchQueue;
  public: std::list<physics::JointPtr> goldLaunchJoints;
  public: physics::ModelPtr goldLaunched;
  public: gazebo::common::Time goldLaunchTime;

  public: transport::NodePtr node;
  public: transport::SubscriberPtr newEntitySub;
  public: std::list<msgs::Model> modelMsgs;
  public: std::mutex mutex;
  public: std::set<std::string> drones;

  public: double interval = 0;
  public: double distance = 0;
  public: physics::WorldPtr world;
};

/////////////////////////////////////////////////
SASCPlugin::SASCPlugin()
  : dataPtr(new SASCPluginPrivate)
{
}

/////////////////////////////////////////////////
SASCPlugin::~SASCPlugin()
{
}

/////////////////////////////////////////////////
void SASCPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{

  this->dataPtr->interval = 30;
  if (_sdf->HasElement("interval"))
  {
    this->dataPtr->interval = _sdf->Get<double>("interval");
  }

  // currently not used.
  this->dataPtr->distance = 0;
  if (_sdf->HasElement("distance"))
  {
    this->dataPtr->distance = _sdf->Get<double>("distance");
  }

  this->dataPtr->world = _world;

  this->dataPtr->blueLaunchTime =
    _world->SimTime() + gazebo::common::Time(this->dataPtr->interval, 0);
  this->dataPtr->goldLaunchTime =
    _world->SimTime() + gazebo::common::Time(this->dataPtr->interval, 0);

  // listen to new model msgs
  this->dataPtr->node.reset(new transport::Node());
  this->dataPtr->node->Init();
  this->dataPtr->newEntitySub = this->dataPtr->node->Subscribe("~/model/info",
      &SASCPlugin::OnModel, this, true);

  // Listen to the update event. This event is broadcast every simulation
  // iteration.
  this->dataPtr->updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&SASCPlugin::OnUpdate, this));
}

/////////////////////////////////////////////////
void SASCPlugin::Init()
{
}

/////////////////////////////////////////////////
void SASCPlugin::OnUpdate()
{
  // look out for new models
  this->ProcessModelMsgs();

  auto simTime = this->dataPtr->world->SimTime();

  if (simTime >= this->dataPtr->blueLaunchTime &&
      !this->dataPtr->blueLaunchQueue.empty())
  {
    if (!this->dataPtr->blueLaunchQueue.empty())
    {
      this->dataPtr->blueLaunched = this->dataPtr->blueLaunchQueue.front();
      this->dataPtr->blueLaunchQueue.pop_front();

      auto joint = this->dataPtr->blueLaunchJoints.front();
      this->dataPtr->blueLaunchJoints.pop_front();

      std::string jointName = joint->GetName();

      joint.reset();
      this->dataPtr->blueLaunched->RemoveJoint(jointName);

      this->dataPtr->blueLaunchTime = simTime +
          gazebo::common::Time(this->dataPtr->interval, 0);
    }
  }

  if (this->dataPtr->world->SimTime() >= this->dataPtr->goldLaunchTime)
  {
    if (!this->dataPtr->goldLaunchQueue.empty())
    {
      this->dataPtr->goldLaunched = this->dataPtr->goldLaunchQueue.front();
      this->dataPtr->goldLaunchQueue.pop_front();

      auto joint = this->dataPtr->goldLaunchJoints.front();
      this->dataPtr->goldLaunchJoints.pop_front();

      std::string jointName = joint->GetName();

      joint.reset();
      this->dataPtr->goldLaunched->RemoveJoint(jointName);

      this->dataPtr->goldLaunchTime = simTime +
          gazebo::common::Time(this->dataPtr->interval, 0);
    }
  }
}

/////////////////////////////////////////////////
void SASCPlugin::ProcessModelMsgs()
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  while (!this->dataPtr->modelMsgs.empty())
  {
    auto msg = this->dataPtr->modelMsgs.front();
    this->dataPtr->modelMsgs.pop_front();

    if (msg.name().find("wing") == std::string::npos)
      continue;

    if (this->dataPtr->drones.find(msg.name()) != this->dataPtr->drones.end())
      continue;

    auto m = this->dataPtr->world->ModelByName(msg.name());
    if (m)
    {
      this->dataPtr->drones.insert(msg.name());
      this->AddModel(m);
    }
  }
}

/////////////////////////////////////////////////
void SASCPlugin::AddModel(physics::ModelPtr _model)
{
  std::string modelName = _model->GetScopedName();

  _model->GetLink()->SetAutoDisable(false);
  physics::JointPtr joint = _model->CreateJoint(
       modelName + "_launch_joint", "revolute", nullptr,
      _model->GetLink());
  joint->SetAxis(0, ignition::math::Vector3d::UnitZ);
  joint->SetUpperLimit(0, 0);
  joint->SetLowerLimit(0, 0);
  joint->Init();

  size_t idx = modelName.rfind("_");
  if (idx == std::string::npos)
  {
    std::cerr << modelName << ": no suffix" << std::endl;
    return;
  }

  std::string suffix = modelName.substr(idx+1);
  int n = -1;
  try
  {
    n = std::stoi(suffix);
  }
  catch (...)
  {
    std::cerr << modelName << ": suffix is not a number" << std::endl;
    return;
  }

  // ids for fixed wings in blue team
  if (n >= 26 && n <= 50)
  {
    this->dataPtr->blueLaunchQueue.push_back(_model);
    this->dataPtr->blueLaunchJoints.push_back(joint);
  }
  // ids for fixed wings in gold team
  else if (n >= 126 && n <= 150)
  {
    this->dataPtr->goldLaunchQueue.push_back(_model);
    this->dataPtr->goldLaunchJoints.push_back(joint);
  }
}

/////////////////////////////////////////////////
void SASCPlugin::OnModel(ConstModelPtr &_msg)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->modelMsgs.push_back(*_msg.get());
}

