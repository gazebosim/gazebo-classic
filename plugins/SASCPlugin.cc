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
#include <ignition/math/Plane.hh>
#include <ignition/math/Vector3.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Model.hh>
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

  public: ignition::math::Planed blueLaunchBoundaryPlane;
  public: ignition::math::Planed goldLaunchBoundaryPlane;

  public: physics::WorldPtr world;
};

/////////////////////////////////////////////////
SASCPlugin::SASCPlugin()
  : dataPtr(new SASCPluginPrivate)
{
  this->dataPtr->blueLaunchBoundaryPlane.Set(
      ignition::math::Vector3d(-1, 0, 0), 0.0);
  this->dataPtr->goldLaunchBoundaryPlane.Set(
      ignition::math::Vector3d(1, 0, 0), 500.0);
}

/////////////////////////////////////////////////
SASCPlugin::~SASCPlugin()
{
}

/////////////////////////////////////////////////
void SASCPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr /*_sdf*/)
{
  this->dataPtr->world = _world;

  // Collect pointers to all the vehicles.
  physics::Model_V models = _world->Models();
  for (auto model : models)
  {
    std::string modelName = model->ScopedName();
    if (modelName.find("blue") == std::string::npos ||
        modelName.find("gold") == std::string::npos)
    {
      continue;
    }

    physics::JointPtr joint = model->CreateJoint(
        modelName + "_launch_joint", "revolute", nullptr,
        model->GetLink());
    joint->SetAxis(0, ignition::math::Vector3d::UnitZ);
    joint->SetUpperLimit(0);
    joint->SetLowerLimit(0);

    if (modelName.find("blue") != std::string::npos)
    {
      this->dataPtr->blueLaunchQueue.push_back(model);
      this->dataPtr->blueLaunchJoints.push_back(joint);
    }
    else if (model->GetName().find("gold") != std::string::npos)
    {
      this->dataPtr->goldLaunchQueue.push_back(model);
      this->dataPtr->goldLaunchJoints.push_back(joint);
    }
  }

  this->dataPtr->blueLaunchTime =
    _world->SimTime() + gazebo::common::Time(30, 0);
  this->dataPtr->goldLaunchTime =
    _world->SimTime() + gazebo::common::Time(30, 0);

  // Listen to the update event. This event is broadcast every simulation
  // iteration.
  this->dataPtr->updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&SASCPlugin::OnUpdate, this));
}

/////////////////////////////////////////////////
void SASCPlugin::Init()
{
  for (auto joint : this->dataPtr->blueLaunchJoints)
    joint->Init();
  for (auto joint : this->dataPtr->goldLaunchJoints)
    joint->Init();
}

/////////////////////////////////////////////////
void SASCPlugin::OnUpdate()
{
  auto simTime = this->dataPtr->world->SimTime();

  if (simTime >= this->dataPtr->blueLaunchTime &&
      !this->dataPtr->blueLaunchQueue.empty())
  {
    if (!this->dataPtr->blueLaunched ||
        this->dataPtr->blueLaunchBoundaryPlane.Side(
          this->dataPtr->blueLaunched->WorldPose().Pos()) ==
        ignition::math::Planed::POSITIVE_SIDE)
    {
      this->dataPtr->blueLaunched = this->dataPtr->blueLaunchQueue.front();
      this->dataPtr->blueLaunchQueue.pop_front();

      auto joint = this->dataPtr->blueLaunchJoints.front();
      this->dataPtr->blueLaunchJoints.pop_front();

      std::string jointName = joint->GetName();

      joint.reset();
      this->dataPtr->blueLaunched->RemoveJoint(jointName);

      this->dataPtr->blueLaunchTime = simTime + gazebo::common::Time(30, 0);
    }
  }

  if (this->dataPtr->world->SimTime() >= this->dataPtr->goldLaunchTime)
  {
    if (!this->dataPtr->goldLaunched ||
        this->dataPtr->goldLaunchBoundaryPlane.Side(
          this->dataPtr->goldLaunched->WorldPose().Pos()) ==
        ignition::math::Planed::POSITIVE_SIDE)
    {
      this->dataPtr->goldLaunched = this->dataPtr->goldLaunchQueue.front();
      this->dataPtr->goldLaunchQueue.pop_front();

      auto joint = this->dataPtr->blueLaunchJoints.front();
      this->dataPtr->blueLaunchJoints.pop_front();

      std::string jointName = joint->GetName();

      joint.reset();
      this->dataPtr->blueLaunched->RemoveJoint(jointName);

      this->dataPtr->goldLaunchTime = simTime + gazebo::common::Time(30, 0);
    }
  }
}
