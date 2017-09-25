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

#include <string>

#include "gazebo/common/Events.hh"
#include "gazebo/common/UpdateInfo.hh"

#include "gazebo/physics/Entity.hh"
#include "gazebo/physics/World.hh"

#include "gazebo/transport/Node.hh"

#include "BoxContainsPlugin.hh"

namespace gazebo
{
  /// \brief Private data class for the BoxContainsPlugin class
  class BoxContainsPluginPrivate
  {
    /// \brief Connection to world update.
    public: event::ConnectionPtr updateConnection;

    /// \brief Pointer to the world.
    public: physics::WorldPtr world;

    /// \brief Scoped name of the entity we're checking.
    public: std::string entityName;

    /// \brief Pointer to the entity we're checking.
    public: physics::EntityPtr entity;

    /// \brief Box representing the volume to check.
    public: ignition::math::OrientedBoxd box;

    /// \brief Gazebo transport node for communication.
    public: transport::NodePtr node;

    /// \brief Publisher which publishes contain / doesn't contain messages.
    public: transport::PublisherPtr containsPub;

    /// \brief Subscriber to enable messages.
    public: transport::SubscriberPtr enableSub;

    /// \brief Namespace for the topics:
    /// /<ns>/box/contains
    /// /<ns>/box/enable
    public: std::string ns;

    /// \brief Whether contains or not
    public: int contains = -1;
  };
}

using namespace gazebo;

GZ_REGISTER_WORLD_PLUGIN(BoxContainsPlugin)

/////////////////////////////////////////////////
BoxContainsPlugin::BoxContainsPlugin() : WorldPlugin(),
    dataPtr(new BoxContainsPluginPrivate)
{
}

/////////////////////////////////////////////////
void BoxContainsPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
  // Load SDF params
  if (!_sdf->HasElement("size"))
  {
    gzerr << "Missing required parameter <size>, plugin will not be "
          << "initialized." << std::endl;
    return;
  }

  if (!_sdf->HasElement("pose"))
  {
    gzerr << "Missing required parameter <pose>, plugin will not be "
          << "initialized." << std::endl;
    return;
  }

  if (!_sdf->HasElement("entity"))
  {
    gzerr << "Missing required parameter <entity>, plugin will not be "
          << "initialized." << std::endl;
    return;
  }

  if (!_sdf->HasElement("namespace"))
  {
    gzerr << "Missing required parameter <namespace>, plugin will not be "
          << "initialized." << std::endl;
    return;
  }

  auto size = _sdf->Get<ignition::math::Vector3d>("size");
  auto pose = _sdf->Get<ignition::math::Pose3d>("pose");
  this->dataPtr->entityName = _sdf->Get<std::string>("entity");
  this->dataPtr->ns = _sdf->Get<std::string>("namespace");

  this->dataPtr->box = ignition::math::OrientedBoxd(size, pose);

  this->dataPtr->world = _world;

  // Start/stop "service"
  this->dataPtr->node = transport::NodePtr(new transport::Node());
  this->dataPtr->node->Init();
  this->dataPtr->enableSub = this->dataPtr->node->Subscribe("/" +
      this->dataPtr->ns + "/box/enable", &BoxContainsPlugin::Enable, this);

  auto enabled = true;
  if (_sdf->HasElement("enabled"))
    enabled= _sdf->Get<bool>("enabled");

  if (enabled)
  {
    boost::shared_ptr<msgs::Int> msg(new msgs::Int());
    msg->set_data(1);
    this->Enable(msg);
  }
}

//////////////////////////////////////////////////
void BoxContainsPlugin::Enable(ConstIntPtr &_msg)
{
  // Start
  if (_msg->data() == 1 && !this->dataPtr->updateConnection)
  {
    this->dataPtr->entity = this->dataPtr->world->GetEntity(
        this->dataPtr->entityName);
    if (!this->dataPtr->entity)
    {
      gzerr << "Can't find entity[" << this->dataPtr->entityName <<
          "] in world. Failed to enable Box Plugin." << std::endl;
      return;
    }

    // Start update
    this->dataPtr->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&BoxContainsPlugin::OnUpdate, this, std::placeholders::_1));

    this->dataPtr->containsPub = this->dataPtr->node->Advertise<msgs::Int>(
        "/" + this->dataPtr->ns + "/box/contains");
    gzmsg << "Started box contains plugin [" << this->dataPtr->ns << "]" << std::endl;
  }
  // Stop
  else
  {
    this->dataPtr->updateConnection.reset();
    gzmsg << "Stopped box contains plugin [" << this->dataPtr->ns << "]" << std::endl;
  }
}

/////////////////////////////////////////////////
void BoxContainsPlugin::OnUpdate(const common::UpdateInfo &/*_info*/)
{
  // For safety
  if (!this->dataPtr->entity)
  {
    gzerr << "Entity is null" << std::endl;
    return;
  }

  auto pos = this->dataPtr->entity->GetWorldPose().Ign().Pos();
  auto containsNow = this->dataPtr->box.Contains(pos) ? 1 : 0;

  if (containsNow != this->dataPtr->contains)
  {
    this->dataPtr->contains = containsNow;

    msgs::Int msg;
    msg.set_data(this->dataPtr->contains);

    this->dataPtr->containsPub->Publish(msg);
  }
}

