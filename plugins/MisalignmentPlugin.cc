/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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

#include <ignition/math/Pose3.hh>

#include "gazebo/common/Events.hh"
#include "gazebo/common/UpdateInfo.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/Entity.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/transport/Node.hh"

#include "MisalignmentPlugin.hh"

namespace gazebo
{
  namespace physics
  {
    typedef boost::weak_ptr<physics::Entity> EntityWeakPtr;
  }

  /// \brief Private data class for the MisalignmentPlugin class
  class MisalignmentPluginPrivate
  {
    /// \brief Enable or disable the plugin
    /// \param[in] _msg integer message where non-zero value means enable
    public: void Enable(ConstIntPtr &_msg);

    /// \brief Called every world iteration on world update begin.
    /// \param[in] _info Update info.
    public: void OnUpdate(const common::UpdateInfo &_info);

    /// \brief Connection to world update.
    public: event::ConnectionPtr updateConnection;

    /// \brief Pointer to the world.
    public: physics::WorldPtr world;

    /// \brief scoped name of reference entity
    public: std::string referenceFrameName;

    /// \brief Scoped name of the second entity we're checking.
    public: std::string targetFrameName;

    /// \brief pose in referenceEntity frame
    public: ignition::math::Pose3d referencePose;

    /// \brief pose in entity frame
    public: ignition::math::Pose3d targetPose;

    /// \brief pointer to reference entity
    public: physics::EntityWeakPtr referenceEntity;

    /// \brief Pointer to the entity we're checking.
    public: physics::EntityWeakPtr targetEntity;

    /// \brief Gazebo transport node for communication.
    // TODO(sloretz) replace with ign-transport in gazebo8+
    public: transport::NodePtr gzNode;

    /// \brief True if already warned about missing entity
    public: bool didWarnTgt = false;

    /// \brief True if already warned about missing reference entity
    public: bool didWarnRef = false;

    /// \brief Publisher for misalignment
    public: transport::PublisherPtr pubMisalignment;

    /// \brief Debug publisher for reference pose in world frame
    public: transport::PublisherPtr pubDbgRefPose;

    /// \brief Debug publisher for entity pose in world frame
    public: transport::PublisherPtr pubDbgTgtPose;

    /// \brief Subscriber to enable messages.
    public: transport::SubscriberPtr enableGzSub;

    /// \brief Namespace for the topics
    public: std::string ns;

    /// \brief if true output debug info
    public: bool debug = false;

    /// \brief if true plugin is enabled by default
    public: bool enabledByDefault = true;
  };
}

using namespace gazebo;

//////////////////////////////////////////////////
void MisalignmentPluginPrivate::Enable(ConstIntPtr &_msg)
{
  auto enable = _msg->data() != 0;

  // Already started
  if (enable && this->updateConnection)
  {
    gzwarn << "Plugin is already enabled." << std::endl;
    return;
  }

  // Already stopped
  if (!enable && !this->updateConnection)
  {
    gzwarn << "Plugin is already disabled." << std::endl;
    return;
  }

  if (enable)
  {
    // Start
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(
        &MisalignmentPluginPrivate::OnUpdate, this, std::placeholders::_1));
    gzmsg << "Started plugin [" << this->ns << "]\n";
  }
  else
  {
    // Stop
    this->updateConnection.reset();
    gzmsg << "Stopped plugin [" << this->ns << "]\n";
  }
}

/////////////////////////////////////////////////
void MisalignmentPluginPrivate::OnUpdate(const common::UpdateInfo &_info)
{
  // These poses may or may not be in world frame
  ignition::math::Pose3d refPose = this->referencePose;
  ignition::math::Pose3d tgtPose = this->targetPose;

  // If target pose is not in world frame, get the entity it is relative to
  if (!this->targetFrameName.empty())
  {
    physics::EntityPtr entity = this->targetEntity.lock();
    if (!entity)
    {
      this->targetEntity = this->world->EntityByName(this->targetFrameName);
      entity = this->targetEntity.lock();
      if (!entity)
      {
        // Could not find entity to be compaired with reference
        if (!this->didWarnTgt)
        {
          gzwarn << "Did not find target " << this->targetFrameName << "\n";
          this->didWarnTgt = true;
        }
        return;
      }
      this->didWarnTgt = false;
    }
    // tgtPos is relative to entity, transform tgtPose into world frame
    ignition::math::Pose3d entityFromWorld = entity->WorldPose();
    auto worldFromEntity = entityFromWorld.Inverse();
    tgtPose.Pos() =
      (entityFromWorld.Rot() * tgtPose.Pos()) + entityFromWorld.Pos();
    tgtPose.Rot() = (entityFromWorld.Rot() * tgtPose.Rot());
    tgtPose.Rot().Normalize();
  }

  // If reference pose is not in world frame, get the entity it is relative to
  if (!this->referenceFrameName.empty())
  {
    physics::EntityPtr refEntity = this->referenceEntity.lock();
    if (!refEntity)
    {
      this->referenceEntity = this->world->EntityByName(
        this->referenceFrameName);
      refEntity = this->referenceEntity.lock();
      if (!refEntity)
      {
        // Could not find reference entity
        if (!this->didWarnRef)
        {
          gzwarn << "Did not find reference "
            << this->referenceFrameName << "\n";
          this->didWarnRef = true;
        }
        return;
      }
      this->didWarnRef = false;
    }
    // refPos is relative to reference entity, transform into world frame
    ignition::math::Pose3d refFromWorld = refEntity->WorldPose();
    refPose.Pos() = (refFromWorld.Rot() * refPose.Pos()) + refFromWorld.Pos();
    refPose.Rot() = (refFromWorld.Rot() * refPose.Rot());
    refPose.Rot().Normalize();
  }

  // Calculate misaligment by transforming target pose to frame defined by
  // reference pose
  ignition::math::Pose3d worldFromReference = refPose.Inverse();
  ignition::math::Vector3d linearMisalignment =
    (worldFromReference.Rot() * tgtPose.Pos()) + worldFromReference.Pos();
  ignition::math::Quaterniond angularMisalignment =
    (worldFromReference.Rot() * tgtPose.Rot());
  angularMisalignment.Normalize();
  ignition::math::Pose3d misalignment(linearMisalignment, angularMisalignment);

  // Debug printing/publishing
  if (this->debug)
  {
    // Meters
    double x = misalignment.Pos().X();
    double y = misalignment.Pos().Y();
    double z = misalignment.Pos().Z();

    // Radians
    ignition::math::Vector3d eulerMisalignment = misalignment.Rot().Euler();
    double roll = eulerMisalignment.X();
    double pitch = eulerMisalignment.Y();
    double yaw = eulerMisalignment.Z();

    gzdbg << "Misalignment xyz(" << x << ", " << y << ", " << z << ")"
      << " rpy(" << roll << ", " << pitch << ", " << yaw << ")\n";

    msgs::PoseStamped refPoseInWorld;
    msgs::PoseStamped tgtPoseInWorld;
    msgs::Set(refPoseInWorld.mutable_time(), _info.simTime);
    msgs::Set(tgtPoseInWorld.mutable_time(), _info.simTime);
    msgs::Set(refPoseInWorld.mutable_pose(), refPose);
    msgs::Set(tgtPoseInWorld.mutable_pose(), tgtPose);
    this->pubDbgRefPose->Publish(refPoseInWorld);
    this->pubDbgTgtPose->Publish(tgtPoseInWorld);
  }

  // Publish misalignment
  msgs::PoseStamped msg;
  msgs::Set(msg.mutable_time(), _info.simTime);
  msgs::Set(msg.mutable_pose(), misalignment);
  this->pubMisalignment->Publish(msg);
}


GZ_REGISTER_WORLD_PLUGIN(MisalignmentPlugin)

/////////////////////////////////////////////////
MisalignmentPlugin::MisalignmentPlugin() : WorldPlugin(),
    dataPtr(new MisalignmentPluginPrivate)
{
}

/////////////////////////////////////////////////
void MisalignmentPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
  gzmsg << "Loading MisalignmentPlugin\n";
  // Two poses
  if (!_sdf->HasElement("pose"))
  {
    gzerr << "Missing required tag <pose>, plugin will not be initialized\n";
    return;
  }
  sdf::ElementPtr refPose = _sdf->GetElement("pose");
  sdf::ElementPtr tgtPose = refPose->GetNextElement("pose");
  if (!tgtPose)
  {
    gzerr << "Two <pose> tags are required, plugin will not be initialized.\n";
    return;
  }

  if (refPose->GetNextElement("pose"))
  {
    gzwarn << "Found more than two <pose> tags,"
      << " only the first two will be used.\n";
  }

  this->dataPtr->referencePose = refPose->Get<ignition::math::Pose3d>();
  this->dataPtr->targetPose = tgtPose->Get<ignition::math::Pose3d>();

  // Frame of reference for two poses
  sdf::ParamPtr frameParam = refPose->GetAttribute("frame");
  if (frameParam)
  {
    this->dataPtr->referenceFrameName = frameParam->GetAsString();
  }
  frameParam = tgtPose->GetAttribute("frame");
  if (frameParam)
  {
    this->dataPtr->targetFrameName = frameParam->GetAsString();
  }

  // Namespace
  if (!_sdf->HasElement("namespace"))
  {
    gzerr << "Missing required parameter <namespace>, plugin will not be "
          << "initialized." << std::endl;
    return;
  }
  this->dataPtr->ns = _sdf->Get<std::string>("namespace");

  if (_sdf->HasElement("debug"))
  {
    this->dataPtr->debug = _sdf->Get<bool>("debug");
  }

  if (_sdf->HasElement("enabled"))
    this->dataPtr->enabledByDefault = _sdf->Get<bool>("enabled");

  // Transport initialization
  this->dataPtr->gzNode = transport::NodePtr(new transport::Node());
  this->dataPtr->gzNode->Init();

  this->dataPtr->enableGzSub = this->dataPtr->gzNode->Subscribe("/" +
      this->dataPtr->ns + "/enable", &MisalignmentPluginPrivate::Enable,
      &*(this->dataPtr));

  this->dataPtr->pubMisalignment =
    this->dataPtr->gzNode->Advertise<msgs::PoseStamped>(
      "/" + this->dataPtr->ns + "/misalignment");

  if (this->dataPtr->debug)
  {
    this->dataPtr->pubDbgRefPose =
      this->dataPtr->gzNode->Advertise<msgs::PoseStamped>(
        "/" + this->dataPtr->ns + "/debug/first_pose");
    this->dataPtr->pubDbgTgtPose =
      this->dataPtr->gzNode->Advertise<msgs::PoseStamped>(
        "/" + this->dataPtr->ns + "/debug/second_pose");
  }

  // Save the world
  this->dataPtr->world = _world;

  // Enable the plugin
  if (this->dataPtr->enabledByDefault)
  {
    msgs::IntPtr msg(new msgs::Int());
    msg->set_data(1);
    this->dataPtr->Enable(msg);
  }
}
