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

#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>

#include "gazebo/common/Assert.hh"
#include "gazebo/transport/transport.hh"
#include "plugins/TrackedVehiclePlugin.hh"

using namespace gazebo;

class TrackedVehicleKeyboardControls
{
  /// \brief Key for zeroing-out the velocity (default is Space, Enter).
  public: std::vector<unsigned int> stop = { 13, 32 };

  /// \brief Accelerate key (default is up arrow).
  public: std::vector<unsigned int> accelerate = { 38, 16777235 };

  /// \brief Decelerate key (default is down arrow).
  public: std::vector<unsigned int> decelerate = { 40, 16777237 };

  /// \brief Left key (default is left arrow).
  public: std::vector<unsigned int> left = { 37, 16777234 };

  /// \brief Right key (default is right arrow).
  public: std::vector<unsigned int> right = { 39, 16777236 };
};

/// \brief Private data class
class gazebo::TrackedVehiclePluginPrivate
{
  /// \brief Pointer to model containing plugin.
  public: physics::ModelPtr model;

  /// \brief SDF for this plugin;
  public: sdf::ElementPtr sdf;

  /// \brief Pointer to a node with world prefix.
  public: transport::NodePtr worldNode;

  /// \brief Pointer to a node with robot prefix.
  public: transport::NodePtr robotNode;

  /// \brief Velocity command subscriber.
  public: transport::SubscriberPtr velocitySub;

  /// \brief Subscribe to keyboard messages.
  public: transport::SubscriberPtr keyboardSub;

  /// \brief Publisher of the track velocities.
  public: transport::PublisherPtr tracksVelocityPub;

  /// \brief Distance between the centers of the tracks.
  public: double tracksSeparation = 0.1;

  /// \brief Steering efficiency coefficient (between 0.0 and 1.0).
  public: double steeringEfficiency = 0.5;

  /// \brief Coefficient that converts velocity command units to track velocity.
  public: double linearSpeedGain = 1.0;

  /// \brief Coefficient that converts velocity command units to track velocity.
  public: double angularSpeedGain = 1.0;

  /// \brief Friction coefficient in the first friction direction.
  public: double trackMu = 1.0;

  /// \brief Friction coefficient in the second friction direction.
  public: double trackMu2 = 1.0;

  /// \brief Namespace used as a prefix for gazebo topic names.
  public: std::string robotNamespace;

  /// \brief Whether keyboard control should be used for this model.
  public: bool enableKeyboardControl = false;

  /// \brief Definition of the keyboard controls.
  public: TrackedVehicleKeyboardControls keyboardControls;

  /// \brief The message to be sent that is updated by keypresses.
  public: msgs::PosePtr keyboardControlMessage;
};

TrackedVehiclePlugin::TrackedVehiclePlugin()
  : dataPtr(new TrackedVehiclePluginPrivate)
{
  this->trackNames[Tracks::LEFT] = "left";
  this->trackNames[Tracks::RIGHT] = "right";
}

TrackedVehiclePlugin::~TrackedVehiclePlugin() = default;

void TrackedVehiclePlugin::Load(physics::ModelPtr _model,
                     sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model, "TrackedVehiclePlugin _model pointer is NULL");
  this->dataPtr->model = _model;

  GZ_ASSERT(_sdf, "TrackedVehiclePlugin _sdf pointer is NULL");
  this->dataPtr->sdf = _sdf;

  // Load parameters from SDF plugin contents.
  this->LoadParam(_sdf, "robot_namespace", this->dataPtr->robotNamespace,
                  _model->GetName());
  this->LoadParam(_sdf, "steering_efficiency",
                  this->dataPtr->steeringEfficiency, 0.5);
  this->LoadParam(_sdf, "tracks_separation",
                  this->dataPtr->tracksSeparation, 0.4);
  this->LoadParam(_sdf, "linear_speed_gain",
                  this->dataPtr->linearSpeedGain, 1.);
  this->LoadParam(_sdf, "angular_speed_gain",
                  this->dataPtr->angularSpeedGain, 1.);
  this->LoadParam(_sdf, "track_mu", this->dataPtr->trackMu, 2.0);
  this->LoadParam(_sdf, "track_mu2", this->dataPtr->trackMu2, 0.5);

  this->dataPtr->enableKeyboardControl = _sdf->HasElement("key_controls");

  // Load keyboard controls.
  if (this->dataPtr->enableKeyboardControl)
  {
    auto const keyControlsElem = _sdf->GetElement("key_controls");

    auto keyControlsEmpty = keyControlsElem->GetFirstElement() ==
      sdf::ElementPtr();

    gzmsg << this->handleName << " Plugin keyboard control enabled with "
          << (keyControlsEmpty ? "default" : "custom") << " key assignments"
          << std::endl;

    // if the <key_controls> tag is empty (but present) keyboard control should
    // be enabled with the default key assignments
    if (!keyControlsEmpty)
    {
      // Mapping from XML keys to TrackedVehicleKeyboardControls members.
      const std::map<const std::string, std::vector<unsigned int> &>
        controlsMapping =
        {
          {"stop", this->dataPtr->keyboardControls.stop},
          {"accelerate", this->dataPtr->keyboardControls.accelerate},
          {"decelerate", this->dataPtr->keyboardControls.decelerate},
          {"left", this->dataPtr->keyboardControls.left},
          {"right", this->dataPtr->keyboardControls.right},
        };

      for (auto controlsPair : controlsMapping)
      {
        const std::string &controlKeyName = controlsPair.first;
        std::vector<unsigned int> &controlKeyList = controlsPair.second;

        controlKeyList.clear();
        if (keyControlsElem->HasElement(controlKeyName))
        {
          auto controlElem = keyControlsElem->GetElement(controlKeyName);
          while (controlElem != nullptr)
          {
            controlKeyList.push_back(controlElem->Get<unsigned int>());
            controlElem = controlElem->GetNextElement(controlKeyName);
          }
        }
        else
        {
          gzwarn << "Key " << controlKeyName << " has no assigned keycode." <<
                 std::endl;
        }
      }
    }
  }
}

void TrackedVehiclePlugin::Init()
{
  // Initialize transport nodes.

  this->dataPtr->worldNode = transport::NodePtr(new transport::Node());
  this->dataPtr->worldNode->Init();

  // Prepend world name to robot namespace if it isn't absolute.
  auto robotNamespace = this->GetRobotNamespace();
  if (!robotNamespace.empty() && robotNamespace.at(0) != '/')
  {
    robotNamespace = this->dataPtr->model->GetWorld()->Name() +
      "/" + robotNamespace;
  }
  this->dataPtr->robotNode = transport::NodePtr(new transport::Node());
  this->dataPtr->robotNode->Init(robotNamespace);

  this->dataPtr->velocitySub = this->dataPtr->robotNode->Subscribe("~/cmd_vel",
    &TrackedVehiclePlugin::OnVelMsg, this);

  this->dataPtr->tracksVelocityPub =
    this->dataPtr->robotNode->Advertise<msgs::Vector2d>("~/tracks_speed", 1000);

  if (this->dataPtr->enableKeyboardControl)
  {
    this->dataPtr->keyboardControlMessage = msgs::PosePtr(new msgs::Pose());

    // Keypresses are published on the world namespace, so we need to subscribe
    // them through the worldNode.
    this->dataPtr->keyboardSub = this->dataPtr->worldNode->Subscribe(
      "~/keyboard/keypress", &TrackedVehiclePlugin::OnKeyPress, this, true);
  }
}

void TrackedVehiclePlugin::Reset()
{
  // reset the keyboard update message
  if (this->dataPtr->enableKeyboardControl)
  {
    auto message = this->dataPtr->keyboardControlMessage;
    message->mutable_position()->set_x(0.0);
    msgs::Set(message->mutable_orientation(),
              ignition::math::Quaterniond::Identity);
  }

  this->SetTrackVelocity(0., 0.);

  ModelPlugin::Reset();
}

void TrackedVehiclePlugin::SetTrackVelocity(double _left, double _right)
{
  // Call the descendant custom handler of the subclass.
  this->SetTrackVelocityImpl(_left, _right);

  // Publish the resulting track velocities to anyone who is interested.
  auto speedMsg = msgs::Vector2d();
  speedMsg.set_x(_left);
  speedMsg.set_y(_right);
  this->dataPtr->tracksVelocityPub->Publish(speedMsg);
}

void TrackedVehiclePlugin::OnVelMsg(ConstPosePtr &_msg)
{
  std::lock_guard<std::mutex> lock(this->mutex);

  const double yaw = msgs::ConvertIgn(_msg->orientation()).Euler().Z();

  // Compute effective linear and angular speed.
  const double linearSpeed =
    _msg->position().x() * this->dataPtr->linearSpeedGain;
  const double angularSpeed = yaw * this->dataPtr->angularSpeedGain;

  // Compute track velocities using the tracked vehicle kinematics model.
  double leftVelocity = linearSpeed + angularSpeed *
    this->dataPtr->tracksSeparation / 2 / this->dataPtr->steeringEfficiency;

  double rightVelocity = linearSpeed - angularSpeed *
    this->dataPtr->tracksSeparation / 2 / this->dataPtr->steeringEfficiency;

  // Apply max speed to each of the tracks.
  const double maxLinearSpeed = fabs(this->dataPtr->linearSpeedGain);
  if (fabs(leftVelocity) > maxLinearSpeed)
  {
    leftVelocity = ignition::math::signum(leftVelocity) * maxLinearSpeed;
  }
  if (fabs(rightVelocity) > maxLinearSpeed)
  {
    rightVelocity = ignition::math::signum(rightVelocity) * maxLinearSpeed;
  }

  // Call the track velocity handler (which does the actual vehicle control).
  this->SetTrackVelocity(leftVelocity, rightVelocity);
}

void TrackedVehiclePlugin::OnKeyPress(ConstAnyPtr &_msg)
{
  const auto key = static_cast<const unsigned int>(_msg->int_value());

  double linearVel = 0., angularVel = 0.;
  bool linearVelSet = false, angularVelSet = false;

  const auto &controls = this->dataPtr->keyboardControls;

  auto &message = this->dataPtr->keyboardControlMessage;

  if (std::find(controls.stop.begin(), controls.stop.end(), key) !=
      controls.stop.end())
  {
    linearVel = 0.;
    linearVelSet = true;

    angularVel = 0.;
    angularVelSet = true;
  }
  else
  {
    if (std::find(
      controls.accelerate.begin(), controls.accelerate.end(), key) !=
      controls.accelerate.end())
    {
      linearVel = 1.;
      linearVelSet = true;
    }
    else if (std::find(
      controls.decelerate.begin(), controls.decelerate.end(), key) !=
      controls.decelerate.end())
    {
      linearVel = -1.;
      linearVelSet = true;
    }

    if (linearVelSet)
    {
      const auto oldLinearVel = message->position().x();

      // transition from forward speed to backward should have more
      // than 2 states
      if (!ignition::math::equal(linearVel, oldLinearVel))
      {
        linearVel = oldLinearVel + ignition::math::signum(linearVel) * 0.5;
      }
    }

    if (std::find(controls.left.begin(), controls.left.end(), key) !=
      controls.left.end())
    {
      angularVel = -1.;
      angularVelSet = true;
    }
    else if (std::find(controls.right.begin(), controls.right.end(), key) !=
      controls.right.end())
    {
      angularVel = 1.;
      angularVelSet = true;
    }

    if (angularVelSet)
    {
      const auto oldAngularVel =
        msgs::ConvertIgn(message->orientation()).Euler().Z();

      // transition from left to right should have more than 2 states
      if (!ignition::math::equal(angularVel, oldAngularVel))
      {
        angularVel = oldAngularVel + ignition::math::signum(angularVel) * 0.5;
      }
    }
  }

  if (linearVelSet)
  {
    message->mutable_position()->set_x(linearVel);
  }

  if (angularVelSet)
  {
    auto yaw = ignition::math::Quaterniond::EulerToQuaternion(
      0, 0, angularVel);
    msgs::Set(message->mutable_orientation(), yaw);
  }

  // start the keyboard update timer if this was the first time a control key
  // was pressed
  if (linearVelSet || angularVelSet)
  {
    this->OnVelMsg(message);
  }
}

std::string TrackedVehiclePlugin::GetRobotNamespace()
{
  return this->dataPtr->robotNamespace;
}

double TrackedVehiclePlugin::GetSteeringEfficiency()
{
  return this->dataPtr->steeringEfficiency;
}

void TrackedVehiclePlugin::SetSteeringEfficiency(double _steeringEfficiency)
{
  this->dataPtr->steeringEfficiency = _steeringEfficiency;
  this->dataPtr->sdf->GetElement("steering_efficiency")
    ->Set(_steeringEfficiency);
}

double TrackedVehiclePlugin::GetTracksSeparation()
{
  return this->dataPtr->tracksSeparation;
}

double TrackedVehiclePlugin::GetTrackMu()
{
  return this->dataPtr->trackMu;
}

void TrackedVehiclePlugin::SetTrackMu(double _mu)
{
  this->dataPtr->trackMu = _mu;
  this->dataPtr->sdf->GetElement("track_mu")->Set(_mu);
  this->UpdateTrackSurface();
}

double TrackedVehiclePlugin::GetTrackMu2()
{
  return this->dataPtr->trackMu2;
}

void TrackedVehiclePlugin::SetTrackMu2(double _mu2)
{
  this->dataPtr->trackMu2 = _mu2;
  this->dataPtr->sdf->GetElement("track_mu2")->Set(_mu2);
  this->UpdateTrackSurface();
}

void TrackedVehiclePlugin::SetLinkMu(const physics::LinkPtr &_link)
{
  for (auto const &collision : _link->GetCollisions())
    {
      auto frictionPyramid = collision->GetSurface()->FrictionPyramid();
      if (frictionPyramid == nullptr)
      {
        gzwarn << "This dynamics engine doesn't support setting mu/mu2 friction"
          " parameters. Use its dedicated friction setting mechanism to set the"
          " wheel friction." << std::endl;
        break;
      }

      double mu = this->GetTrackMu();
      double mu2 = this->GetTrackMu2();

      if (!ignition::math::equal(frictionPyramid->MuPrimary(), mu, 1e-6))
      {
        gzdbg << "Setting mu (friction) of link '" << _link->GetName() <<
              "' from " << frictionPyramid->MuPrimary() << " to " <<
              mu << std::endl;
      }

      if (!ignition::math::equal(frictionPyramid->MuSecondary(), mu2, 1e-6))
      {
        gzdbg << "Setting mu2 (friction) of link '" << _link->GetName() <<
              "' from " << frictionPyramid->MuSecondary() << " to " <<
              mu2 << std::endl;
      }

      frictionPyramid->SetMuPrimary(mu);
      frictionPyramid->SetMuSecondary(mu2);
    }
}
