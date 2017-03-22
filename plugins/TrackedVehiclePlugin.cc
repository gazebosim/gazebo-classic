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

/// \brief Private data class
class gazebo::TrackedVehiclePluginPrivate
{
  /// \brief Pointer to model containing plugin.
  public: physics::ModelPtr model;

  /// \brief SDF for this plugin;
  public: sdf::ElementPtr sdf;

  /// \brief Pointer to a node for communication.
  public: transport::NodePtr node;

  /// \brief Velocity command subscriber.
  public: transport::SubscriberPtr velocitySub;

  /// \brief Subscribe to keyboard messages.
  public: transport::SubscriberPtr keyboardSub;

  /// \brief Publisher of the track velocities.
  public: transport::PublisherPtr tracksVelocityPub;

  /// \brief Distance between the centers of the tracks.
  public: double tracksSeparation;

  /// \brief Steering efficiency coefficient (between 0.0 and 1.0).
  public: double steeringEfficiency;

  /// \brief Coefficient that converts velocity command units to track velocity.
  public: double linearSpeedGain;

  /// \brief Coefficient that converts velocity command units to track velocity.
  public: double angularSpeedGain;

  /// \brief Friction coefficient in the first friction direction.
  public: double trackMu;

  /// \brief Friction coefficient in the second friction direction.
  public: double trackMu2;

  /// \brief Namespace used as a prefix for gazebo topic names.
  public: std::string robotNamespace;
};

TrackedVehiclePlugin::TrackedVehiclePlugin()
  : dataPtr(new TrackedVehiclePluginPrivate)
{
  this->trackNames[Tracks::LEFT] = "left";
  this->trackNames[Tracks::RIGHT] = "right";
}

TrackedVehiclePlugin::~TrackedVehiclePlugin()
{
}

void TrackedVehiclePlugin::Load(physics::ModelPtr _model,
                     sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model, "TrackedVehiclePlugin _model pointer is NULL");
  this->dataPtr->model = _model;

  GZ_ASSERT(_sdf, "TrackedVehiclePlugin _sdf pointer is NULL");
  this->dataPtr->sdf = _sdf;

  // Load parameters from SDF plugin contents.
  this->LoadParam(_sdf, "robot_namespace", this->dataPtr->robotNamespace, "~");
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

  gzdbg << this->handleName.c_str() << ": Loading done." << std::endl;
}

void TrackedVehiclePlugin::Init()
{
  // Initialize transport.
  this->dataPtr->node = transport::NodePtr(new transport::Node());
  this->dataPtr->node->Init(this->GetRobotNamespace());

  this->dataPtr->velocitySub = this->dataPtr->node->Subscribe(
    this->dataPtr->robotNamespace + "/cmd_vel",
    &TrackedVehiclePlugin::OnVelMsg, this);

  this->dataPtr->tracksVelocityPub =
    this->dataPtr->node->Advertise<msgs::Vector2d>(
      this->dataPtr->robotNamespace + "/tracks_speed", 1000);

  this->dataPtr->keyboardSub = this->dataPtr->node->Subscribe(
    "/gazebo/default/keyboard/keypress", &TrackedVehiclePlugin::OnKeyPress,
    this, true);

  gzdbg << this->handleName.c_str() << ": Init done.\n";
}

void TrackedVehiclePlugin::Reset()
{
  if (this->dataPtr->tracksVelocityPub)
  {
    auto speedMsg = msgs::Vector2d();
    speedMsg.set_x(0);
    speedMsg.set_y(0);
    this->dataPtr->tracksVelocityPub->Publish(speedMsg);
  }

  ModelPlugin::Reset();
}

void TrackedVehiclePlugin::OnVelMsg(ConstPosePtr &_msg)
{
  std::lock_guard<std::mutex> lock(this->mutex);

  const double yaw = msgs::ConvertIgn(_msg->orientation()).Euler().Z();

  const double linearSpeed =
    _msg->position().x() * this->dataPtr->linearSpeedGain;
  const double angularSpeed = yaw * this->dataPtr->angularSpeedGain;

  double leftVelocity = linearSpeed + angularSpeed *
    this->dataPtr->tracksSeparation / 2 / this->dataPtr->steeringEfficiency;

  double rightVelocity = linearSpeed - angularSpeed *
    this->dataPtr->tracksSeparation / 2 / this->dataPtr->steeringEfficiency;

  const double maxLinearSpeed = fabs(this->dataPtr->linearSpeedGain);
  if (fabs(leftVelocity) > maxLinearSpeed)
  {
    leftVelocity = ignition::math::signum(leftVelocity) * maxLinearSpeed;
  }
  if (fabs(rightVelocity) > maxLinearSpeed)
  {
    rightVelocity = ignition::math::signum(rightVelocity) * maxLinearSpeed;
  }

  // call the descendant handler
  this->SetTrackVelocity(leftVelocity, rightVelocity);

  auto speedMsg = msgs::Vector2d();
  speedMsg.set_x(leftVelocity);
  speedMsg.set_y(rightVelocity);
  this->dataPtr->tracksVelocityPub->Publish(speedMsg);
}

void TrackedVehiclePlugin::OnKeyPress(ConstAnyPtr &_msg)
{
  const int key = _msg->int_value();

  double linearVel = 0., angularVel = 0.;

  switch (key)
  {
    // ENTER
    case 13:
    // SPACE
    case 32:
      linearVel = 0.;
      angularVel = 0.;
      break;
    // UP
    case 38:
    case 16777235:
      linearVel = 1.;
      break;
    // DOWN
    case 40:
    case 16777237:
      linearVel = -1.;
      break;
    // LEFT
    case 37:
    case 16777234:
      angularVel = -1.;
      break;
    // RIGHT
    case 39:
    case 16777236:
      angularVel = 1.;
      break;
    default:
      return;
  }

  {
    msgs::PosePtr message(new msgs::Pose());
    message->mutable_position()->set_x(linearVel);
    auto yaw = ignition::math::Quaterniond::EulerToQuaternion(0, 0, angularVel);
    msgs::Set(message->mutable_orientation(), yaw);
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
  for (auto collision : _link->GetCollisions())
    {
      auto frictionPyramid = collision->GetSurface()->FrictionPyramid();
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
