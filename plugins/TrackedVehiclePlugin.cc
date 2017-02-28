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

  public: template <typename T> int sgn(T val)
  {
    return (T(0) < val) - (val < T(0));
  }
};

/////////////////////////////////////////////////
TrackedVehiclePlugin::TrackedVehiclePlugin()
  : dataPtr(new TrackedVehiclePluginPrivate)
{
}

/////////////////////////////////////////////////
TrackedVehiclePlugin::~TrackedVehiclePlugin()
{
}

/////////////////////////////////////////////////
void TrackedVehiclePlugin::Load(physics::ModelPtr _model,
                     sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model, "TrackedVehiclePlugin _model pointer is NULL");
  this->dataPtr->model = _model;

  GZ_ASSERT(_sdf, "PlaneDemoPlugin _sdf pointer is NULL");
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

  // Initialize transport.
  this->dataPtr->node = transport::NodePtr(new transport::Node());
  this->dataPtr->node->Init();

  this->dataPtr->velocitySub = this->dataPtr->node->Subscribe(
    this->dataPtr->robotNamespace + "/cmd_vel",
    &TrackedVehiclePlugin::OnVelMsg, this);

  this->dataPtr->tracksVelocityPub =
      this->dataPtr->node->Advertise<msgs::Vector2d>(
          this->dataPtr->robotNamespace + "/tracks_speed", 1000);

  gzdbg << this->handleName.c_str() << ": Loading done." << std::endl;
}

/////////////////////////////////////////////////
void TrackedVehiclePlugin::Init()
{
  gzdbg << this->handleName.c_str() << ": Init done.\n";
}

void TrackedVehiclePlugin::OnVelMsg(ConstPosePtr &msg)
{
  std::lock_guard<std::mutex> lock(this->mutex);

  const double yaw = msgs::ConvertIgn(msg->orientation()).Euler().Z();

  const double linearSpeed = msg->position().x() * this->dataPtr->linearSpeedGain;
  const double angularSpeed = yaw * this->dataPtr->angularSpeedGain;

  double vel_left = linearSpeed + angularSpeed *
    this->dataPtr->tracksSeparation / 2 / this->dataPtr->steeringEfficiency;

  double vel_right = linearSpeed - angularSpeed *
    this->dataPtr->tracksSeparation / 2 / this->dataPtr->steeringEfficiency;

  const double maxLinearSpeed = fabs(this->dataPtr->linearSpeedGain);
  if (fabs(vel_left) > maxLinearSpeed) {
    vel_left = this->dataPtr->sgn(vel_left) * maxLinearSpeed;
  }
  if (fabs(vel_right) > maxLinearSpeed) {
    vel_right = this->dataPtr->sgn(vel_right) * maxLinearSpeed;
  }

  // call the descendant handler
  this->SetTrackVelocity(vel_left, vel_right);

  auto speedMsg = msgs::Vector2d();
  speedMsg.set_x(vel_left);
  speedMsg.set_y(vel_right);
  this->dataPtr->tracksVelocityPub->Publish(speedMsg);
}

std::string TrackedVehiclePlugin::GetRobotNamespace() {
  return this->dataPtr->robotNamespace;
}

double TrackedVehiclePlugin::GetSteeringEfficiency() {
  return this->dataPtr->steeringEfficiency;
}

void TrackedVehiclePlugin::SetSteeringEfficiency(double steeringEfficiency) {
  this->dataPtr->steeringEfficiency = steeringEfficiency;
  this->dataPtr->sdf->GetElement("steering_efficiency")->Set(steeringEfficiency);
}

double TrackedVehiclePlugin::GetTracksSeparation() {
  return this->dataPtr->tracksSeparation;
}

double TrackedVehiclePlugin::GetTrackMu() {
  return this->dataPtr->trackMu;
}

void TrackedVehiclePlugin::SetTrackMu(double mu) {
  this->dataPtr->trackMu = mu;
  this->dataPtr->sdf->GetElement("track_mu")->Set(mu);
}

double TrackedVehiclePlugin::GetTrackMu2() {
  return this->dataPtr->trackMu2;
}

void TrackedVehiclePlugin::SetTrackMu2(double mu2) {
  this->dataPtr->trackMu2 = mu2;
  this->dataPtr->sdf->GetElement("track_mu2")->Set(mu2);
}