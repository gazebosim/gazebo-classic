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
#ifndef GAZEBO_PLUGINS_TRACKEDVEHICLEPLUGIN_HH_
#define GAZEBO_PLUGINS_TRACKEDVEHICLEPLUGIN_HH_

#include <string>

#include <boost/algorithm/string.hpp>

#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/TransportTypes.hh"

namespace gazebo
{
  class TrackedVehiclePluginPrivate;

  /// \class TrackedVehiclePlugin TrackedVehiclePlugin.hh
  /// \brief An abstract gazebo model plugin for tracked vehicles.
  /// \since 8.1
  ///
  /// The plugin processes the following parameters (all have defaults):
  /// <steering_efficiency>  Steering efficiency coefficient (0.0 to 1.0).
  /// <tracks_separation>  Distance between the centers of the tracks.
  /// <linear_speed_gain>  Coefficient that converts velocity command units to
  ///                      track velocity.
  /// <angular_speed_gain>  Coefficient that converts velocity command units to
  ///                       track velocity.
  /// <track_mu>  Friction coefficient in the first friction direction.
  /// <track_mu2>  Friction coefficient in the second friction direction.
  /// <robot_namespace>  Namespace used as a prefix for gazebo topic names.
  class GAZEBO_VISIBLE TrackedVehiclePlugin : public ModelPlugin
  {
    /// \brief Default Contstuctor
    public: TrackedVehiclePlugin();

    /// \brief Destructor
    public: virtual ~TrackedVehiclePlugin();

    /// \brief Called when the plugin is loaded
    /// \param[in] _model Pointer to the model for which the plugin is loaded
    /// \param[in] _sdf Pointer to the SDF for _model
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Initialize the plugin.
    public: virtual void Init();

    /// \brief Reset the plugin.
    public: virtual void Reset();

    /// \brief Namespace used as a prefix for gazebo topic names.
    public: virtual std::string GetRobotNamespace();

    /// \brief Steering efficiency coefficient (between 0.0 and 1.0).
    public: virtual double GetSteeringEfficiency();

    /// \brief Set steering efficiency coefficient (between 0.0 and 1.0).
    /// \param[in] _steeringEfficiency The new steering efficiency.
    public: virtual void SetSteeringEfficiency(double _steeringEfficiency);

    /// \brief Friction coefficient in the first friction direction.
    public: virtual double GetTrackMu();

    /// \brief Friction coefficient in the first friction direction.
    /// \param[in] _mu The new coefficient.
    public: virtual void SetTrackMu(double _mu);

    /// \brief Friction coefficient in the second friction direction.
    public: virtual double GetTrackMu2();

    /// \brief Friction coefficient in the second friction direction.
    /// \param[in] _mu2 The new coefficient.
    public: virtual void SetTrackMu2(double _mu2);

    /// \brief Distance between the centers of the tracks.
    public: virtual double GetTracksSeparation();

    /// \brief Set new target velocity for the tracks.
    ///
    /// Descendant classes need to implement this function.
    ///
    /// \param[in] _left Velocity of left track.
    /// \param[in] _right Velocity of right track.
    protected: virtual void SetTrackVelocity(double _left, double _right) = 0;

    /// \brief Callback for setting desired body velocity.
    ///
    /// Normally, this callback converts the x/y/yaw message to track velocities
    /// and calls SetTrackVelocity().
    ///
    /// \param[in] _msg Pose message from external publisher
    protected: virtual void OnVelMsg(ConstPosePtr &_msg);

    /// \brief Callback each time a key message is received.
    /// \param[in] _msg Keypress message.
    protected: virtual void OnKeyPress(ConstAnyPtr &_msg);

    /// \brief Mutex to protect updates
    protected: std::mutex mutex;

    /// \brief Private data pointer
    private: std::unique_ptr<TrackedVehiclePluginPrivate> dataPtr;
  };
}
#endif
