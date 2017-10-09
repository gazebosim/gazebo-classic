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
#include <unordered_map>

#include <boost/algorithm/string.hpp>
#include <gazebo/physics/ode/ODELink.hh>
#include <gazebo/physics/ode/ODECollision.hh>

#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "plugins/TrackedVehiclePlugin.hh"

namespace gazebo
{
  /// \enum Tracks
  /// \brief Enum for distinguishing between left and right tracks.
  enum class Tracks : bool { LEFT, RIGHT };
}

namespace std
{
  template <> struct hash<gazebo::Tracks>
  {
    size_t operator() (const gazebo::Tracks &_t) const
    {
      return size_t(_t);
    }
  };
}

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
  /// <key_controls>  If present, enable keyboard control of the vehicle. If
  ///                 this tag is empty, the default assignment (arrow keys) is
  ///                 is used; otherwise, the keys can be set using the
  ///                 (repeatable) subelements <stop>, <accelerate>,
  ///                 <decelerate>, <left> and <right> containing the keycodes.
  class TrackedVehiclePlugin : public ModelPlugin
  {
    /// \brief Default Contstuctor
    public: TrackedVehiclePlugin();

    /// \brief Destructor
    public: virtual ~TrackedVehiclePlugin();

    /// \brief Called when the plugin is loaded
    /// \param[in] _model Pointer to the model for which the plugin is loaded
    /// \param[in] _sdf Pointer to the SDF for _model
    public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

    /// \brief Initialize the plugin.
    public: void Init() override;

    /// \brief Reset the plugin.
    public: void Reset() override;

    /// \brief Namespace used as a prefix for gazebo topic names.
    protected: virtual std::string GetRobotNamespace();

    /// \brief Steering efficiency coefficient (between 0.0 and 1.0).
    protected: virtual double GetSteeringEfficiency();

    /// \brief Set steering efficiency coefficient (between 0.0 and 1.0).
    /// \param[in] _steeringEfficiency The new steering efficiency.
    protected: virtual void SetSteeringEfficiency(double _steeringEfficiency);

    /// \brief Friction coefficient in the first friction direction.
    protected: virtual double GetTrackMu();

    /// \brief Friction coefficient in the first friction direction.
    /// \param[in] _mu The new coefficient.
    protected: virtual void SetTrackMu(double _mu);

    /// \brief Friction coefficient in the second friction direction.
    protected: virtual double GetTrackMu2();

    /// \brief Friction coefficient in the second friction direction.
    /// \param[in] _mu2 The new coefficient.
    protected: virtual void SetTrackMu2(double _mu2);

    /// \brief Update surface parameters of the tracks to correspond to the
    ///        values set in this plugin.
    protected: virtual void UpdateTrackSurface() = 0;

    /// \brief Set mu and mu2 of all collisions of the given link to values
    ///        given by GetTrackMu() and GetTrackMu2().
    ///
    /// \param[in] _link The link whose "mu"s are to be set.
    protected: void SetLinkMu(const physics::LinkPtr &_link);

    /// \brief Distance between the centers of the tracks.
    protected: virtual double GetTracksSeparation();

    /// \brief Textual lowercase names of the tracks.
    protected: std::unordered_map<Tracks, std::string> trackNames;

    /// \brief Set new target velocity for the tracks.
    ///
    /// Descendant classes need to implement this function.
    ///
    /// Do not call this function directly, instead call SetTrackVelocity().
    ///
    /// \param[in] _left Velocity of left track.
    /// \param[in] _right Velocity of right track.
    protected: virtual void SetTrackVelocityImpl(double _left,
                                                 double _right) = 0;

    /// \brief Set new target velocity for the tracks.
    ///
    /// Descendant classes need to implement this function.
    ///
    /// \param[in] _left Velocity of left track.
    /// \param[in] _right Velocity of right track.
    protected: virtual void SetTrackVelocity(double _left, double _right);

    /// \brief Callback for setting desired body velocity.
    ///
    /// Normally, this callback converts the x/yaw message to track velocities
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
