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

#ifndef GAZEBO_WHEELTRACKEDVEHICLEPLUGIN_HH
#define GAZEBO_WHEELTRACKEDVEHICLEPLUGIN_HH


#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "gazebo/physics/Joint.hh"

#include "plugins/TrackedVehiclePlugin.hh"

namespace gazebo
{
  /// \class WheelTrackedVehiclePlugin WheelTrackedVehiclePlugin.hh
  /// \brief An approximate model of non-deformable tracks emulated by wheels.
  /// \since 8.1
  ///
  /// Inserting this model in simulation automatically sets the friction model
  /// to "cone_model", which is required for the model to correctly steer.
  ///
  /// The plugin processes the following parameters, plus the common parameters
  /// defined in TrackedVehiclePlugin.
  ///
  /// <left_joint>  Name of the revolute joint connecting a left wheel (should
  ///               appear multiple times, once for each left wheel).
  /// <right_joint>  Name of the revolute joint connecting a right wheel (should
  ///               appear multiple times, once for each right wheel).
  /// <default_wheel_radius>  The radius used for wheels where radius
  ///                         autodetection fails (default is 0.5 meters).
  class WheelTrackedVehiclePlugin :
    public TrackedVehiclePlugin
  {
    public: WheelTrackedVehiclePlugin() = default;

    public: virtual ~WheelTrackedVehiclePlugin() = default;

    /// \brief Called when the plugin is loaded
    /// \param[in] model Pointer to the model for which the plugin is loaded
    /// \param[in] _sdf Pointer to the SDF for _model
    public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

    /// \brief Initialize the plugin.
    public: void Init() override;

    /// \brief Reset the plugin.
    public: void Reset() override;

    /// \brief Set new target velocity for the tracks.
    ///
    /// \param[in] _left Velocity of left track.
    /// \param[in] _right Velocity of right track.
    protected: void SetTrackVelocityImpl(double _left, double _right) override;

    /// \brief Update surface parameters of the tracks to correspond to the
    ///        values set in this plugin.
    protected: void UpdateTrackSurface() override;

    /// \brief Load a wheel connected to joint named jointName and append it to
    ///        this->wheels[track]
    ///
    /// \param[in] _model The model to work with.
    /// \param[in] _track The track this wheel belongs to.
    /// \param[in] _jointName Name of the wheel joint.
    protected: void LoadWheel(physics::ModelPtr &_model, Tracks &_track,
                              const std::string &_jointName);

    /// \struct WheelInfo
    /// \brief Holds information about each wheel.
    protected: struct WheelInfo
    {
      /// \brief The hinge joint connecting the wheel to the track/body.
      physics::JointPtr joint;

      /// \brief Name of the hinge joint.
      std::string jointName;

      /// \brief Radius of the wheel (used to convert linear to angular speed).
      // cppcheck-suppress unusedStructMember
      double radius;
    };

    typedef std::shared_ptr<WheelInfo> WheelInfoPtr;
    typedef std::vector<WheelInfoPtr> WheelInfo_V;

    /// \brief The wheels on the LEFT/RIGHT track.
    protected: std::unordered_map<Tracks, WheelInfo_V> wheels;

    /// \brief Desired velocities of the tracks.
    protected: std::unordered_map<Tracks, double> trackVelocity;

    /// \brief The radius (in meters) used for wheels where autodetection fails.
    protected: double defaultWheelRadius;

    /// \brief Pointer to the world the model lives in.
    protected: physics::WorldPtr world;

    /// \brief Mutex to protect updates
    protected: std::mutex mutex;

    private: event::ConnectionPtr updateConnection;

    private: void OnUpdate();
  };
}

#endif
