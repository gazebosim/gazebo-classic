/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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
#ifndef GAZEBO_PLUGINS_HARNESSPLUGIN_HH_
#define GAZEBO_PLUGINS_HARNESSPLUGIN_HH_

#include <memory>
#include <string>
#include <vector>

#include "gazebo/common/Plugin.hh"

namespace gazebo
{
  // Forward declare private data class
  class HarnessPluginPrivate;

  /// \brief This plugin is designed to lower a model at a controlled rate.
  /// Joints between a harness model and a model to lower are created
  /// according to SDF provided to this plugin.
  ///
  /// A winch joint and detach joint can be specified. The winch joint,
  /// ideally a prismatic joint, has a PID controller. The detach joint,
  /// which can be the same joint as the winch joint, is detached on a given
  /// signal.
  ///
  /// Three topics are created:
  ///
  ///  1. ~/<plugin_model_name>/harness/velocity
  ///      - Message Type: GzString, expected to be a float
  ///      - Purpose: Set target winch velocity
  ///
  ///  2. ~/<plugin_model_name>/harness/detach
  ///      - Message Type: GzString, expected to be a bool ("true")
  ///      - Purpose: Detach the <detach> joint.
  ///
  ///  3. ~/<plugin_model_name>/harness/attach
  ///      - Message Type: Pose, world pose to be set for child link
  ///        before attaching
  ///      - Purpose: Attach the joint at the specified world pose
  ///
  /// For an example refer to:
  ///   - World file: worlds/harness.world
  ///   - Code: examples/stand_alone/harness
  class GAZEBO_VISIBLE HarnessPlugin : public ModelPlugin
  {
    /// \brief Constructor.
    public: HarnessPlugin();

    /// \brief Destructor.
    public: virtual ~HarnessPlugin();

    // Documentation Inherited.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    // Documentation Inherited.
    public: virtual void Init();

    /// \brief Set the target winch velocity
    /// \param[in] _value Target winch velocity.
    public: void SetWinchVelocity(const float _value);

    /// \brief Get the current winch velocity
    /// \return Velocity of the winch joint
    public: double WinchVelocity() const;

    /// \brief Move the child link to the specified pose and recreate the
    /// harness joint.
    /// \param[in] _pose Desired world pose of child link before harnessing
    public: void Attach(const ignition::math::Pose3d &_pose);

    /// \brief Detach the <detach> joint. Once the joint is detached, it
    /// can be reattached with the Attach method.
    public: void Detach();

    /// \brief Attach harness joints at current robot pose.
    /// \param[in] _model Pointer to parent model.
    private: void Attach();

    /// \brief Callback for World Update events.
    /// \param[in] _info Update information
    private: void OnUpdate(const common::UpdateInfo &_info);

    /// \brief Velocity control callback.
    /// \param[in] _msg Message data, interpreted as a float
    private: void OnVelocity(ConstGzStringPtr &_msg);

    /// \brief Attach callback.
    /// \param[in] _msg Pose where attachment should occur.
    private: void OnAttach(ConstPosePtr &_msg);

    /// \brief Detach callback.
    /// \param[in] _msg Message data, interpreted as a bool
    private: void OnDetach(ConstGzStringPtr &_msg);

    /// \brief Get the index of a joint with the given name.
    /// \param[in] _name Name of the joint to find.
    /// \return Index into this->jointsto
    private: int JointIndex(const std::string &_name) const;

    /// \internal
    /// \brief Pointer to private data.
    private: std::unique_ptr<HarnessPluginPrivate> dataPtr;
  };
}
#endif
