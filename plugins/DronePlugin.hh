/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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
#ifndef _GAZEBO_DRONE_PLUGIN_HH_
#define _GAZEBO_DRONE_PLUGIN_HH_

#include <string>
#include <vector>

#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/common/PID.hh"

namespace gazebo
{
  namespace util
  {
    class Joystick;
  }

  class WrenchHelper
  {
    /// \brief Operator =
    /// \param[in] _wrench wrench to set from.
    /// \return *this
    public: WrenchHelper &operator =(const WrenchHelper &_wrench);

    /// \brief Operator +
    /// \param[in] _wrench wrench to add
    /// \return *this
    public: inline WrenchHelper &operator +(const WrenchHelper &_wrench);

    /// \brief Operator -
    /// \param[in] _wrench wrench to subtract
    /// \return *this
    public: inline WrenchHelper &operator -(const WrenchHelper &_wrench);

    /// \brief linear forces
    public: math::Vector3 force;

    /// \brief angular torques
    public: math::Vector3 torque;

    /// \brief reference link frame
    public: physics::LinkPtr referenceFrame;
  };

  /// \brief A plugin that simulates lift and drag.
  class DronePlugin : public ModelPlugin
  {
    /// \brief Constructor.
    public: DronePlugin();

    // Documentation Inherited.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Callback for World Update events.
    private: virtual void OnUpdate();

    /// \brief Connection to World Update events.
    private: event::ConnectionPtr updateConnection;

    /// \brief Pointer to model containing plugin.
    private: physics::ModelPtr model;
    private: physics::HeightmapShapePtr heightmap;

    /// \brief Pointer to the joystick interface.
    private: util::Joystick *joy;

    private: double yawSpeed;
    private: double pitchSpeed;

    private: math::Vector3 velocity;

    private: sdf::ElementPtr sdf;

    /// \brief PID for controlling base link position in world frame.
    private: common::PID posPid;

    /// \brief PID for controlling base link orientation in world frame.
    private: common::PID rotPid;

    /// \brief base link to be pid'd
    private: physics::LinkPtr baseLink;

    /// \brief: a pointer to the base link "floating" joint for positioning
    /// the link in space. This joint is provides implicit viscous damping
    /// of the base link motion.
    private: physics::JointPtr baseJoint;

    /// \brief: target link control pose
    private: math::Pose targetBaseLinkPose;

    /// \brief: mutex for writing targetBaseLinkPose
    private: common::Time lastSimTime;

    /// \brief force for moving the arm base link to target location.
    private: WrenchHelper wrench;

    public: physics::RayShapePtr testRay;
  };
}
#endif
