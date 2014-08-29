/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
#ifndef _MOVING_PLANE_PLUGIN_HH_
#define _MOVING_PLANE_PLUGIN_HH_

#include <string>
#include <vector>

#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    class Wrench
    {
      /// \brief Operator =
      /// \param[in] _wrench wrench to set from.
      /// \return *this
      public: Wrench &operator =(const Wrench &_wrench)
              {
                this->force = _wrench.force;
                this->torque = _wrench.torque;
                return *this;
              }

      /// \brief Operator +
      /// \param[in] _wrench wrench to add
      /// \return *this
      public: inline Wrench &operator +(const Wrench &_wrench)
              {
                this->force += _wrench.force;
                this->torque += _wrench.torque;
                return *this;
              }

      /// \brief Operator -
      /// \param[in] _wrench wrench to subtract
      /// \return *this
      public: inline Wrench &operator -(const Wrench &_wrench)
              {
                this->force -= _wrench.force;
                this->torque -= _wrench.torque;
                return *this;
              }

      /// \brief linear forces
      public: math::Vector3 force;

      /// \brief angular torques
      public: math::Vector3 torque;

      /// \brief reference link frame
      public: physics::LinkPtr referenceFrame;
    };
  }

  class GAZEBO_VISIBLE MovingPlanePlugin : public ModelPlugin
  {
    /// \brief Constructor.
    public: MovingPlanePlugin();

    // Documentation Inherited.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    // Documentation Inherited.
    public: virtual void Init();

    /// \brief Callback for World Update events.
    private: void OnUpdate();

    /// \brief Add a link to to be moved.
    /// \param[in] _link Link to be moved.
    private: void AddLink(physics::LinkPtr _link);

    /// \brief Connection to World Update events.
    private: event::ConnectionPtr updateConnection;

    /// \brief Pointer to world.
    private: physics::WorldPtr world;

    /// \brief Pointer to physics engine.
    private: physics::PhysicsEnginePtr physics;

    /// \brief Pointer to model containing plugin.
    private: physics::ModelPtr model;

    /// \brief Direction of plane movement.
    private: math::Vector3 direction;

    /// \brief Distance to move along the direction vector.
    private: double distance;

    /// \brief Name of model containing plugin.
    private: std::string modelName;

    /// \brief Initial pose of the model.
    private: math::Pose modelInitPose;

    /// \brief SDF for this plugin;
    private: sdf::ElementPtr sdf;

    /// \brief Map of link names to the link pointer.
    private: std::map<std::string, physics::LinkPtr> links;

    /// \brief Position PID controllers, one for each axis.
    private: std::map<std::string, std::vector<common::PID> > posPids;

    /// \brief Orientation PID controllers, one for each axis.
    private: std::map<std::string, std::vector<common::PID> > rotPids;

    /// \brief Wrenches applied to links.
    private: std::map<std::string, physics::Wrench> wrenches;

    /// \brief target link poses
    private: std::map<std::string, math::Pose> targetPoses;

    /// \brief Last time the controller was updated.
    private: common::Time prevUpdateTime;
  };
}
#endif  // ifndef _MUD_PLUGIN_HH_
