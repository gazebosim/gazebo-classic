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
#ifndef _GAZEBO_SkidSteerDrive_PLUGIN_HH_
#define _GAZEBO_SkidSteerDrive_PLUGIN_HH_

#include <string>

#include "gazebo/common/common.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/gazebo.hh"

#define NUMBER_OF_WHEELS 4

namespace gazebo
{
  // \class SkidSteerDrivePlugin SkidSteerDrivePlugin.hh
  /// \brief A gazebo model plugin that controls a four wheel skid-steer
  ///        robot via a gazebo topic. See the Pioneer3AT model in the
  ///        OSRF model database for an example use case.
  class SkidSteerDrivePlugin : public ModelPlugin
  {
    /// \brief Default Contstuctor
    public: SkidSteerDrivePlugin();

    /// \brief Called when the plugin is loaded
    /// \param[in] _model Pointer to the model for which the plugin is loaded
    /// \param[in] _sdf Pointer to the SDF for _model
    public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \def ID for each of the four wheels
    public: enum {RIGHT_FRONT, RIGHT_REAR, LEFT_FRONT, LEFT_REAR};

    /// \brief Associates a joint to each of the wheels
    /// \param[in] _index Internal wheel index (Zero based)
    /// \param[in] _name Name wheel joint
    /// \return {0: Success, else: Error}
    private: int RegisterJoint(int _index, const std::string &_name);

    /// \brief Callback for gazebo topic
    /// \param[in] _msg Pose message from external publisher
    private: void OnVelMsg(ConstPosePtr &_msg);

    /// \brief Node for subscriber
    private: transport::NodePtr node;

    /// \brief Gazebo topic subscriber
    private: transport::SubscriberPtr velSub;

    /// \brief Pointer to the model which this plugin is attached
    private: physics::ModelPtr model;

    /// \brief Pointer to each wheel joint
    private: physics::JointPtr joints[NUMBER_OF_WHEELS];

    /// \brief Max force limit for each wheel joint (Default 5.0)
    private: double maxForce;

    /// \brief Distance between wheels on the same axis (Determined from SDF)
    private: double wheelSeparation;

    /// \brief Radius of the wheels (Determined from SDF)
    private: double wheelRadius;
  };
}
#endif
