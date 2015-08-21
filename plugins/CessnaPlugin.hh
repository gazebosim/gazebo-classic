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

#ifndef _GAZEBO_CESSNA_PLUGIN_HH_
#define _GAZEBO_CESSNA_PLUGIN_HH_

#include <array>
#include <mutex>
#include <string>
#include <sdf/sdf.hh>
#include <gazebo/common/PID.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/transport/TransportTypes.hh>

namespace gazebo
{
  /// \brief Allow moving the control surfaces of a Cessna C-172 plane. This
  /// plugin might be used with other models that have similar control surfaces.
  ///
  /// The plugin requires the following parameters:
  /// <propeller>         Name of the joint controlling the propeller spin.
  /// <propeller_max_rpm> Maximum angular speed in rpm.
  /// <left_aileron>      Name of the joint controlling the left aileron.
  /// <left_flap>         Name of the joint controlling the left flap.
  /// <right_aileron>     Name of the joint controlling the right aileron.
  /// <right_flap>        Name of the joint controlling the right flap.
  /// <elevators>         Name of the joint controlling the rear elevators.
  /// <rudder>            Name of the joint controlling the rudder.
  ///
  /// The following parameters are optional:
  /// <propeller_p_gain> P gain for the PID that controls the propeller's speed.
  /// <propeller_i_gain> I gain for the PID that controls the propeller's speed.
  /// <propeller_d_gain> D gain for the PID that controls the propeller's speed.
  /// <surfaces_p_gain> P gain for the PID that controls the position of the
  ///                   control surfaces.
  /// <surfaces_i_gain> I gain for the PID that controls the position of the
  ///                   control surfaces.
  /// <surfaces_d_gain> D gain for the PID that controls the position of the
  ///                   control surfaces.
  ///
  /// The plugin will be subscribed to the following topic:
  /// "~/<model_name>/control" The expected value is a Cessna message.
  ///
  /// The plugin will advertise the following topic with the current state:
  /// "~/<model_name>/state"
  class GAZEBO_VISIBLE CessnaPlugin : public ModelPlugin
  {
    /// \brief Constructor.
    public: CessnaPlugin();

    /// \brief Destructor.
    public: ~CessnaPlugin();

    // Documentation Inherited.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Read an SDF parameter with a joint name and initialize a pointer
    /// to this joint.
    /// \param[in] _sdfParam SDF parameter containing a joint name.
    /// \param[in] _sdf Pointer to the SDF element containing the parameters.
    /// \param[out] _joint Pointer to the joint to be initialized.
    /// \return True if the SDF parameter is found and the joint name is found,
    ///         false otherwise.
    private: bool FindJoint(const std::string &_sdfParam,
        sdf::ElementPtr _sdf, physics::JointPtr &_joint);

    /// \brief Update the control surfaces controllers.
    /// \param[in] _info Update information provided by the server.
    private: void Update(const common::UpdateInfo &_info);

    /// \brief Callback executed when a new message containing control commands
    /// is received.
    /// \param[in] _msg New message containing control commands.
    private: void OnControl(ConstCessnaPtr &_msg);

    /// \brief Update PID Joint controllers.
    /// \param[in] _dt time step size since last update.
    private: void UpdatePIDs(double _dt);

    /// \brief Publish Cessna state.
    private: void PublishState();

    /// \brief Joint indexes.
    private: static const unsigned int kLeftAileron  = 0;
    private: static const unsigned int kLeftFlap     = 1;
    private: static const unsigned int kRightAileron = 2;
    private: static const unsigned int kRightFlap    = 3;
    private: static const unsigned int kElevators    = 4;
    private: static const unsigned int kRudder       = 5;
    private: static const unsigned int kPropeller    = 6;

    /// \brief Pointer to the update event connection.
    private: event::ConnectionPtr updateConnection;

    /// \brief Node used for using Gazebo communications.
    private: transport::NodePtr node;

    /// \brief Subscriber pointer.
    private: transport::SubscriberPtr controlSub;

    /// \brief Publisher pointer.
    private: transport::PublisherPtr statePub;

    /// \brief Pointer to the model;
    private: physics::ModelPtr model;

    /// \brief Control surfaces joints.
    private: std::array<physics::JointPtr, 7> joints;

    /// \brief Max propeller RPM.
    private: int32_t propellerMaxRpm = 2500;

    /// \brief Next command to be applied to the propeller and control surfaces.
    private: std::array<float, 7>cmds;

    /// \brief Velocity PID for the propeller.
    private: common::PID propellerPID;

    /// \brief Position PID for the control surfaces.
    private: std::array<common::PID, 6> controlSurfacesPID;

    /// \brief keep track of controller update sim-time.
    private: gazebo::common::Time lastControllerUpdateTime;

    /// \brief Controller update mutex.
    private: std::mutex mutex;
  };
}
#endif
