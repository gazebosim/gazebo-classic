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
/* Desc: A 4-wheeled vehicle
 * Author: Nate Koenig
 */

#ifndef _GAZEBO_VEHICLE_PLUGIN_HH_
#define _GAZEBO_VEHICLE_PLUGIN_HH_

#include <string>
#include <vector>

#include "gazebo/common/common.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/gazebo.hh"

namespace gazebo
{
  class VehiclePlugin : public ModelPlugin
  {
    /// \brief Constructor
    public: VehiclePlugin();

    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    public: virtual void Init();

    private: void OnUpdate();

    private: void OnVelMsg(ConstPosePtr &_msg);

    private: std::vector<event::ConnectionPtr> connections;

    private: physics::ModelPtr model;
    private: physics::LinkPtr chassis;
    private: std::vector<physics::JointPtr> joints;
    private: physics::JointPtr gasJoint, brakeJoint;
    private: physics::JointPtr steeringJoint;

    private: math::Vector3 velocity;

    private: transport::NodePtr node;
    private: transport::SubscriberPtr velSub;

    private: double frontPower, rearPower;
    private: double maxSpeed;
    private: double wheelRadius;

    private: double steeringRatio;
    private: double tireAngleRange;
    private: double maxGas, maxBrake;

    private: double aeroLoad;
    private: double swayForce;
  };
}
#endif
