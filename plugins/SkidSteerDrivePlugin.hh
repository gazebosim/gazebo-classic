/*
 * Copyright 2012 Open Source Robotics Foundation
 * Copyright 2013 Dereck Wonnacott
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
#ifndef __GAZEBO_SkidSteerDrive_PLUGIN_HH__
#define __GAZEBO_SkidSteerDrive_PLUGIN_HH__

#include "common/common.hh"
#include "physics/physics.hh"
#include "transport/TransportTypes.hh"
#include "gazebo.hh"

#define NUMBER_OF_WHEELS 4

namespace gazebo
{
  class SkidSteerDrivePlugin : public ModelPlugin
  {
    public: SkidSteerDrivePlugin();
    public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    
    public: enum {RIGHT_FRONT, RIGHT_REAR, LEFT_FRONT, LEFT_REAR};

    private: int RegisterJoint(int index, std::string name);
    private: void OnVelMsg(ConstPosePtr &_msg);

    private: transport::NodePtr node;
    private: transport::SubscriberPtr velSub; 

    private: physics::ModelPtr model;
    private: physics::JointPtr Joints[NUMBER_OF_WHEELS];

    private: double MaxForce;
    private: double wheelSeparation;
    private: double wheelRadius;
    
  };
}
#endif
