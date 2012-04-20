/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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
#ifndef __GAZEBO_JOINTCONTROLLER_PLUGIN_HH__
#define __GAZEBO_JOINTCONTROLLER_PLUGIN_HH__

#include "common/common.h"
#include "msgs/msgs.h"
#include "physics/physics.h"
#include "transport/transport.h"
#include "gazebo.h"

namespace gazebo
{
  class JointControllerPlugin : public ModelPlugin
  {
    public: JointControllerPlugin();
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    public: virtual void Init();

    private: void OnUpdate();

    private: void OnJointCmd(ConstJointCmdPtr &_msg);
             
    private: transport::NodePtr node;
    private: transport::SubscriberPtr jointCmdSub;

    private: physics::ModelPtr model;
    private: std::vector<physics::JointPtr> joints;
    private: std::vector<common::PID> pids;
    private: event::ConnectionPtr updateConnection;
  };
}
#endif
