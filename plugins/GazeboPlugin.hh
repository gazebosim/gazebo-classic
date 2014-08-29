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

#ifndef _GAZEBO_GAZEBO_PLUGIN_HH_
#define _GAZEBO_GAZEBO_PLUGIN_HH_

#include <string>
#include <vector>

#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  class GAZEBO_VISIBLE GazeboPlugin : public ModelPlugin
  {
    /// \brief Constructor
    public: GazeboPlugin();
    public: ~GazeboPlugin();

    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    public: virtual void Init();

    private: void OnUpdate();

    private: void OnMsg(ConstPosePtr &_msg);

    private: std::vector<event::ConnectionPtr> connections;

    private: physics::WorldPtr world;
    private: physics::ModelPtr model;

    private: transport::NodePtr node;
    private: transport::SubscriberPtr sub;
  };
}
#endif
