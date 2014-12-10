/*
 * Copyright (C) 2014 Open Source Robotics Foundation
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

#ifndef _GAZEBO_ACTOR_PLUGIN_HH_
#define _GAZEBO_ACTOR_PLUGIN_HH_

#include <string>
#include <vector>

#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  class GAZEBO_VISIBLE ActorPlugin : public ModelPlugin
  {
    /// \brief Constructor
    public: ActorPlugin();

    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    public: virtual void Init();

    private: void OnUpdate(const common::UpdateInfo &_info);

    private: void ChooseNewTarget();
    private: void HandleObstacles(math::Vector3 &_pos);

    private: physics::ActorPtr actor;
    private: physics::WorldPtr world;

    private: math::Vector3 velocity;

    private: transport::NodePtr node;

    private: std::vector<event::ConnectionPtr> connections;

    private: math::Vector3 target;
    private: common::Time lastUpdate;
  };
}
#endif
