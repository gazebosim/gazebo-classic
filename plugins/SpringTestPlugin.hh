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
#ifndef __GAZEBO_SPRING_TEST_PLUGIN_HH__
#define __GAZEBO_SPRING_TEST_PLUGIN_HH__

#include "gazebo/common/common.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/gazebo.hh"

namespace gazebo
{
  class SpringTestPlugin : public ModelPlugin
  {
    public: SpringTestPlugin();
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    public: virtual void Init();

    private: void OnUpdate();

    private: event::ConnectionPtr updateConnection;

    private: physics::ModelPtr model;

    private: common::Time prevUpdateTime;

    private: double k;
  };
}
#endif
