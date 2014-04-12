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
#ifndef _GAZEBO_SPRING_TEST_PLUGIN_HH_
#define _GAZEBO_SPRING_TEST_PLUGIN_HH_

#include <string>

#include "ignition/common.hh"
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

    private: void ExplicitUpdate();

    private: ignition::common::ConnectionPtr updateConnection;

    private: physics::ModelPtr model;

    private: ignition::common::Time prevUpdateTime;

    private: physics::JointPtr jointExplicit;
    private: std::string jointExplicitName;

    /// \brief simulate spring/damper with ExplicitUpdate function
    private: double kpExplicit;

    /// \brief simulate spring/damper with ExplicitUpdate function
    private: double kdExplicit;

    private: physics::JointPtr jointImplicit;
    private: std::string jointImplicitName;

    /// \brief simulate spring/damper with Joint::SetStiffnessDamping
    private: double kpImplicit;

    /// \brief simulate spring/damper with Joint::SetStiffnessDamping
    private: double kdImplicit;
  };
}
#endif
