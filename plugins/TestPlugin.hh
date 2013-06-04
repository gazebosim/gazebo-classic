/*
 * Copyright 2012 Open Source Robotics Foundation
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

#ifndef __GAZEBO_TEST_PLUGIN_HH__
#define __GAZEBO_TEST_PLUGIN_HH__

#include <string>
#include <vector>

#include "common/common.hh"
#include "physics/physics.hh"
#include "transport/transport.hh"
#include "gazebo.hh"

namespace gazebo
{
  class TestPlugin : public ModelPlugin
  {
    /// \brief Constructor
    public: TestPlugin();

    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    private: void CreateJoint(physics::LinkPtr _centerLink,
                 physics::LinkPtr _extLink);

    private: std::list<physics::JointPtr> myJoints;
    private: physics::ModelPtr model;
  };
}
#endif
