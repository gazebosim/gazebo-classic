/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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

#ifndef _GAZEBO_FOOSBALL_DEMO_PLUGIN_HH_
#define _GAZEBO_FOOSBALL_DEMO_PLUGIN_HH_

#include <sdf/sdf.hh>
#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/PhysicsTypes.hh"

namespace gazebo
{

  /// \brief A world plugin that...
  class GAZEBO_VISIBLE FoosballDemoPlugin : public WorldPlugin
  {
    /// \brief Constructor.
    public: FoosballDemoPlugin();

    // Documentation Inherited.
    public: virtual void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

    /// \brief World pointer.
    private: physics::WorldPtr world;

    /// \brief SDF for this plugin.
    private: sdf::ElementPtr sdf;
  };
}
#endif
