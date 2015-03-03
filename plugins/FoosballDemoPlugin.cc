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

#include <sdf/sdf.hh>
#include "gazebo/common/Assert.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/PhysicsTypes.hh"
#include "plugins/FoosballDemoPlugin.hh"

using namespace gazebo;

GZ_REGISTER_WORLD_PLUGIN(FoosballDemoPlugin)

/////////////////////////////////////////////////
FoosballDemoPlugin::FoosballDemoPlugin()
{
}

/////////////////////////////////////////////////
void FoosballDemoPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_world, "FoosballDemoPlugin _world pointer is NULL");
  this->world = _world;
  GZ_ASSERT(_sdf, "FoosballDemoPlugin _sdf pointer is NULL");
  this->sdf = _sdf;
}
