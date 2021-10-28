/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include "WorldSpawnModelPlugin.hh"

namespace gazebo
{
  /// \brief Private data class for WorldSpawnModelPlugin
  class WorldSpawnModelPluginPrivate
  {
  };
}

using namespace gazebo;

GZ_REGISTER_WORLD_PLUGIN(WorldSpawnModelPlugin)

/////////////////////////////////////////////////
WorldSpawnModelPlugin::WorldSpawnModelPlugin()
  : dataPtr(new WorldSpawnModelPluginPrivate)
{
}

/////////////////////////////////////////////////
void WorldSpawnModelPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
  if (!_sdf->HasElement("sdf"))
  {
    gzerr << "No <sdf> detected. Nothing will be spawned." << std::endl;
    return;
  }

  sdf::ElementPtr sdfElem = _sdf->GetElement("sdf");
  if (!sdfElem->HasElement("model"))
  {
    gzerr << "No <model> detected in <sdf>. Nothing will be spawned."
          << std::endl;
    return;
  }
  sdf::ElementPtr modelElem = sdfElem->GetElement("model");

  std::string modelName;
  if (modelElem->HasAttribute("name"))
  {
    modelName = modelElem->Get<std::string>("name");
  }

  gzmsg << "Spawning model with name [" << modelName << "]" << std::endl;
  _world->InsertModelString(sdfElem->ToString(""));
}
