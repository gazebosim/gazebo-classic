/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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

#include "gazebo/rendering/RenderingIface.hh"
#include "gazebo/rendering/Scene.hh"
#include "plugins/ShadowPlugin.hh"

using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(ShadowPlugin)

/////////////////////////////////////////////////
ShadowPlugin::ShadowPlugin()
: SensorPlugin()
{
}

/////////////////////////////////////////////////
ShadowPlugin::~ShadowPlugin()
{
}

/////////////////////////////////////////////////
void ShadowPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
  rendering::ScenePtr scene = rendering::get_scene();
  if (!scene)
  {
    gzerr << "Scene is null" << std::endl;
    return;
  }
  unsigned int shadowTextureSize = 2048;
  if (_sdf->HasElement("shadow_texture_size"))
    shadowTextureSize = _sdf->Get<unsigned int>("shadow_texture_size");
  scene->SetShadowTextureSize(shadowTextureSize);
}
