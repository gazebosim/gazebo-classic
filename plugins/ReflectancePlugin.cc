/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#include <boost/algorithm/string/replace.hpp>

#include <gazebo/common/ModelDatabase.hh>
#include <gazebo/rendering/Camera.hh>
#include <gazebo/rendering/RenderEngine.hh>
#include <gazebo/rendering/RenderTypes.hh>
#include <gazebo/rendering/Scene.hh>
#include <gazebo/rendering/UserCamera.hh>
#include <gazebo/rendering/Visual.hh>
#include <gazebo/rendering/ogre_gazebo.h>

#include "ReflectancePlugin.hh"

namespace gazebo
{
using namespace gazebo;

GZ_REGISTER_VISUAL_PLUGIN(ReflectancePlugin)

/////////////////////////////////////////////////
ReflectancePlugin::ReflectancePlugin()
{
}

/////////////////////////////////////////////////
ReflectancePlugin::~ReflectancePlugin()
{
}

/////////////////////////////////////////////////
void ReflectancePlugin::Load(rendering::VisualPtr _visual,
    sdf::ElementPtr _sdf)
{
  // Check if attached to valid visual
  if (!_visual || !_sdf)
  {
    gzerr << "[ReflectancePlugin] Invalid visual or SDF element." << std::endl;
    return;
  }

  // Unique name
  if (!_sdf->HasElement("reflectance_map"))
  {
    gzerr << "<reflectance_map> is required" << std::endl;
    return;
  }

  sdf::ElementPtr reflectanceMapElem = _sdf->GetElement("reflectance_map");
  std::string name = reflectanceMapElem->Get<std::string>("name");
  std::string uri = reflectanceMapElem->Get<std::string>("uri");

  if (!uri.empty())
  {
    rendering::RenderEngine::Instance()->AddResourcePath(uri);
  }
  else
  {
    if (!name.empty())
    {
      rendering::RenderEngine::Instance()->AddResourcePath(name);
    }
    else
    {
      gzerr << "<name> inside <reflectance_map> tag is required" << std::endl;
      return;
    }
  }

  _visual->GetSceneNode()->getUserObjectBindings()
                .setUserAny(std::string("reflectance_map"),
                            Ogre::Any(std::string(name)));
}
}
