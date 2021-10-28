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

#include <gazebo/rendering/RenderingIface.hh>
#include <gazebo/rendering/Scene.hh>
#include <gazebo/rendering/Visual.hh>

#include "HeightmapLODPlugin.hh"

namespace gazebo
{
  /// \internal
  /// \class HeightmapLODPlugin HeightmapLODPlugin.hh
  /// \brief Private data for the HeightmapLODPlugin class.
  class HeightmapLODPluginPrivate
  {
    /// \brief Parent visual
    public: rendering::VisualPtr viusal;

    /// \brief Default LOD value to set to.
    public: unsigned int lod = 3u;

    /// \brief Default skirt length
    public: double skirtLength = 1.0;
  };
}

using namespace gazebo;

GZ_REGISTER_VISUAL_PLUGIN(HeightmapLODPlugin)

/////////////////////////////////////////////////
HeightmapLODPlugin::HeightmapLODPlugin()
    : dataPtr(new HeightmapLODPluginPrivate)
{
}

/////////////////////////////////////////////////
void HeightmapLODPlugin::Load(rendering::VisualPtr _visual,
    sdf::ElementPtr _sdf)
{
  if (!_visual || !_sdf)
  {
    gzerr << "No visual or SDF element specified. Plugin won't load." <<
        std::endl;
    return;
  }

  auto scene = _visual->GetScene();

  // Pointer to the <server/> or <gui/> element, depending on the result
  // of scene->IsServer() or nullptr if they don't exist.
  sdf::ElementPtr serverGui;
  if (scene->IsServer() && _sdf->HasElement("server"))
  {
    serverGui = _sdf->GetElement("server");
  }
  else if (!scene->IsServer() && _sdf->HasElement("gui"))
  {
    serverGui = _sdf->GetElement("gui");
  }

  if (_sdf->HasElement("lod"))
  {
    this->dataPtr->lod = _sdf->Get<unsigned int>("lod");
  }
  else if (serverGui && serverGui->HasElement("lod"))
  {
    this->dataPtr->lod = serverGui->Get<unsigned int>("lod");
  }

  if (_sdf->HasElement("skirt_length"))
  {
    this->dataPtr->skirtLength = _sdf->Get<double>("skirt_length");
  }
  else if (serverGui && serverGui->HasElement("skirt_length"))
  {
    this->dataPtr->skirtLength = serverGui->Get<double>("skirt_length");
  }

  scene->SetHeightmapLOD(this->dataPtr->lod);
  scene->SetHeightmapSkirtLength(this->dataPtr->skirtLength);
}
