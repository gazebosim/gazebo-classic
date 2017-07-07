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

#include <gazebo/rendering/Scene.hh>
#include <gazebo/rendering/UserCamera.hh>
#include <gazebo/rendering/Visual.hh>
#include <gazebo/rendering/LensFlare.hh>
#include "SceneVisualPlugin.hh"

namespace gazebo
{
  /// \internal
  /// \class SceneVisualPlugin SceneVisualPlugin.hh
  /// \brief Private data for the SceneVisualPlugin class.
  class SceneVisualPluginPrivate
  {
    /// \brief Parent visual 
    public: rendering::VisualPtr visual;

    /// \brief Lens flare
    public: rendering::LensFlarePtr lensFlare;

    /// \brief Connects to rendering update event.
    public: event::ConnectionPtr updateConnection;
  };
}

using namespace gazebo;

GZ_REGISTER_VISUAL_PLUGIN(SceneVisualPlugin)

/////////////////////////////////////////////////
SceneVisualPlugin::SceneVisualPlugin() : dataPtr(new SceneVisualPluginPrivate)
{
}

/////////////////////////////////////////////////
SceneVisualPlugin::~SceneVisualPlugin()
{
}

/////////////////////////////////////////////////
void SceneVisualPlugin::Load(rendering::VisualPtr _visual, sdf::ElementPtr _sdf)
{
  if (!_visual || !_sdf)
  {
    gzerr << "No visual or SDF element specified. Plugin won't load." <<
        std::endl;
    return;
  }
  this->dataPtr->visual = _visual;

  rendering::ScenePtr scene = this->dataPtr->visual->GetScene();
  if (scene->UserCameraCount() == 0u)
    return;

  rendering::UserCameraPtr userCamera = scene->GetUserCamera(0);

  this->dataPtr->lensFlare.reset(new rendering::LensFlare);
  this->dataPtr->lensFlare->SetCamera(userCamera);
}
