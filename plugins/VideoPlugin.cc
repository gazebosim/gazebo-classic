/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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

#include <gazebo/rendering/VideoVisual.hh>
#include <gazebo/rendering/Visual.hh>
#include "VideoPlugin.hh"

namespace gazebo
{
  /// \internal
  /// \class VideoPlugin VideoPlugin.hh
  /// \brief Private data for the VideoPlugin class.
  class VideoPluginPrivate
  {
    /// \brief Visual whose texture will be changed.
    public: rendering::VisualPtr visual;
  };
}

using namespace gazebo;

GZ_REGISTER_VISUAL_PLUGIN(VideoPlugin)

/////////////////////////////////////////////////
VideoPlugin::VideoPlugin() : dataPtr(new VideoPluginPrivate)
{
}

/////////////////////////////////////////////////
VideoPlugin::~VideoPlugin()
{
}

/////////////////////////////////////////////////
void VideoPlugin::Load(rendering::VisualPtr _visual, sdf::ElementPtr _sdf)
{
  if (!_visual || !_sdf)
  {
    gzerr << "No visual or SDF element specified. Plugin won't load." <<
        std::endl;
    return;
  }
  this->dataPtr->visual = _visual;

  if (!_sdf->HasElement("sdf"))
  {
    gzerr << "And <sdf> tag can't be found within the plugin." << std::endl;
    return;
  }

  // Get the video directory
  if (!_sdf->GetElement("sdf")->HasElement("uri"))
  {
    gzerr << "<uri> element missing from Video plugin. "
      << "The plugin will not function." << std::endl;
    return;
  }

  auto vis = new rendering::VideoVisual("VideoVisual", this->dataPtr->visual);
}

