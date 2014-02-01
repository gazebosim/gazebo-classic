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
/* Desc: Video Visualization Class
 * Author: Nate Koenig
 */

#ifndef _VIDEO_VISUAL_HH_
#define _VIDEO_VISUAL_HH_

#include <string>
#include <vector>
#include "gazebo/rendering/Visual.hh"

namespace gazebo
{
  namespace common
  {
    class Video;
  }

  namespace rendering
  {
    /// \addtogroup gazebo_rendering
    /// \{

    /// \class VideoVisual VideoVisual.hh rendering/rendering.hh
    /// \brief A visual element that displays a video as a texture
    class VideoVisual : public Visual
    {
      /// \brief Constructor
      /// \param[in] _name Name of the video visual.
      /// \param[in] _parent Parent of the video visual.
      public: VideoVisual(const std::string &_name, VisualPtr _parent);

      /// \brief Destructor
      public: virtual ~VideoVisual();

      /// \brief PreRender event callback.
      private: void PreRender();

      /// \brief Load a video
      private: common::Video *video;

      /// \brief All the event connections.
      private: std::vector<event::ConnectionPtr> connections;

      /// \brief Texture to draw the video onto.
      private: Ogre::TexturePtr texture;

      /// \brief One frame of the viedeo.
      private: unsigned char *imageBuffer;

      /// \brief Width and height of the video.
      private: int width, height;
    };
    /// \}
  }
}
#endif
