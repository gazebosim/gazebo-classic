/*
 * Copyright 2011 Nate Koenig
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

#ifndef __VIDEO_VISUAL_HH__
#define __VIDEO_VISUAL_HH__

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
    /// \brief A visual element that displays a video as a texture
    class VideoVisual : public Visual
    {
      /// \brief Constructor
      public: VideoVisual(const std::string &_name, VisualPtr _parent);

      /// \brief Destructor
      public: virtual ~VideoVisual();

      private: void PreRender();

      private: common::Video *video;

      private: std::vector<event::ConnectionPtr> connections;
      private: Ogre::TexturePtr texture;
      private: unsigned char *imageBuffer;
      private: int width, height;
    };
    /// \}
  }
}
#endif
