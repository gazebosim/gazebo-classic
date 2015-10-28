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

#ifndef _VIDEO_VISUAL_PRIVATE_HH_
#define _VIDEO_VISUAL_PRIVATE_HH_

#include <string>
#include <vector>
#include "gazebo/rendering/VisualPrivate.hh"

namespace gazebo
{
  namespace common
  {
    class Video;
  }

  namespace rendering
  {
    /// \brief Private data for the Video Visual class.
    class VideoVisualPrivate : public VisualPrivate
    {
      /// \brief Load a video
      public: common::Video *video;

      /// \brief All the event connections.
      public: std::vector<event::ConnectionPtr> connections;

      /// \brief Texture to draw the video onto.
      public: Ogre::TexturePtr texture;

      /// \brief One frame of the viedeo.
      public: unsigned char *imageBuffer;

      /// \brief Width of the video.
      public: int width;

      /// \brief Height of the video.
      public: int height;
    };
  }
}
#endif
