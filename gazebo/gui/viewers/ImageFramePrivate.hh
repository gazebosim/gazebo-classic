/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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
#ifndef GAZEBO_GUI_VIEWERS_IMAGEFRAMEPRIVATE_HH_
#define GAZEBO_GUI_VIEWERS_IMAGEFRAMEPRIVATE_HH_

#include <mutex>
#include "gazebo/gui/qt.h"

namespace gazebo
{
  namespace gui
  {
    class ImageFramePrivate
    {
      /// \brief The image to draw.
      public: QImage image;

      /// \brief Mutex for protecting the image.
      public: std::mutex mutex;

      /// \brief Depth camera image data buffer.
      public: float *depthBuffer = nullptr;

      /// \brief camera image data buffer 16 bit format.
      public: uint16_t *imageBufferHalf = nullptr;

      /// \brief Camera image data buffer.
      public: unsigned char *imageBuffer = nullptr;
    };
  }
}

#endif
