/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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
#ifndef _GAZEBO_GUI_VIEWERS_IMAGESVIEW_PRIVATE_HH_
#define _GAZEBO_GUI_VIEWERS_IMAGESVIEW_PRIVATE_HH_

#include <vector>
#include <utility>

#include "gazebo/gui/viewers/ImageFrame.hh"
#include "gazebo/gui/qt.h"

namespace gazebo
{
  namespace gui
  {
    /// \brief Private data for the ImagesView class.
    class ImagesViewPrivate
    {
      /// \brief Mutex to protect variables.
      public: std::mutex mutex;

      /// \brief Set to true to clear the images from the widget
      public: bool clearImages;

      /// \brief Vector of image sizes to add
      public: std::vector<std::pair<int, int> > addImage;
    };
  }
}

#endif
