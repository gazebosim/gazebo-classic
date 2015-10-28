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
#ifndef _IMAGESVIEW_PRIVATE_HH_
#define _IMAGESVIEW_PRIVATE_HH_

#include <vector>
#include <utility>
#include <boost/thread/mutex.hpp>

#include "gazebo/gui/viewers/ImageFrame.hh"
#include "gazebo/gui/qt.h"

namespace gazebo
{
  namespace gui
  {
    /// \brief Private data for the ImagesView class.
    class ImagesViewPrivate
    {
      /// \brief Storage mechansim for image data.
      public: std::vector<ImageFrame *> images;

      /// \brief Pointer to the frame containing the images
      public: QGridLayout *frameLayout;

      /// \brief Mutex to protect the image vectors
      public: boost::mutex mutex;

      /// \brief Set to true to clear the images from the widget
      public: bool clearImages;

      /// \brief Vector of image sizes to add
      public: std::vector<std::pair<int, int> > addImage;
    };
  }
}

#endif
