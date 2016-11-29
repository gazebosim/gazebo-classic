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
#ifndef GAZEBO_GUI_VIEWERS_IMAGESVIEW_HH_
#define GAZEBO_GUI_VIEWERS_IMAGESVIEW_HH_

#include <string>

#include "gazebo/util/system.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/gui/qt.h"
#include "gazebo/gui/viewers/TopicView.hh"

namespace gazebo
{
  namespace gui
  {
    class ImagesViewPrivate;

    class GZ_GUI_VISIBLE ImagesView : public TopicView
    {
      Q_OBJECT

      /// \brief Constructor
      /// \param[in] _parent Pointer to the parent widget.
      public: ImagesView(QWidget *_parent = nullptr);

      /// \brief Destructor
      public: virtual ~ImagesView();

      // Documentation inherited
      public: virtual void SetTopic(const std::string &_topicName);

      // Documentation inherited
      private: virtual void UpdateImpl();

      /// \brief Receives incoming image messages.
      /// \param[in] _msg New image message.
      private: void OnImages(ConstImagesStampedPtr &_msg);

      /// \brief Add a new QImage and QLabel to the list.
      /// \param[in] _width Width of the image.
      /// \param[in] _height Height of the image.
      private: void AddImage(int _width, int _height);

      /// \brief Private data.
      private: ImagesViewPrivate *dataPtr;
    };
  }
}
#endif
