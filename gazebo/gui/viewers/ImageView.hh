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
#ifndef GAZEBO_GUI_VIEWERS_IMAGEVIEW_HH_
#define GAZEBO_GUI_VIEWERS_IMAGEVIEW_HH_

#include <string>

#include "gazebo/common/Time.hh"
#include "gazebo/msgs/msgs.hh"

#include "gazebo/transport/TransportTypes.hh"

#include "gazebo/gui/qt.h"
#include "gazebo/gui/viewers/ImageFrame.hh"
#include "gazebo/gui/viewers/TopicView.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace gui
  {
    class ImageViewPrivate;

    class GZ_GUI_VISIBLE ImageView : public TopicView
    {
      Q_OBJECT

      /// \brief Constructor
      /// \param[in] _parent Pointer to the parent widget.
      public: ImageView(QWidget *_parent = NULL);

      /// \brief Destructor
      public: virtual ~ImageView();

      // Documentation inherited
      public: virtual void SetTopic(const std::string &_topicName);

      /// \brief Receives incoming image messages.
      /// \param[in] _msg New image message.
      public: void OnImage(ConstImageStampedPtr &_msg);

      /// \brief Private data.
      private: ImageViewPrivate *dataPtr;
    };
  }
}
#endif
