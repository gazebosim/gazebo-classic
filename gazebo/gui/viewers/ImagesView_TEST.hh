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

#ifndef _GAZEBO_GUI_VIEWERS_IMAGESVIEW_TEST_HH_
#define _GAZEBO_GUI_VIEWERS_IMAGESVIEW_TEST_HH_

#include <string>

#include "gazebo/gui/QTestFixture.hh"
#include "gazebo/gui/viewers/ImagesView.hh"

/// \brief A test class for the ImagesView widget.
class ImagesView_TEST : public QTestFixture
{
  Q_OBJECT

  /// \brief Test construction and usage in an empty world
  private slots: void Construction();

  /// \brief Test switching between cameras
  private slots: void Switch();

  /// \brief Switch the images view topic, and make sure the images appear.
  /// \param[in] _view Pointer to the ImagesView widget.
  /// \param[in] _topicName Name of the topic publishing images.
  /// \param[in] _count Number of expected images on the topic.
  private: void SetTopic(gazebo::gui::ImagesView *_view,
                         const std::string &_topicName, int _count);
};
#endif
