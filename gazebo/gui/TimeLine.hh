/*
 * Copyright 2014 Open Source Robotics Foundation
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
#ifndef _GAZEBO_TIMELINE_WIDGET_HH_
#define _GAZEBO_TIMELINE_WIDGET_HH_

#include "gazebo/common/Time.hh"
#include "gazebo/gui/qt.h"

namespace gazebo
{
  namespace gui
  {
    class TimeLine : public QSlider
    {
      /// \brief Constructor
      /// \param[in] _parent Pointer to the parent widget.
      public: TimeLine(QWidget *_parent = 0);

      /// \brief Destructor
      public: virtual ~TimeLine();

      public: void SetTime(const common::Time &_time);

      protected: virtual void mousePressEvent(QMouseEvent *_event);
      protected: virtual void mouseDoubleClickEvent(QMouseEvent *_event);
      protected: virtual void mouseMoveEvent(QMouseEvent *_event);
      protected: virtual void mouseReleaseEvent(QMouseEvent *_event);

      protected: virtual void paintEvent(QPaintEvent *_evt);

      private: common::Time time;
    };
  }
}

#endif
