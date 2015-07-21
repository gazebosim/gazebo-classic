/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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
#ifndef _GAZEBO_LOG_PLAY_WIDGET_PRIVATE_HH_
#define _GAZEBO_LOG_PLAY_WIDGET_PRIVATE_HH_

#include "gazebo/gui/qt.h"

namespace gazebo
{
  namespace gui
  {
    /// \internal
    /// \brief Private data for the LogPlayWidget class
    class LogPlayWidgetPrivate
    {
      /// \brief Paused state of the simulation.
      public: bool paused;

      /// \brief Paused state of the simulation.
      public: TimePanel *timePanel;

      /// \brief Log start time.
      public: common::Time startTime;

      /// \brief Log end time.
      public: common::Time endTime;

      /// \brief Log current time.
      public: common::Time currentTime;

      /// \brief If log is less than 1 hour long.
      public: bool lessThan1h;

      /// \brief View which containes the timeline.
      public: LogPlayView *view;
    };

    /// \class LogPlayViewPrivate LogPlayViewPrivate.hh
    /// \brief Private data for the LogPlayView class
    class LogPlayViewPrivate
    {
      /// \brief Item which indicates the current time.
      public: CurrentTimeItem *currentTimeItem;

      /// \brief Start time in milliseconds.
      public: common::Time startTime;

      /// \brief End time in milliseconds.
      public: common::Time endTime;

      /// \brief Log start time has been set or not.
      public: bool startTimeSet;

      /// \brief Log end time has been set or not.
      public: bool endTimeSet;

      /// \brief Width of this view's scene.
      public: int sceneWidth;

      /// \brief Height of this view's scene.
      public: int sceneHeight;

      /// \brief Margin from the ends.
      public: int margin;

      /// \brief Whether the timeline has already been drawn.
      public: bool timelineDrawn = false;
     };
  }
}
#endif
