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
#ifndef _GAZEBO_LOG_PLAY_WIDGET_HH_
#define _GAZEBO_LOG_PLAY_WIDGET_HH_

#include "gazebo/gui/qt.h"
#include "gazebo/gui/TimePanel.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace gui
  {
    class LogPlayWidgetPrivate;
    class TimePanel;

    class GAZEBO_VISIBLE LogPlayWidget : public QWidget
    {
      Q_OBJECT

      /// \brief Constructor
      /// \param[in] _parent Parent widget, commonly a TimePanel.
      public: LogPlayWidget(QWidget *_parent = 0);

      /// \brief Destructor
      public: virtual ~LogPlayWidget();

      /// \brief Returns if the simulation is displayed as paused.
      /// \return True if paused, false otherwise.
      public: bool IsPaused() const;

      /// \brief Set whether to display the simulation as paused.
      /// \param[in] _p True to display the simulation as paused. False
      /// indicates the simulation is running
      public: void SetPaused(bool _paused);

      /// \brief Emit signal to set sim time line edit.
      /// \param[in] _string String representation of current time.
      public: void EmitSetCurrentTime(QString _string);

      /// \brief Play simulation.
      public slots: void OnPlay();

      /// \brief Pause simulation.
      public slots: void OnPause();

      /// \brief Play simulation.
      public slots: void OnStepForward();

      /// \brief Qt signal to show the play button.
      signals: void ShowPlay();

      /// \brief Qt signal to hide the play button.
      signals: void HidePlay();

      /// \brief Qt signal to show the pause button.
      signals: void ShowPause();

      /// \brief Qt signal to hide the pause button.
      signals: void HidePause();

      /// \brief Qt signal used to set the current time line edit.
      /// \param[in] _string String representation of current time.
      signals: void SetCurrentTime(const QString &);

      /// \internal
      /// \brief Pointer to private data.
      private: LogPlayWidgetPrivate *dataPtr;
    };
  }
}

#endif
