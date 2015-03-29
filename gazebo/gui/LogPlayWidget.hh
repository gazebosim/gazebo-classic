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

      /// \brief A signal used to set the sim time line edit.
      /// \param[in] _string String representation of sim time.
      signals: void SetSimTime(QString _string);

      /// \brief A signal used to set the sim time line edit.
      /// \param[in] _string String representation of sim time.
      public: void EmitSetSimTime(QString _string);

      /// \brief A signal used to set the iterations line edit.
      /// \param[in] _string String representation of iterations.
      signals: void SetIterations(QString _string);

      /// \brief A signal used to set the sim time line edit.
      /// \param[in] _string String representation of sim time.
      public: void EmitSetIterations(QString _string);

      /// \brief Qt call back when the step value in the spinbox changed
      public slots: void OnStepValueChanged(int _value);

      /// \internal
      /// \brief Pointer to private data.
      private: LogPlayWidgetPrivate *dataPtr;
    };
  }
}

#endif
