/*
 * Copyright (C) 2015-2016 Open Source Robotics Foundation
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
#ifndef GAZEBO_GUI_TIMEWIDGET_HH_
#define GAZEBO_GUI_TIMEWIDGET_HH_

#include "gazebo/gui/qt.h"
#include "gazebo/gui/TimePanel.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace gui
  {
    class TimeWidgetPrivate;
    class TimePanel;

    class GZ_GUI_VISIBLE TimeWidget : public QWidget
    {
      Q_OBJECT

      /// \brief Constructor
      /// \param[in] _parent Parent widget, commonly a TimePanel.
      public: TimeWidget(QWidget *_parent = 0);

      /// \brief Destructor
      public: virtual ~TimeWidget();

      /// \brief Show real time factor.
      /// \param[in] _show True to display real time factor.
      public: void ShowRealTimeFactor(bool _show);

      /// \brief Show real time.
      /// \param[in] _show True to display real time.
      public: void ShowRealTime(bool _show);

      /// \brief Show sim time.
      /// \param[in] _show True to display sim time.
      public: void ShowSimTime(bool _show);

      /// \brief Show the iterations.
      /// \param[in] _show True to show the iterations widget.
      public: void ShowIterations(bool _show);

      /// \brief Show the step widget.
      /// \param[in] _show True to show the step widget.
      public: void ShowStepWidget(bool _show);

      /// \brief Show fps.
      /// \param[in] _show True to show the fps widget.
      public: void ShowFPS(bool _show);

      /// \brief Returns if the simulation is displayed as paused.
      /// \return True if paused, false otherwise.
      public: bool IsPaused() const;

      /// \brief Set whether to display the simulation as paused.
      /// \param[in] _paused True to display the simulation as paused. False
      /// indicates the simulation is running
      public: void SetPaused(bool _paused);

      /// \brief Emit a signal used to set the sim time line edit.
      /// \param[in] _string String representation of sim time.
      public: void EmitSetSimTime(QString _string);

      /// \brief Emit a signal used to set the real time line edit.
      /// \param[in] _string String representation of real time.
      public: void EmitSetRealTime(QString _string);

      /// \brief Emit a signal used to set the iterations line edit.
      /// \param[in] _string String representation of iterations.
      public: void EmitSetIterations(QString _string);

      /// \brief Emit a signal used to set the FPS line edit.
      /// \param[in] _string String representation of average FPS.
      public: void EmitSetFPS(QString _string);

      /// \brief A signal used to set the sim time line edit.
      /// \param[in] _string String representation of real time factor.
      public: void SetPercentRealTimeEdit(QString _text);

      /// \brief Qt call back when the step value in the spinbox changed
      /// \param[in] _value New step value.
      public slots: void OnStepValueChanged(int _value);

      /// \brief QT callback when the reset time button is pressed.
      public slots: void OnTimeReset();

      /// \brief A signal used to set the sim time line edit.
      /// \param[in] _string String representation of sim time.
      signals: void SetSimTime(QString _string);

      /// \brief A signal used to set the real time line edit.
      /// \param[in] _string String representation of real time.
      signals: void SetRealTime(QString _string);

      /// \brief A signal used to set the iterations line edit.
      /// \param[in] _string String representation of iterations.
      signals: void SetIterations(QString _string);

      /// \brief A signal used to set the avg fps line edit.
      /// \param[in] _string String representation of avg fps.
      signals: void SetFPS(QString _string);

      /// \internal
      /// \brief Pointer to private data.
      private: TimeWidgetPrivate *dataPtr;
    };
  }
}

#endif
