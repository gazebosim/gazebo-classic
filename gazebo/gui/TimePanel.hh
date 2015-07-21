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
#ifndef _TIME_PANEL_HH_
#define _TIME_PANEL_HH_

#include <vector>
#include <list>
#include <string>

#include "gazebo/gui/qt.h"
#include "gazebo/gui/TimeWidget.hh"
#include "gazebo/gui/LogPlayWidget.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/msgs/MessageTypes.hh"
#include "gazebo/common/Event.hh"
#include "gazebo/common/Time.hh"
#include "gazebo/util/system.hh"

class QLineEdit;
class QLabel;

namespace gazebo
{
  namespace gui
  {
    class TimePanelPrivate;
    class TimeWidget;
    class LogPlayWidget;

    class GAZEBO_VISIBLE TimePanel : public QWidget
    {
      Q_OBJECT

      /// \brief Constructor
      /// \param[in] _parent Parent widget.
      public: TimePanel(QWidget *_parent = 0);

      /// \brief Destructor
      public: virtual ~TimePanel();

      /// \brief Show real time factor.
      /// \param[in] _show True to display real time factor.
      public: void ShowRealTimeFactor(bool _show);

      /// \brief Show real time
      /// \param[in] _show True to display real time.
      public: void ShowRealTime(bool _show);

      /// \brief Show sim time
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

      /// \brief Qt call back when the step value in the spinbox changed
      /// \param[in] _value New step value.
      public slots: void OnStepValueChanged(int _value);

      /// \brief QT callback when the reset time button is pressed.
      public slots: void OnTimeReset();

      /// \brief QT signal to set visibility of time widget.
      /// \param[in] _visible True to make visible.
      signals: void SetTimeWidgetVisible(bool _visible);

      /// \brief QT signal to set visibility of log play widget.
      /// \param[in] _visible True to make visible.
      signals: void SetLogPlayWidgetVisible(bool _visible);

      /// \brief Update the data output.
      private slots: void Update();

      /// \brief Qt call back when the play action state changes
      private slots: void OnPlayActionChanged();

      /// \brief Called when the GUI enters/leaves full-screen mode.
      /// \param[in] _value True when entering full screen, false when
      /// leaving.
      private: void OnFullScreen(bool _value);

      /// \brief Called when a world stats message is received.
      /// \param[in] _msg World statistics message.
      private: void OnStats(ConstWorldStatisticsPtr &_msg);

      /// \internal
      /// \brief Pointer to private data.
      private: TimePanelPrivate *dataPtr;
    };
  }
}

#endif
