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

#include "gazebo/common/Time.hh"
#include "gazebo/gui/qt.h"
#include "gazebo/gui/TimePanel.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace gui
  {
    class LogPlayWidgetPrivate;
    class LogPlayViewPrivate;
    class TimePanel;

    /// \class LogPlayWidget LogPlayWidget.hh
    /// \brief Widget which displays log playback options.
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
      public: void SetPaused(const bool _paused);

      /// \brief Emit signal to set current time.
      /// \param[in] _time Current time.
      public: void EmitSetCurrentTime(const common::Time &_time);

      /// \brief Emit signal to set start time.
      /// \param[in] _time Start time.
      public: void EmitSetStartTime(const common::Time &_time);

      /// \brief Emit signal to set end time.
      /// \param[in] _time End time.
      public: void EmitSetEndTime(const common::Time &_time);

      /// \brief Play simulation.
      public slots: void OnPlay();

      /// \brief Pause simulation.
      public slots: void OnPause();

      /// \brief Step simulation forward.
      public slots: void OnStepForward();

      /// \brief Step simulation back.
      public slots: void OnStepBack();

      /// \brief Jump to the start of the log file.
      public slots: void OnRewind();

      /// \brief Jump to the end of the log file.
      public slots: void OnForward();

      /// \brief Jump to a given time in the log file
      /// \param[in] _time Desired time
      public slots: void OnSeek(const common::Time &_time);

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

      /// \brief Qt signal used to set the end time line edit.
      /// \param[in] _string String representation of current time.
      signals: void SetEndTime(const QString &);

      /// \brief Qt signal used to set the current time in the view.
      /// \param[in] _time Current time.
      signals: void SetCurrentTime(const common::Time &_time);

      /// \brief Qt signal used to set the start time in the view.
      /// \param[in] _time Start time.
      signals: void SetStartTime(const common::Time &_time);

      /// \brief Qt signal used to set the end time in the view.
      /// \param[in] _time End time.
      signals: void SetEndTime(const common::Time &_time);

      /// \brief Publish a multistep message.
      /// \param[in] _step Number of steps.
      private: void PublishMultistep(int _step);

      /// \brief Helper function to prepare each of the four small buttons.
      /// \param[in] _button Pointer to the button.
      /// \param[in] _icon Icon uri.
      private: void SetupSmallButton(QToolButton *_button, QString _icon);

      /// \internal
      /// \brief Pointer to private data.
      private: LogPlayWidgetPrivate *dataPtr;
    };

    /// \class LogPlayView LogPlayView.hh
    /// \brief View for the timeline.
    class GAZEBO_VISIBLE LogPlayView: public QGraphicsView
    {
      Q_OBJECT

      /// \brief Constructor;
      /// \param[in] _parent Parent widget.
      public: LogPlayView(LogPlayWidget *_parent = 0);

      /// \brief Set the position of the current time item.
      /// \param[in] _time Current time.
      public slots: void SetCurrentTime(const common::Time &_time);

      /// \brief Set the log start time.
      /// \param[in] _time Start time.
      public slots: void SetStartTime(const common::Time &_time);

      /// \brief Set the log end time.
      /// \param[in] _time End time.
      public slots: void SetEndTime(const common::Time &_time);

      /// \brief Draw the timeline.
      public slots: void DrawTimeline();

      /// \brief Qt signal used to seek.
      /// \param[in] _time Time to jump to.
      signals: void Seek(const common::Time &_time);

      // Documentation inherited
      protected: void mousePressEvent(QMouseEvent *_event);

      // Documentation inherited
      protected: void mouseReleaseEvent(QMouseEvent *_event);

      // Documentation inherited
      protected: void mouseMoveEvent(QMouseEvent *_event);

      /// \internal
      /// \brief Pointer to private data.
      private: LogPlayViewPrivate *dataPtr;
    };

    /// \class CurrentTimeItem CurrentTimeItem.hh
    /// \brief Item which represents the current time within the view.
    class GAZEBO_VISIBLE CurrentTimeItem: public QObject,
        public QGraphicsRectItem
    {
      Q_OBJECT

      /// \brief Constructor;
      public: CurrentTimeItem();

      // Documentation inherited
      private: virtual void paint(QPainter *_painter,
          const QStyleOptionGraphicsItem *_option, QWidget *_widget);
    };
  }
}

#endif
