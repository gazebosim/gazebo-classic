/*
 * Copyright 2012 Open Source Robotics Foundation
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
#ifndef _VIDEORECORDER_HH_
#define _VIDEORECORDER_HH_

#include <string>
#include "gui/qt.h"
#include "common/Event.hh"

namespace gazebo
{
  namespace gui
  {
    class RenderWidget;

    class VideoRecorder : public QWidget
    {
      Q_OBJECT

      /// \brief Constructor
      /// \param[in] _parent Parent Widget
      public: VideoRecorder(QWidget *_parent = 0);

      /// \brief Destructor
      public: ~VideoRecorder();

      /// \brief Create Qt actions
      public: void CreateActions();

      /// \brief Qt signal emitted when a message needs to be displayed.
      /// \param[in] _msg Message to be displayed
      /// \param[in] _duration Duration to display the message for.
      Q_SIGNALS: void MessageChanged(std::string _msg, int _duration);

      /// \brief Qt callback when the record video action is triggered.
      private slots: void RecordVideo();

      /// \brief Qt callback when a video format menu action is triggered.
      private slots: void SetRecordVideoFormat(QAction *_action);

      /// \brief Qt callback when the record video format button is clicked.
      private slots: void ShowVideoFormatMenu();

      /// \brief Qt callback connected to a QTimer for displaying a blinking
      /// recording text to the screen.
      private slots: void DisplayRecordingMsg();

      /// \brief Timer for displaying the video recording message.
      private: QTimer *recordVideoTimer;
    };
  }
}

#endif
