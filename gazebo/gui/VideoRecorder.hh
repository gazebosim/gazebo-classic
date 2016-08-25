/*
 * Copyright 2016 Open Source Robotics Foundation
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
#ifndef GAZEBO_GUI_VIDEORECORDER_HH_
#define GAZEBO_GUI_VIDEORECORDER_HH_

#include <memory>
#include "gazebo/gui/qt.h"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace gui
  {
    // Forward declare private data class.
    class VideoRecorderPrivate;

    /// \brief Helper class for recording the user camera to a video
    /// file.
    class GZ_GUI_VISIBLE VideoRecorder : public QWidget
    {
      Q_OBJECT

      /// \brief Constructor
      /// \param[in] _parent Parent Widget
      public: VideoRecorder(QWidget *_parent = 0);

      /// \brief Destructor
      public: ~VideoRecorder();

      /// \brief Signal emitted when a recording is started.
      Q_SIGNALS: void RecordingStarted();

      /// \brief Signal emitted when a recording is stopped.
      Q_SIGNALS: void RecordingStopped();

      /// \brief Signal emitted when the recording state changes.
      /// \param[out] _recording True if recording is enabled, false
      /// otherwise.
      Q_SIGNALS: void RecordingChanged(bool _recording);

      /// \brief Qt callback when the record stop button is pressed.
      private slots: void OnRecordStop();

      /// \brief Qt callback when the record video action is triggered.
      /// \param[in] _format Format to record (mp4, ogv, avi).
      private slots: void OnRecordStart(const QString &_format);

      /// \internal
      /// \brief Private data pointer
      private: std::unique_ptr<VideoRecorderPrivate> dataPtr;
    };
  }
}

#endif
