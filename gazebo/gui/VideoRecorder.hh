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

#include "gazebo/rendering/UserCamera.hh"

#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/gui/qt.h"

namespace gazebo
{
  namespace gui
  {
    /// \addtogroup gazebo_gui
    /// \{

    /// \class VideoRecorder VideoRecorder.hh gui/VideoRecorder.hh
    /// \brief A widget that provides data logging functionality.
    class VideoRecorder : public QDialog
    {
      Q_OBJECT

      /// \brief Constructor.
      /// \param[in] _parent Parent widget pointer.
      public: VideoRecorder(QWidget *_parent = 0);

      /// \brief Destructor.
      public: virtual ~VideoRecorder();

      /// \brief QT callback for the record button.
      private slots: void OnRecord(bool _toggled);

      /// \brief QT callback for the preview button.
      private slots: void OnPreview();

      /// \brief QT callback for the set pose button.
      private slots: void OnSetPose();

      /// \brief QT callback for the grab pose button.
      private slots: void OnGrabPose();

      /// \brief QT callback when a row is selected in the pose list.
      private slots: void OnPoseSelect(int _row);

      /// \brief Node to handle communication.
      private: transport::NodePtr node;

      /// \brief Publisher for log control messages.
      private: transport::PublisherPtr pub;

      private: rendering::UserCameraPtr camera;

      private: QListWidget *poseList;
      private: QLineEdit *xEdit;
      private: QLineEdit *yEdit;
      private: QLineEdit *zEdit;
      private: QLineEdit *rollEdit;
      private: QLineEdit *pitchEdit;
      private: QLineEdit *yawEdit;
      private: QLineEdit *durationEdit;

      private: int activeRow;
    };
    /// \}
  }
}
#endif
