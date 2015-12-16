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
#ifndef _GAZEBO_DATALOGGER_PRIVATE_HH_
#define _GAZEBO_DATALOGGER_PRIVATE_HH_

#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/gui/qt.h"

namespace gazebo
{
  namespace gui
  {
    /// \brief Private data for the DataLogger class.
    class DataLoggerPrivate
    {
      /// \brief Node to handle communication.
      public: transport::NodePtr node;

      /// \brief Publisher for log control messages.
      public: transport::PublisherPtr pub;

      /// \brief Subscriber for log status messages.
      public: transport::SubscriberPtr sub;

      /// \brief The button used to start and pause logging.
      public: QToolButton *recordButton;

      /// \brief Label to display the log time.
      public: QLabel *timeLabel;

      /// \brief Log base path.
      public: QString basePath;

      /// \brief Text edit to display the log destination path.
      public: QPlainTextEdit *destPath;

      /// \brief Label to display the log destination uri.
      public: QLineEdit *destURI;

      /// \brief Label to display the log file size.
      public: QLabel *sizeLabel;

      /// \brief Label to display status information.
      public: QLabel *statusLabel;

      /// \brief Timer used to blink the status label.
      public: QTimer *statusTimer;

      /// \brief Keep track of the time the status label blinks.
      public: double statusTime;

      /// \brief Name of the log file path
      public: QLineEdit *filenameEdit;

      /// \brief Frame that holds settings.
      public: QFrame *settingsFrame;

      /// \brief List of recorded logs.
      public: QTextBrowser *logList;

      /// \brief Dialog that displays confirmation after saving.
      public: QDialog *confirmationDialog;

      /// \brief Timer used to timeout confirmation dialog.
      public: QTimer *confirmationTimer;
    };
  }
}
#endif

