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
#ifndef _LOGPLAYWIDGET_HH_
#define _LOGPLAYWIDGET_HH_

#include "gazebo/gui/qt.h"

namespace gazebo
{
  namespace gui
  {
    /// \brief A widget to control playback of log files.
    class LogPlayWidget : public QWidget
    {
      Q_OBJECT

      /// \brief Constructor.
      public: LogPlayWidget(QWidget *_parent = NULL);

      /// \brief Destructor.
      public: virtual ~LogPlayWidget();

      /// \brief Callback when the file button is pressed.
      private slots: void OnFileButton();
    };
  }
}
#endif
