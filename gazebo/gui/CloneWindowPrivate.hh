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
#ifndef _GAZEBO_CLONE_WINDOW_PRIVATE_HH_
#define _GAZEBO_CLONE_WINDOW_PRIVATE_HH_

#include "gazebo/gui/qt.h"

namespace gazebo
{
  namespace gui
  {
    /// \class CloneWindowPrivate CloneWindowPrivate.hh
    /// \brief Private data for the CloneWindowPrivate class.
    class CloneWindowPrivate
    {
      /// \brief Button used to finalize port selection.
      public: QPushButton *okayButton;

      /// \brief QT widget for reading the port used in the cloned server.
      public: QLineEdit *portEdit;

      /// \brief Port used for the cloned server.
      public: int port;

      /// \brief Used to flag if the text entered by the user is a valid port.
      public: bool validPort;
    };
  }
}
#endif
