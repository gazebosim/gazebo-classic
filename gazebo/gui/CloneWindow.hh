/*
 * Copyright (C) 2014-2015 Open Source Robotics Foundation
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

#ifndef _GAZEBO_CLONE_WINDOW_HH_
#define _GAZEBO_CLONE_WINDOW_HH_

#include "gazebo/gui/qt.h"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace gui
  {
    /// \addtogroup gazebo_gui
    /// \{

    /// \class CloneConfig CloneConfig.hh gui/CloneConfig.hh
    /// \brief A dialog window used to set some parameters before cloning a
    /// simulation.
    class GAZEBO_VISIBLE CloneWindow : public QDialog
    {
      Q_OBJECT

      /// \brief Constructor.
      /// \param[in] _parent Parent widget pointer.
      public: CloneWindow(QWidget *_parent = 0);

      /// \brief Destructor
      public: virtual ~CloneWindow();

      /// \brief Get the port for the new gzserver specified by the user.
      /// \return The port for the new gzserver.
      public: int GetPort();

      /// \brief True if the stored port is a valid one. A valid port is an
      /// integer number in the 1025-65535 range.
      /// \return True when the port is valid or false otherwise.
      public: bool IsValidPort();

      /// \brief Callback when okay button is selected.
      private slots: void OnOkay();

      /// \brief Callback when cancel button is selected.
      private slots: void OnCancel();

      /// \brief Update the member variables after the user pressed okay.
      private: void Update();

      /// \brief Button used to finalize port selection.
      private: QPushButton *okayButton;

      /// \brief QT widget for reading the port used in the cloned server.
      private: QLineEdit *portEdit;

      /// \brief Port used for the cloned server.
      private: int port;

      /// \brief Used to flag if the text entered by the user is a valid port.
      private: bool validPort;
    };
  }
}

#endif
