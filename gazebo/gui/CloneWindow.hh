/*
 * Copyright (C) 2014-2016 Open Source Robotics Foundation
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
#ifndef GAZEBO_GUI_CLONEWINDOW_HH_
#define GAZEBO_GUI_CLONEWINDOW_HH_

#include <memory>

#include "gazebo/gui/qt.h"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace gui
  {
    class CloneWindowPrivate;

    /// \addtogroup gazebo_gui GUI
    /// \brief The graphical user interface
    /// \{

    /// \class CloneWindow CloneWindow.hh gui/CloneWindow.hh
    /// \brief A dialog window used to set some parameters before cloning a
    /// simulation.
    class GZ_GUI_VISIBLE CloneWindow : public QDialog
    {
      Q_OBJECT

      /// \brief Constructor.
      /// \param[in] _parent Parent widget pointer.
      public: CloneWindow(QWidget *_parent = 0);

      /// \brief Destructor
      public: virtual ~CloneWindow();

      /// \brief Get the port for the new gzserver specified by the user.
      /// \return The port for the new gzserver.
      public: int Port() const;

      /// \brief True if the stored port is a valid one. A valid port is an
      /// integer number in the 1025-65535 range.
      /// \return True when the port is valid or false otherwise.
      public: bool IsValidPort() const;

      /// \brief Callback when okay button is selected.
      private slots: void OnOkay();

      /// \brief Callback when cancel button is selected.
      private slots: void OnCancel();

      /// \brief Update the member variables after the user pressed okay.
      private: void Update();

      /// \internal
      /// \brief Pointer to private data.
      private: std::unique_ptr<CloneWindowPrivate> dataPtr;
    };
  }
}

#endif
