/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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
#ifndef GAZEBO_GUI_RENDEROPTIONSWINDOW_HH_
#define GAZEBO_GUI_RENDEROPTIONSWINDOW_HH_

#include <memory>

#include "gazebo/gui/qt.h"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace gui
  {
    class RenderOptionsWindowPrivate;

    /// \addtogroup gazebo_gui GUI
    /// \brief The graphical user interface
    /// \{

    /// \class RenderOptionsWindow RenderOptionsWindow.hh 
    ///     gui/RenderOptionsWindow.hh
    /// \brief A dialog window used to set some parameters before cloning a
    /// simulation.
    class GZ_GUI_VISIBLE RenderOptionsWindow : public QDialog
    {
      Q_OBJECT

      /// \brief Constructor.
      /// \param[in] _parent Parent widget pointer.
      public: explicit RenderOptionsWindow(QWidget *_parent = 0);

      /// \brief Destructor
      public: virtual ~RenderOptionsWindow();

      /// \brief Get the port for the new gzserver specified by the user.
      /// \return The port for the new gzserver.
      public: float RenderRate() const;

      /// \brief Callback when okay button is selected.
      private slots: void OnOkay();

      /// \brief Callback when cancel button is selected.
      private slots: void OnCancel();

      /// \brief Update the member variables after the user pressed okay.
      private: void Update();

      /// \internal
      /// \brief Pointer to private data.
      private: std::unique_ptr<RenderOptionsWindowPrivate> dataPtr;
    };
  }
}

#endif
