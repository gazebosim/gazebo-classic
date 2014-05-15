/*
 * Copyright (C) 2014 Open Source Robotics Foundation
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
#ifndef _OCULUSCONFIG_HH_
#define _OCULUSCONFIG_HH_

#include <string>
#include "gazebo/math/Pose.hh"
#include "gazebo/math/Vector2d.hh"
#include "gazebo/gui/qt.h"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace gui
  {
    /// \addtogroup gazebo_gui
    /// \{

    /// \class OculusConfig OculusConfig.hh gui/OculusConfig.hh
    /// \brief A widget that configures Oculus Rift to be used within Gazebo.
    class GAZEBO_VISIBLE OculusConfig : public QDialog
    {
      Q_OBJECT

      /// \brief Constructor
      /// \param[in] _parent Parent widget pointer.
      public: OculusConfig(QWidget *_parent = 0);

      /// \brief Destructor
      public: virtual ~OculusConfig();

      /// \brief Get the coordinates of the Oculus Window.
      /// \return Vector2 with the (x,y) coordinates of the Oculus window.
      public: math::Vector2d GetOculuswindowCoordinates();

      /// \brief Get the visual link attached to Oculus Rift.
      /// \return The name of the visual link.
      public: std::string GetVisual();

      /// \brief It is possible to attach Oculus rift to a visual link with a
      /// specific offset. This method gets the selected offset.
      /// \return The offset that will be applied to the Oculus.
      public: math::Pose GetOffset();

      /// \brief Update the member variables after the user pressed okay.
      private: void Update();

      /// \brief Callback when okay button is selected.
      private slots: void OnOkay();

      /// \brief Callback when cancel button is selected.
      private slots: void OnCancel();

      /// \brief Button used to finalize topic selection
      private: QPushButton *okayButton;

      /// \brief (x,y) Oculus window coordinates.
      private: math::Vector2d oculusWindowCoordinates;

      /// \brief Name of the visual string to attach Oculus.
      private: std::string visual;

      /// \brief Offset to be applied to Oculus.
      private: math::Pose offset;

      /// \brief QT widget for reading the Oculus window X coordinate.
      private: QLineEdit *xEdit;

      /// \brief QT widget for reading the Oculus window Y coordinate.
      private: QLineEdit *yEdit;

      /// \brief QT widget for reading the name of the visual link to attach
      /// Oculus Rift.
      private: QLineEdit *visualEdit;

      /// \brief QT widget for reading the offset (x).
      private: QLineEdit *xOffsetEdit;

      /// \brief QT widget for reading the offset (y).
      private: QLineEdit *yOffsetEdit;

      /// \brief QT widget for reading the offset (z).
      private: QLineEdit *zOffsetEdit;

      /// \brief QT widget for reading the offset (roll).
      private: QLineEdit *rollOffsetEdit;

      /// \brief QT widget for reading the offset (pitch).
      private: QLineEdit *pitchOffsetEdit;

      /// \brief QT widget for reading the offset (yaw).
      private: QLineEdit *yawOffsetEdit;
    };
    /// \}
  }
}
#endif
