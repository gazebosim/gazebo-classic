/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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

#ifndef GAZEBO_GUI_MODEL_GUIINSPECTOR_HH_
#define GAZEBO_GUI_MODEL_GUIINSPECTOR_HH_

#include <memory>

#include "gazebo/gui/qt.h"

#include "gazebo/msgs/msgs.hh"

#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace gui
  {
    // Forward declare private data.
    class GUIInspectorPrivate;

    /// \class GUIInspector gui/GUIInspector.hh
    /// \brief A class to inspect and modify joints.
    class GZ_GUI_VISIBLE GUIInspector : public QDialog
    {
      Q_OBJECT

      /// \brief Constructor
      /// \param[in] _jointMaker Pointer to joint maker.
      /// \param[in] _parent Parent QWidget.
      public: GUIInspector(QWidget *_parent = 0);

      /// \brief Destructor
      public: ~GUIInspector();

      /// \brief Update the joint config widget with a joint msg.
      /// \param[in] _jointMsg GUI message.
      public: void Update(ConstGUIPtr _jointMsg);

      /// \brief Open the inspector.
      public: void Open();

      /// \brief Qt event emiited when the mouse enters this widget.
      /// \param[in] _event Qt event.
      protected: virtual void enterEvent(QEvent *_event);

      /// \brief Qt callback when a pose value has changed.
      /// \param[in] _name of widget in the config widget that emitted the
      /// signal.
      /// \param[in] _value New value.
      private slots: void OnPoseChanged(const QString &_name,
          const ignition::math::Pose3d &_value);

      /// \brief Qt callback when a Vector3d value has changed.
      /// \param[in] _name of widget in the config widget that emitted the
      /// signal.
      /// \param[in] _value New value.
      private slots: void OnUIntChanged(const QString &_name,
          const unsigned int _value);

      /// \brief Qt callback when a Vector3d value has changed.
      /// \param[in] _name of widget in the config widget that emitted the
      /// signal.
      /// \param[in] _value New value.
      private slots: void OnDoubleChanged(const QString &_name,
          const double _value);

      /// \brief Qt callback when a Vector3d value has changed.
      /// \param[in] _name of widget in the config widget that emitted the
      /// signal.
      /// \param[in] _value New value.
      private slots: void OnColorChanged(const QString &_name,
          const common::Color &_value);

      /// \brief Qt callback when the Cancel button is pressed.
      private slots: void OnCancel();

      /// \brief Qt callback when the Ok button is pressed.
      private slots: void OnOK();

      /// \brief Restore the widget's data to how it was when first opened.
      private slots: void RestoreOriginalData();

      /// \brief Qt key press event.
      /// \param[in] _event Qt key event.
      private: void keyPressEvent(QKeyEvent *_event);

      /// \brief Qt close event
      /// \param[in] _event Qt close event pointer
      private: void closeEvent(QCloseEvent *_event);

      /// \internal
      /// \brief Pointer to private data.
      private: std::unique_ptr<GUIInspectorPrivate> dataPtr;
    };
    /// \}
  }
}

#endif
