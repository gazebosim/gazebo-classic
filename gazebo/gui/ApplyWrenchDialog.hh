/*
 * Copyright 2015 Open Source Robotics Foundation
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

#ifndef _GAZEBO_APPLY_WRENCH_DIALOG_HH_
#define _GAZEBO_APPLY_WRENCH_DIALOG_HH_

#include <string>

#include "gazebo/common/MouseEvent.hh"
#include "gazebo/gui/qt.h"
#include "gazebo/math/Vector3.hh"
#include "gazebo/transport/TransportTypes.hh"

namespace gazebo
{
  namespace gui
  {
    class ApplyWrenchDialogPrivate;

    /// \addtogroup gazebo_gui
    /// \{

    /// \class ApplyWrenchDialog ApplyWrenchDialog.hh gui/gui.hh
    /// \brief Dialog for applying force and torque to a model.
    class GZ_GUI_VISIBLE ApplyWrenchDialog : public QDialog
    {
      Q_OBJECT

      /// \enum Modes
      /// \brief Dialog is in one of these modes at each time.
      public: enum Mode
      {
        /// \brief None
        NONE = 0,
        /// \brief Force
        FORCE = 1,
        /// \brief Torque
        TORQUE = 2
      };

      /// \brief Constructor.
      /// \param[in] _parent Parent QWidget.
      public: ApplyWrenchDialog(QWidget *_parent = 0);

      /// \brief Destructor.
      public: ~ApplyWrenchDialog();

      /// \brief Initiate the dialog.
      /// \param[in] _modelName Scoped name of the model.
      /// \param[in] _linkName Scoped name of a link within the model to which
      /// a wrench will be applied. The link might be changed later, but not
      /// the model.
      public: void Init(const std::string &_modelName,
          const std::string &_linkName);

      /// \brief Finish the dialog.
      public: void Fini();

      /// \brief Get the mode, either force, torque or none.
      /// \return Current mode.
      public: Mode GetMode() const;

      /// \brief Set model to which wrench will be applied.
      /// \param[in] _modelName Scoped model name.
      /// \return True if model was properly set.
      private: bool SetModel(const std::string &_modelName);

      /// \brief Set link to which wrench will be applied.
      /// \param[in] _linkName Scoped link name.
      /// \return True if link was properly set.
      private: bool SetLink(const std::string &_linkName);

      /// \brief Set link from combo box.
      /// \param[in] _linkName Link leaf name.
      private slots: void SetLink(const QString _linkName);

      /// \brief Qt callback when the Apply All button is pressed.
      private slots: void OnApplyAll();

      /// \brief Qt callback when the Apply Force button is pressed.
      private slots: void OnApplyForce();

      /// \brief Qt callback when the Apply Torque button is pressed.
      private slots: void OnApplyTorque();

      /// \brief Qt callback when the Cancel button is pressed.
      private slots: void OnCancel();

      /// \brief Qt callback to set position to CoM.
      /// \param[in] _checked Whether it is checked or not.
      private slots: void ToggleComRadio(bool _checked);

      /// \brief Qt callback when some force position component was changed.
      /// \param[in] _value New value, not used.
      private slots: void OnForcePosChanged(double _value);

      /// \brief Qt callback when the force magnitude was changed.
      /// \param[in] _magnitude Force magnitude.
      private slots: void OnForceMagChanged(double _magnitude);

      /// \brief Qt callback when some force component was changed.
      /// \param[in] _value New value, not used.
      private slots: void OnForceChanged(double _value);

      /// \brief Qt callback when the the force clear button is clicked.
      private slots: void OnForceClear();

      /// \brief Qt callback when the torque magnitude was changed.
      /// \param[in] _magnitude Torque magnitude.
      private slots: void OnTorqueMagChanged(double _magnitude);

      /// \brief Qt callback when some torque component was changed.
      /// \param[in] _value New value, not used.
      private slots: void OnTorqueChanged(double _value);

      /// \brief Qt callback when the the torque clear button is clicked.
      private slots: void OnTorqueClear();

      /// \brief Qt callback when entering a manipulation mode.
      private slots: void OnManipulation();

      /// \brief Filter events from other Qt objects.
      /// param[in] _object Qt object watched by the event filter
      /// param[in] _event Qt event to be filtered.
      /// \return True to stop event propagation.
      private slots: bool eventFilter(QObject *_object, QEvent *_event);

      /// \brief Handle change events related to this dialog.
      /// param[in] _event Qt event.
      private slots: void changeEvent(QEvent *_event);

      /// \brief Set the value of a specific spin without triggering signals to
      /// avoid recursion loops.
      /// \param[in] _spin Spin whose value will be changed.
      /// \param[in] _value New value.
      private: void SetSpinValue(QDoubleSpinBox *_spin, double _value);

      /// \brief Set force position vector, send it to visuals and update spins.
      /// \param[in] _forcePos New force position.
      private: void SetForcePos(const math::Vector3 &_forcePos);

      /// \brief Set force vector, send it to visuals and update spins.
      /// \param[in] _force New force.
      /// \param[in] _rotatedByMouse Rot tool has been rotated by the mouse.
      private: void SetForce(const math::Vector3 &_force,
          const bool _rotatedByMouse = false);

      /// \brief Set torque vector, send it to visuals and update spins.
      /// \param[in] _torque New torque.
      /// \param[in] _rotatedByMouse Rot tool has been rotated by the mouse.
      private: void SetTorque(const math::Vector3 &_torque,
          const bool _rotatedByMouse = false);

      /// \brief Callback on prerender to check if target link hasn't been
      /// deleted.
      private: void OnPreRender();

      /// \brief Attach apply wrench visual to target link.
      private: void AttachVisuals();

      /// \brief Set CoM vector and send it to visuals.
      /// \param[in] _com CoM position in link frame.
      private: void SetCoM(const math::Vector3 &_com);

      /// \brief Callback for a mouse press event.
      /// \param[in] _event The mouse press event
      /// \return True if handled by this function.
      private: bool OnMousePress(const common::MouseEvent &_event);

      /// \brief Callback for a mouse release event.
      /// \param[in] _event The mouse release event
      /// \return True if handled by this function.
      private: bool OnMouseRelease(const common::MouseEvent &_event);

      /// \brief Callback for a mouse move event.
      /// \param[in] _event The mouse move event
      /// \return True if handled by this function.
      private: bool OnMouseMove(const common::MouseEvent &_event);

      /// \brief Set the mode to either force, torque or none.
      /// \param[in] _mode Current mode.
      private: void SetMode(Mode _mode);

      /// \brief Update force vector with direction given by mouse, magnitude
      /// from spin.
      /// \param[in] _dir New direction, doesn't need to be normalized.
      private: void NewForceDirection(const math::Vector3 &_dir);

      /// \brief Update torque vector with direction given by mouse, magnitude
      /// from spin.
      /// \param[in] _dir New direction, doesn't need to be normalized.
      private: void NewTorqueDirection(const math::Vector3 &_dir);

      /// \brief Set this dialog to be active, visuals visible and mouse
      /// filters on.
      /// \param[in] _active True to make it active.
      private: void SetActive(bool _active);

      /// \brief Set this dialog window to be active.
      private: void ActivateWindow();

      /// \internal
      /// \brief Pointer to private data.
      private: ApplyWrenchDialogPrivate *dataPtr;
    };
    /// \}
  }
}

#endif
