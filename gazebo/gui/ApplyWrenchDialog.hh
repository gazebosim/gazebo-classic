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

#include "gazebo/gui/qt.h"
#include "gazebo/common/KeyEvent.hh"
#include "gazebo/math/Vector2i.hh"
#include "gazebo/math/Vector3.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/rendering/ApplyWrenchVisual.hh"
#include "gazebo/rendering/RenderTypes.hh"

namespace gazebo
{
  namespace gui
  {
    class ApplyWrenchDialogPrivate;

    /// \addtogroup gazebo_gui
    /// \{

    /// \class ApplyWrenchDialog ApplyWrenchDialog.hh gui/gui.hh
    /// \brief Dialog for applying force and torque to a model.
    class GAZEBO_VISIBLE ApplyWrenchDialog : public QDialog
    {
      Q_OBJECT

      /// \brief Constructor.
      /// \param[in] _parent Parent QWidget.
      public: ApplyWrenchDialog(QWidget *_parent = 0);

      /// \brief Destructor.
      public: ~ApplyWrenchDialog();

      /// \brief TODO
      public: void Init(std::string _modelName, std::string _linkName);

      /// \brief TODO
      public: void Fini();

      /// \brief Set model to which wrench will be applied.
      /// \param[in] _modelName Model name.
      /// \return True if model was properly set.
      public: bool SetModel(std::string _modelName);

      /// \brief Set link to which wrench will be applied.
      /// \param[in] _linkName Link name.
      /// \return True if link was properly set.
      public: bool SetLink(std::string _linkName);

      /// \brief Set link from combo box.
      private slots: void SetLink(QString _linkName);

      /// \brief Qt callback when the Apply button is pressed.
      private slots: void OnApplyAll();

      /// \brief Qt callback when the Apply button is pressed.
      private slots: void OnApplyForce();

      /// \brief Qt callback when the Apply button is pressed.
      private slots: void OnApplyTorque();

      /// \brief Qt callback when the Cancel button is pressed.
      private slots: void OnCancel();

      /// \brief Qt callback to show/hide position options.
      /// \param[in] _checked Whether it is checked or not.
      private slots: void ToggleForcePos(bool _checked);

      /// \brief Qt callback to set position to CoM.
      /// \param[in] _checked Whether it is checked or not.
      private slots: void ToggleComRadio(bool _checked);

      /// \brief Qt callback to show/hide force options.
      /// \param[in] _checked Whether it is checked or not.
      private slots: void ToggleForce(bool _checked);

      /// \brief Qt callback to show/hide torque options.
      /// \param[in] _checked Whether it is checked or not.
      private slots: void ToggleTorque(bool _checked);

      /// \brief Qt callback when the the position X has changed.
      /// \param[in] _magnitude ForcePos vector X component
      private slots: void OnForcePosXChanged(double _pX);

      /// \brief Qt callback when the the position Y has changed.
      /// \param[in] _magnitude ForcePos vector Y component
      private slots: void OnForcePosYChanged(double _pY);

      /// \brief Qt callback when the the position Z has changed.
      /// \param[in] _magnitude ForcePos vector Z component
      private slots: void OnForcePosZChanged(double _pZ);

      /// \brief Qt callback when the the force magnitude has changed.
      /// \param[in] _magnitude Force magnitude
      private slots: void OnForceMagChanged(double _magnitude);

      /// \brief Qt callback when the the force X has changed.
      /// \param[in] _magnitude Force vector X component
      private slots: void OnForceXChanged(double _fX);

      /// \brief Qt callback when the the force Y has changed.
      /// \param[in] _magnitude Force vector Y component
      private slots: void OnForceYChanged(double _fY);

      /// \brief Qt callback when the the force Z has changed.
      /// \param[in] _magnitude Force vector Z component
      private slots: void OnForceZChanged(double _fZ);

      /// \brief Qt callback when the the force clear button is clicked.
      private slots: void OnForceClear();

      /// \brief Qt callback when the the torque magnitude has changed.
      /// \param[in] _magnitude Torque magnitude
      private slots: void OnTorqueMagChanged(double _magnitude);

      /// \brief Qt callback when the the torque X has changed.
      /// \param[in] _magnitude Torque vector X component
      private slots: void OnTorqueXChanged(double _fX);

      /// \brief Qt callback when the the torque Y has changed.
      /// \param[in] _magnitude Torque vector Y component
      private slots: void OnTorqueYChanged(double _fY);

      /// \brief Qt callback when the the torque Z has changed.
      /// \param[in] _magnitude Torque vector Z component
      private slots: void OnTorqueZChanged(double _fZ);

      /// \brief Qt callback when the the torque clear button is clicked.
      private slots: void OnTorqueClear();

      /// \brief Qt callback when TODO
      private slots: void OnManipulation();

      /// \brief TODO
      private slots: bool eventFilter(QObject *_object, QEvent *_event);

      /// \brief TODO
      private slots: void changeEvent(QEvent *_event);

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

      /// \brief Callback for a key press event.
      /// \param[in] _event The ley press event
      /// \return True if handled by this function.
      private: bool OnKeyPress(const common::KeyEvent &_event);

      /// \brief TODO
      private: void SetPublisher();

      /// \brief TODO
      private: void AttachVisuals();

      /// \brief TODO
      private: void SetSpinValue(QDoubleSpinBox *_spin, double _value);

      /// \brief TODO
      public: void SetWrenchMode(std::string _mode);

      /// \brief TODO
      private: void SetCoM(math::Vector3 _com);

      /// \brief TODO
      private: void SetForcePos(math::Vector3 _force);

      /// \brief TODO
      private: void NewForcePosVector();

      /// \brief TODO
      private: void SetForce(math::Vector3 _force, bool _rotationByMouse = false);

      /// \brief TODO
      private: void NewForceVector();

      /// \brief TODO
      private: void NewForceMag();

      /// \brief TODO
      private: void NewForceDirection(math::Vector3 _dir);

      /// \brief TODO
      private: void SetTorque(math::Vector3 _torque, bool _rotationByMouse = false);

      /// \brief TODO
      private: void NewTorqueVector();

      /// \brief TODO
      private: void NewTorqueMag();

      /// \brief TODO
      private: void NewTorqueDirection(math::Vector3 _dir);

      /// \brief TODO
      private: void OnResponse(ConstResponsePtr &_msg);

      /// \brief TODO
      private: void SetActive(bool _active);

      /// \brief TODO
      private: void OnPreRender();

      /// \internal
      /// \brief Pointer to private data.
      private: ApplyWrenchDialogPrivate *dataPtr;
    };
    /// \}
  }
}

#endif
