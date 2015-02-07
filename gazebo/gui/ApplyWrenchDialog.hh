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

#ifndef _APPLY_WRENCH_DIALOG_HH_
#define _APPLY_WRENCH_DIALOG_HH_

#include "gazebo/gui/qt.h"
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

      /// \brief Set link to which wrench will be applied.
      /// \param[in] _linkName Link name.
      public: void SetLink(std::string _linkName);

      /// \brief TODO
      public: void SetMode(rendering::ApplyWrenchVisual::WrenchModes _mode);

      /// \brief Qt callback when the Apply button is pressed.
      private slots: void OnApply();

      /// \brief Qt callback when the Cancel button is pressed.
      private slots: void OnCancel();

      /// \brief Qt callback to show/hide point options.
      /// \param[in] _checked Whether it is checked or not.
      private slots: void TogglePoint(bool _checked);

      /// \brief Qt callback to show/hide force options.
      /// \param[in] _checked Whether it is checked or not.
      private slots: void ToggleForce(bool _checked);

      /// \brief Qt callback to show/hide torque options.
      /// \param[in] _checked Whether it is checked or not.
      private slots: void ToggleTorque(bool _checked);

      /// \brief Qt callback when the the point X has changed.
      /// \param[in] _magnitude Point vector X component
      private slots: void OnPointXChanged(double _pX);

      /// \brief Qt callback when the the point Y has changed.
      /// \param[in] _magnitude Point vector Y component
      private slots: void OnPointYChanged(double _pY);

      /// \brief Qt callback when the the point Z has changed.
      /// \param[in] _magnitude Point vector Z component
      private slots: void OnPointZChanged(double _pZ);

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

      /// \brief TODO
      private slots: bool eventFilter(QObject *_object, QEvent *_event);

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

      /// \brief TODO
      private: void SetPublisher();

      /// \brief TODO
      private: void CalculatePoint();

      /// \brief TODO
      private: void CalculateForce();

      /// \brief TODO
      private: void CalculateTorque();

      /// \brief TODO
      private: void UpdatePointVisual();

      /// \brief TODO
      private: void UpdateForceVisual();

      /// \brief TODO
      private: void UpdateTorqueVisual();

      /// \brief TODO
      private: void UpdateForceMag();

      /// \brief TODO
      private: void UpdateForceVector();

      /// \brief TODO
      private: void UpdateForceVectorSpins(math::Vector3 _v);

      /// \brief TODO
      private: void UpdateTorqueMag();

      /// \brief TODO
      private: void UpdateTorqueVector();

      /// \brief TODO
      private: void UpdateTorqueVectorSpins(math::Vector3 _v);

      /// \brief TODO
      private: void AttachVisuals();

      /// \brief TODO
      private: void SetSpinValue(QDoubleSpinBox *_spin, double _value);

      /// \internal
      /// \brief Pointer to private data.
      private: ApplyWrenchDialogPrivate *dataPtr;
    };
    /// \}
  }
}

#endif
