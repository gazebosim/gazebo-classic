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

#ifndef _APPLY_FORCE_DIALOG_HH_
#define _APPLY_FORCE_DIALOG_HH_

#include "gazebo/gui/qt.h"
#include "gazebo/math/Vector3.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/rendering/RenderTypes.hh"

namespace gazebo
{
  namespace gui
  {
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

      /// \brief Set model to be followed.
      /// \param[in] Model name.
      public: void SetModel(std::string _modelName);

      /// \brief Qt callback when the Apply button is pressed.
      private slots: void OnApply();

      /// \brief Qt callback when the Cancel button is pressed.
      private slots: void OnCancel();

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
      private: void SetPublisher();

      /// \brief TODO
      private: void CalculateWrench();

      /// \brief TODO
      private: void UpdateForceMag();

      /// \brief TODO
      private: void UpdateForceVector();

      /// \brief TODO
      private: void UpdateTorqueMag();

      /// \brief TODO
      private: void UpdateTorqueVector();

      /// \brief Node for communication.
      private: transport::NodePtr node;

      /// TODO
      private: std::string modelName;

      /// TODO
      private: std::string topicName;

      /// TODO
      private: QLabel *messageLabel;

      /// TODO
      private: QDoubleSpinBox *forceMagSpin;

      /// TODO
      private: QDoubleSpinBox *forceXSpin;

      /// TODO
      private: QDoubleSpinBox *forceYSpin;

      /// TODO
      private: QDoubleSpinBox *forceZSpin;

      /// TODO
      private: QDoubleSpinBox *torqueMagSpin;

      /// TODO
      private: QDoubleSpinBox *torqueXSpin;

      /// TODO
      private: QDoubleSpinBox *torqueYSpin;

      /// TODO
      private: QDoubleSpinBox *torqueZSpin;

      /// TODO
      private: math::Vector3 forceVector;

      /// TODO
      private: math::Vector3 torqueVector;

      /// \brief Publishes the wrench message.
      private: transport::PublisherPtr wrenchPub;
    };
    /// \}
  }
}

#endif
