/*
 * Copyright (C) 2015-2016 Open Source Robotics Foundation
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
#ifndef _GAZEBO_GUI_JOINTCONTROLWIDGET_PRIVATE_HH_
#define _GAZEBO_GUI_JOINTCONTROLWIDGET_PRIVATE_HH_

#include <map>
#include <string>

#include "gazebo/gui/qt.h"
#include "gazebo/transport/TransportTypes.hh"

namespace gazebo
{
  namespace gui
  {
    class JointForceControl;
    class JointPIDPosControl;
    class JointPIDVelControl;

    /// \brief Private data for the JointControlWidget class.
    class JointControlWidgetPrivate
    {
      /// \brief Node for coomunication.
      public: transport::NodePtr node;

      /// \brief Publisher for joint messages.
      public: transport::PublisherPtr jointPub;

      /// \brief Sliders for force control
      public: std::map<std::string, JointForceControl *> sliders;

      /// \brief Sliders for position control
      public: std::map<std::string, JointPIDPosControl *> pidPosSliders;

      /// \brief Sliders for velocity control
      public: std::map<std::string, JointPIDVelControl *> pidVelSliders;

      /// \brief Label for the name of the current model being controlled.
      public: QLabel *modelLabel;

      /// \brief Tab widget for all the types of join control.
      public: QTabWidget *tabWidget;

      /// \brief Layout for the force controls.
      public: QGridLayout *forceGridLayout;

      /// \brief Layout for the position controls.
      public: QGridLayout *positionGridLayout;

      /// \brief Layout for the velocity controls.
      public: QGridLayout *velocityGridLayout;
    };

    /// \brief Private data for the JointForceControl class.
    class JointForceControlPrivate
    {
      /// \brief Name of the joint.
      public: std::string name;

      /// \brief Joint force slider.
      public: QDoubleSpinBox *forceSpin;
    };

    /// \brief Private data for the JointPIDPosControl class.
    class JointPIDPosControlPrivate
    {
      /// \brief Slider for the position.
      public: QDoubleSpinBox *posSpin;

      /// \brief Sliders for the P gain.
      public: QDoubleSpinBox *pGainSpin;

      /// \brief Sliders for the I gain.
      public: QDoubleSpinBox *iGainSpin;

      /// \brief Sliders for the D gain.
      public: QDoubleSpinBox *dGainSpin;

      /// \brief Name of the joint.
      public: std::string name;

      /// \brief True if the units are radians.
      public: bool radians;
    };

    /// \brief Private data for the JointPIDVelControl class.
    class JointPIDVelControlPrivate
    {
      /// \brief Slider for the position.
      public: QDoubleSpinBox *posSpin;

      /// \brief Sliders for the P gain.
      public: QDoubleSpinBox *pGainSpin;

      /// \brief Sliders for the I gain.
      public: QDoubleSpinBox *iGainSpin;

      /// \brief Sliders for the D gain.
      public: QDoubleSpinBox *dGainSpin;

      /// \brief Name of the joint.
      public: std::string name;
    };
  }
}
#endif
