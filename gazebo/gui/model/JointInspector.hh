/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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

#ifndef _JOINT_INSPECTOR_HH_
#define _JOINT_INSPECTOR_HH_

#include <string>
#include <vector>

#include "gazebo/gui/qt.h"
#include "gazebo/gui/model/JointMaker.hh"
#include "gazebo/util/system.hh"


namespace gazebo
{
  namespace gui
  {
    class JointMaker;

    /// \class JointInspector gui/JointInspector.hh
    /// \brief A class to inspect and modify joints.
    class GAZEBO_VISIBLE JointInspector : public QDialog
    {
      Q_OBJECT

      /// \brief Constructor
      /// \param[in] _mode Dialog mode
      /// \param[in] _parent Parent QWidget.
      public: JointInspector(JointMaker::JointType _jointType,
          QWidget *_parent = 0);

      /// \brief Destructor
      public: ~JointInspector();

      /// \brief Get joint pose.
      /// \return Joint pose.
      public: math::Pose GetPose() const;

      /// \brief Get axis.
      /// \param[in] _index Index of axis
      /// \return Axis direction.
      public: math::Vector3 GetAxis(unsigned int _index) const;

      /// \brief Get axis lower limit.
      /// \param[in] _index Index of axis
      /// \return Lower limit.
      public: double GetLowerLimit(unsigned int _index) const;

      /// \brief Get axis upper limit.
      /// \param[in] _index Index of axis
      /// \return Upper limit.
      public: double GetUpperLimit(unsigned int _index) const;

      /// \brief Get joint type.
      /// \return Joint type.
      public: JointMaker::JointType GetType() const;

      /// \brief Get joint name.
      /// \return Joint name.
      public: std::string GetName() const;

      /// \brief Set the joint name.
      /// \param[in] _name Name to set to.
      public: void SetName(const std::string &_name);

      /// \brief Set the joint parent name.
      /// \param[in] _name Name of joint parent.
      public: void SetParent(const std::string &_parent);

      /// \brief Set the joint child name.
      /// \param[in] _name Name of joint child.
      public: void SetChild(const std::string &_child);

      /// \brief Set joint pose.
      /// \param[in] _pose Joint pose.
      public: void SetPose(const math::Pose &_pose);

      /// \brief Set axis.
      /// \param[in] _index Index of axis.
      /// \param[in] _axis Axis direction.
      public: void SetAxis(unsigned int _index, const math::Vector3 &_axis);

      /// \brief Set axis lower limit.
      /// \param[in] _index Index of axis.
      /// \param[in] _lower Lower limit.
      public: void SetLowerLimit(unsigned int _index, double _lower);

      /// \brief Set axis upper limit.
      /// \param[in] _index Index of axis.
      /// \param[in] _upper Upper limit.
      public: void SetUpperLimit(unsigned int _index, double _upper);

      /// \brief Set joint type.
      /// \param[in] _type joint type.
      public: void SetType(JointMaker::JointType _type);

      /// \brief Qt signal emitted to indicate that changes should be applied.
      Q_SIGNALS: void Applied();

      /// \brief Qt callback when the joint type combox index has changed.
      private slots: void OnJointTypeChanged(int _index);

      /// \brief Qt callback when the Cancel button is pressed.
      private slots: void OnCancel();

      /// \brief Qt callback when the Apply button is pressed.
      private slots: void OnApply();

      /// \brief Qt callback when the Ok button is pressed.
      private slots: void OnOK();

      /// \brief Line edit that displays the name of the joint.
      private: QLineEdit* jointNameLineEdit;

      /// \brief Label that displays the parent name of the joint.
      private: QLabel* jointParentLabel;

      /// \brief Label that displays the child name of the joint.
      private: QLabel* jointChildLabel;

      /// \brief ComboBox that displays the type of the joint.
      private: QComboBox *jointTypeComboBox;

      /// \brief Spin box for configuring the X position of the joint.
      private: QDoubleSpinBox *poseXSpinBox;

      /// \brief Spin box for configuring the Y position of the joint.
      private: QDoubleSpinBox *poseYSpinBox;

      /// \brief Spin box for configuring the Z position of joint.
      private: QDoubleSpinBox *poseZSpinBox;

      /// \brief Spin box for configuring the roll of the joint.
      private: QDoubleSpinBox *poseRollSpinBox;

      /// \brief Spin box for configuring the pitch of the joint.
      private: QDoubleSpinBox *posePitchSpinBox;

      /// \brief Spin box for configuring the yaw of the joint.
      private: QDoubleSpinBox *poseYawSpinBox;

      /// \brief Spin box for configuring the X direction of the axis.
      private: std::vector<QDoubleSpinBox *> axisXSpinBoxes;

      /// \brief Spin box for configuring the Y direction of the axis.
      private: std::vector<QDoubleSpinBox *> axisYSpinBoxes;

      /// \brief Spin box for configuring the Z direction of the axis.
      private: std::vector<QDoubleSpinBox *> axisZSpinBoxes;

      /// \brief Spin box for configuring the lower limit of the axis.
      private: std::vector<QDoubleSpinBox *> lowerLimitSpinBoxes;

      /// \brief Spin box for configuring the upper limit of the axis.
      private: std::vector<QDoubleSpinBox *> upperLimitSpinBoxes;

      /// \brief Type of joint.
      private: JointMaker::JointType jointType;

      /// \brief A list of group boxes for configuring joint axis properties.
      private: std::vector<QGroupBox *> axisGroupBoxes;
    };
    /// \}
  }
}

#endif
