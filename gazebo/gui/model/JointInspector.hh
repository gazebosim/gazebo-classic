/*
 * Copyright (C) 2014-2015 Open Source Robotics Foundation
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
    class ConfigWidget;

    /// \class JointInspector gui/JointInspector.hh
    /// \brief A class to inspect and modify joints.
    class GAZEBO_VISIBLE JointInspector : public QDialog
    {
      Q_OBJECT

      /// \brief Constructor
      /// \param[in] _parent Parent QWidget.
      public: JointInspector(QWidget *_parent = 0);

      /// \brief Destructor
      public: ~JointInspector();

      /// \brief Update the joint config widget with a joint msg.
      /// \param[in] _jointMsg Joint message.
      public: void Update(ConstJointPtr _jointMsg);

      /// \brief Get the msg containing all joint data.
      /// \return Joint msg.
      public: msgs::Joint *GetData() const;

      /// \brief Set the pose of the joint.
      /// \param[in] _pose Pose to set the joint to.
      public: void SetPose(const math::Pose &_pose);

      /// \brief Qt event emiited when the mouse enters this widget.
      /// \param[in] _event Qt event.
      protected: virtual void enterEvent(QEvent *_event);

      /// \brief Qt callback when the joint type has changed.
      /// \param[in] _name of widget in the config widget that emitted the
      /// signal.
      /// \param[in] _value New joint type value in string.
      private slots: void OnJointTypeChanged(const QString &_name,
          const QString &_value);

      /// \brief Qt signal emitted to indicate that changes should be applied.
      Q_SIGNALS: void Applied();

      /// \brief Qt callback when the Cancel button is pressed.
      private slots: void OnCancel();

      /// \brief Qt callback when the Apply button is pressed.
      private slots: void OnApply();

      /// \brief Qt callback when the Ok button is pressed.
      private slots: void OnOK();

      /// \brief Config widget for configuring joint properties.
      private: ConfigWidget *configWidget;
    };
    /// \}
  }
}

#endif
