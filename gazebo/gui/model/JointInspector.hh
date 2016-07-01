/*
 * Copyright (C) 2014-2016 Open Source Robotics Foundation
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

#ifndef _GAZEBO_GUI_JOINT_INSPECTOR_HH_
#define _GAZEBO_GUI_JOINT_INSPECTOR_HH_

#include <memory>
#include <string>

#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

#include "gazebo/gui/qt.h"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace gui
  {
    class JointMaker;

    // Forward declare private data.
    class JointInspectorPrivate;

    /// \class JointInspector gui/JointInspector.hh
    /// \brief A class to inspect and modify joints.
    class GZ_GUI_VISIBLE JointInspector : public QDialog
    {
      Q_OBJECT

      /// \brief Constructor
      /// \param[in] _jointMaker Pointer to joint maker.
      /// \param[in] _parent Parent QWidget.
      public: JointInspector(JointMaker *_jointMaker, QWidget *_parent = 0);

      /// \brief Destructor
      public: ~JointInspector();

      /// \brief Update the joint config widget with a joint msg.
      /// \param[in] _jointMsg Joint message.
      public: void Update(ConstJointPtr _jointMsg);

      /// \brief Get the msg containing all joint data.
      /// \return Joint msg.
      public: msgs::Joint *Data() const;

      /// \brief Set the pose of the joint.
      /// \param[in] _pose Pose to set the joint to.
      public: void SetPose(const ignition::math::Pose3d &_pose);

      /// \brief Open the inspector.
      public: void Open();

      /// \brief Set the unique id for the joint this inspector is attached to.
      /// The ID might be generated after the inspector.
      /// \param[in] _id Unique id.
      public: void SetJointId(const std::string &_id);

      /// \brief Qt event emiited when the mouse enters this widget.
      /// \param[in] _event Qt event.
      protected: virtual void enterEvent(QEvent *_event);

      /// \brief Qt callback when an enum value has changed.
      /// \param[in] _name of widget in the config widget that emitted the
      /// signal.
      /// \param[in] _value New value in string.
      private slots: void OnEnumChanged(const QString &_name,
          const QString &_value);

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
      private slots: void OnVector3dChanged(const QString &_name,
          const ignition::math::Vector3d &_value);

      /// \brief Qt callback when a string value has changed.
      /// \param[in] _name of widget in the config widget that emitted the
      /// signal.
      /// \param[in] _value New value.
      private slots: void OnStringChanged(const QString &_name,
          const std::string &_value);

      /// \brief Callback when the joint type has changed.
      /// \param[in] _value New joint type.
      private: void OnJointTypeChanged(const QString &_value);

      /// \brief Callback when the joint parent or child link has changed.
      /// \param[in] _linkName New link's name.
      private: void OnLinksChanged(const QString &_linkName = "");

      /// \brief Callback when the swap button is pressed.
      private slots: void OnSwap();

      /// \brief Add a link to the parent and child lists.
      /// \param[in] _linkName Scoped link name.
      private slots: void OnLinkInserted(const std::string &_linkName);

      /// \brief Remove a link from the parent and child lists.
      /// \param[in] _linkName Link name.
      private slots: void OnLinkRemoved(const std::string &_linkName);

      /// \brief Qt signal emitted to indicate that changes should be applied.
      Q_SIGNALS: void Applied();

      /// \brief Qt callback when the Remove button is pressed.
      private slots: void OnRemove();

      /// \brief Qt callback when the Cancel button is pressed.
      private slots: void OnCancel();

      /// \brief Qt callback when the Ok button is pressed.
      private slots: void OnOK();

      /// \brief Restore the widget's data to how it was when first opened.
      private slots: void RestoreOriginalData();

      /// \brief Qt key press event.
      /// \param[in] _event Qt key event.
      private: void keyPressEvent(QKeyEvent *_event);

      /// \brief Enable ok button if all values in the dialog are valid.
      /// \return True if all values are valid.
      private: bool CheckValid();

      /// \brief Qt close event
      /// \param[in] _event Qt close event pointer
      private: void closeEvent(QCloseEvent *_event);

      /// \internal
      /// \brief Pointer to private data.
      private: std::unique_ptr<JointInspectorPrivate> dataPtr;
    };
    /// \}
  }
}

#endif
