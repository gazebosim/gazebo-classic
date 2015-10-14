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

#ifndef _GAZEBO_JOINT_DATA_HH_
#define _GAZEBO_JOINT_DATA_HH_

#include <string>

#include "gazebo/common/CommonTypes.hh"

#include "gazebo/gui/qt.h"
#include "gazebo/gui/model/JointMaker.hh"

namespace gazebo
{
  namespace gui
  {
    class JointInspector;

    /// \addtogroup gazebo_gui
    /// \{

    /// \class JointData JointData.hh
    /// \brief Helper class to store joint data
    class GZ_GUI_VISIBLE JointData : public QObject
    {
      Q_OBJECT

      /// \brief Constructor;
      public: JointData() = default;

      /// \brief Destructor;
      public: ~JointData() = default;

      /// \brief Open the joint inspector.
      public: void OpenInspector();

      /// \brief Update this joint data.
      public: void Update();

      /// \brief Update joint.
      public: void UpdateJointLine();

      /// \brief Set joint type
      /// \param[in] _type New joint type.
      public: void SetType(const JointMaker::JointType _type);

      /// \brief Set joint type
      /// \param[in] _type New joint type.
      public: void SetAxis(const QString &_axis,
          const ignition::math::Vector3d &_value);

      /// \brief Set joint parent visual
      /// \param[in] _vis New parent visual.
      public: void SetParent(const rendering::VisualPtr &_vis);

      /// \brief Set joint child visual
      /// \param[in] _vis New child visual.
      public: void SetChild(const rendering::VisualPtr &_vis);

      /// \brief Get joint type
      /// \param[in] _type New joint type.
      public: JointMaker::JointType Type() const;

      /// \brief Get joint parent visual
      /// \param[in] _vis Pointer to parent visual.
      public: rendering::VisualPtr Parent() const;

      /// \brief Get joint child visual
      /// \param[in] _vis Pointer to child visual.
      public: rendering::VisualPtr Child() const;

      /// \brief Name of the joint.
      public: std::string name;

      /// \brief Visual of the dynamic line
      public: rendering::VisualPtr visual;

      /// \brief Joint visual.
      public: rendering::JointVisualPtr jointVisual;

      /// \brieft Visual of the hotspot
      public: rendering::VisualPtr hotspot;

      /// \internal
      /// \brief Parent visual pose used to determine if updates are needed.
      public: math::Pose parentPose;

      /// \internal
      /// \brief Child visual pose used to determine if updates are needed.
      public: math::Pose childPose;

      /// \internal
      /// \brief Child visual scale used to determine if updates are needed.
      public: math::Vector3 childScale;

      /// \brief Visual line used to represent joint connecting parent and child
      public: rendering::DynamicLines *line;

      /// \brief Visual handle used to represent joint parent
      public: Ogre::BillboardSet *handles;

      /// \brief True if the joint visual needs update.
      public: bool dirty;

      /// \brief Msg containing joint data.
      public: msgs::JointPtr jointMsg;

      /// \brief Inspector for configuring joint properties.
      public: JointInspector *inspector;

      /// \brief Qt Callback when joint inspector is to be opened.
      private slots: void OnOpenInspector();

      /// \brief Qt Callback when joint inspector configurations are to be
      /// applied.
      private slots: void OnApply();

      /// \brief Type of joint.
      private: JointMaker::JointType type;

      /// \brief Parent visual the joint is connected to.
      private: rendering::VisualPtr parent;

      /// \brief Child visual the joint is connected to.
      private: rendering::VisualPtr child;
    };
    /// \}
  }
}
#endif
