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

#ifndef _LINK_CONFIG_HH_
#define _LINK_CONFIG_HH_

#include <string>

#include "gazebo/math/Pose.hh"
#include "gazebo/msgs/msgs.hh"

#include "gazebo/gui/qt.h"

namespace gazebo
{
  namespace gui
  {
    class ConfigWidget;

    /// \addtogroup gazebo_gui
    /// \{

    /// \class LinkConfig LinkConfig.hh
    /// \brief A tab for configuring properties of a link.
    class LinkConfig : public QWidget
    {
      Q_OBJECT

      /// \brief Constructor
      public: LinkConfig();

      /// \brief Destructor
      public: ~LinkConfig();

      /// \brief Update the link config widget with a link msg.
      /// \param[in] _linkMsg Link message.
      public: void Update(ConstLinkPtr _linkMsg);

      /// \brief Get the msg containing all link data.
      /// \return Link msg.
      public: msgs::Link *GetData() const;

      /// \brief Set the pose of the link.
      /// \param[in] _pose Pose to set the link to.
      public: void SetPose(const ignition::math::Pose3d &_pose);

      /// \brief Set the mass of the link.
      /// \param[in] _mass Mass to set the link to.
      public: void SetMass(double _mass);

      /// \brief Set the inertia matrix of the link.
      /// \param[in] _ixx X second moment of inertia (MOI) about x axis.
      /// \param[in] _iyy Y second moment of inertia about y axis.
      /// \param[in] _izz Z second moment of inertia about z axis.
      /// \param[in] _ixy XY inertia.
      /// \param[in] _ixz XZ inertia.
      /// \param[in] _iyz YZ inertia.
      public: void SetInertiaMatrix(double _ixx, double _iyy, double _izz,
          double _ixy, double _ixz, double _iyz);

      /// \brief Set the mass of the link.
      /// \param[in] _pose Inertial pose to set the link to.
      public: void SetInertialPose(const ignition::math::Pose3d &_pose);

      /// \brief config widget for configuring link properties.
      private: ConfigWidget *configWidget;
    };
    /// \}
  }
}
#endif
