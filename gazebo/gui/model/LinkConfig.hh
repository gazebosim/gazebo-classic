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
#include "gazebo/gui/model/LinkInspector.hh"

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
    class GZ_GUI_VISIBLE LinkConfig : public QWidget
    {
      Q_OBJECT

      /// \brief Constructor
      public: LinkConfig();

      /// \brief Destructor
      public: virtual ~LinkConfig();

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
      public: void SetMass(const double _mass);

      /// \brief Retrieve current mass value.
      /// \return The current mass.
      public: double Mass() const;

      /// \brief Set the density of the link.
      /// \param[in] _density Density to set the link to.
      public: void SetDensity(const double _density);

      /// \brief Retrieve current density value.
      /// \return The current density.
      public: double Density() const;

      /// \brief Set the inertia matrix of the link.
      /// \param[in] _ixx X second moment of inertia (MOI) about x axis.
      /// \param[in] _iyy Y second moment of inertia about y axis.
      /// \param[in] _izz Z second moment of inertia about z axis.
      /// \param[in] _ixy XY inertia.
      /// \param[in] _ixz XZ inertia.
      /// \param[in] _iyz YZ inertia.
      public: void SetInertiaMatrix(const double _ixx, const double _iyy,
          const double _izz, const double _ixy, const double _ixz,
          const double _iyz);

      /// \brief Set the inertial pose of the link.
      /// \param[in] _pose Inertial pose to set the link to.
      public: void SetInertialPose(const ignition::math::Pose3d &_pose);

      /// \brief Get the configuration widget for the link
      public: const ConfigWidget *GetConfigWidget() const;

      /// \brief Signal emitted when density changes.
      /// \param[in] _value The new density.
      Q_SIGNALS: void DensityValueChanged(const double &_value);

      /// \brief Signal emitted when mass changes.
      /// \param[in] _value The new mass.
      Q_SIGNALS: void MassValueChanged(const double &_value);

      /// \brief Callback for density changes in config widget.
      /// \param[in] _value The new density value.
      private slots: void OnDensityValueChanged(const double &_value);

      /// \brief Callback for mass changes in config widget.
      /// \param[in] _value The new mass value.
      private slots: void OnMassValueChanged(const double &_value);

/// \brief Initialize widget.
      public: void Init();

      /// \brief Restore the widget's data to how it was when first opened.
      public slots: void RestoreOriginalData();

      /// \brief Qt signal emitted to indicate that changes should be applied.
      Q_SIGNALS: void Applied();

      /// \brief Qt callback when a pose value has changed.
      /// \param[in] _name of widget in the config widget that emitted the
      /// signal.
      /// \param[in] _value New value.
      private slots: void OnPoseChanged(const QString &_name,
          const ignition::math::Pose3d &_value);

      /// \brief config widget for configuring link properties.
      private: ConfigWidget *configWidget;

      /// \brief Message containing the data which was in the widget when first
      /// open.
      private: msgs::Link originalDataMsg;
    };
    /// \}
  }
}
#endif
