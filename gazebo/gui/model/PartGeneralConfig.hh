/*
 * Copyright 2012 Open Source Robotics Foundation
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

#ifndef _PART_GENERAL_TAB_HH_
#define _PART_GENERAL_TAB_HH_

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

    /// \class PartGeneralConfig PartGeneralConfig.hh
    /// \brief A tab for configuring general properties of a part.
    class PartGeneralConfig : public QWidget
    {
      Q_OBJECT

      /// \brief Constructor
      public: PartGeneralConfig();

      /// \brief Destructor
      public: ~PartGeneralConfig();

      /// \brief Get the msg containing all link data.
      /// \return Link msg.
      public: msgs::Link *GetData() const;

      /// \brief Set the pose of the part.
      /// \param[in] _pose Pose to set the part to.
      public: void SetPose(const math::Pose &_pose);

      private: ConfigWidget *configWidget;

      /*/// \brief Set the part to be affected by gravity.
      /// \param[in] _enabled True to enable gravity on part.
      public: void SetGravity(bool _enabled);

      /// \brief Get whether the part is affected by gravity.
      /// \return True if the part is affected by gravity.
      public: bool GetGravity() const;

      /// \brief Set whether to allow collision with other parts.
      /// \param[in] _enabled True to enable self collision with this part.
      public: void SetSelfCollide(bool _enabled);

      /// \brief Get whether self collision is enabled.
      /// \return True if self collision is enabled.
      public: bool GetSelfCollide() const;

      /// \brief Set the part to be kinematic.
      /// \param[in] _enabled True to set the part to be kinematic.
      public: void SetKinematic(bool _enabled);

      /// \brief Get whether the part is kinematic.
      /// \return True if the part is kinematic.
      public: bool GetKinematic() const;

      /// \brief Set the pose of the part.
      /// \param[in] _pose Pose to set the part to.
      public: void SetPose(const math::Pose &_pose);

      /// \brief Get the pose of the part.
      /// \return Pose of the part.
      public: math::Pose GetPose() const;

      /// \brief Set the mass of the part
      /// \param[in] _mass Mass to set the part to.
      public: void SetMass(double mass);

      /// \brief Get the mass of the part
      /// \return Mass of the part.
      public: double GetMass() const;

      /// \brief Set the inertial pose of the part.
      /// \param[in] _pose Inertial pose to set the part to.
      public: void SetInertialPose(const math::Pose &_pose);

      /// \brief Get the inertial pose of the part.
      /// \return Inertial pose of the part.
      public: math::Pose GetInertialPose() const;

      /// \brief Set the part inertia.
      /// \param[in] _ixx Inertia IXX value.
      /// \param[in] _iyy Inertia IYY value.
      /// \param[in] _izz Inertia IZZ value.
      /// \param[in] _ixy Inertia IXY value.
      /// \param[in] _ixz Inertia IXZ value.
      /// \param[in] _iyz Inertia IYZ value.
      public: void SetInertia(double _ixx, double _iyy, double _izz,
          double _ixy, double _ixz, double _iyz);

      /// \brief Get IXX
      /// \return Inertia IXX value
      public: double GetInertiaIXX() const;

      /// \brief Get IYY
      /// \return Inertia IYY value
      public: double GetInertiaIYY() const;

      /// \brief Get IZZ
      /// \return Inertia IZZ value
      public: double GetInertiaIZZ() const;

      /// \brief Get IXY
      /// \return Inertia IXY value
      public: double GetInertiaIXY() const;

      /// \brief Get IXZ
      /// \return Inertia IXZ value
      public: double GetInertiaIXZ() const;

      /// \brief Get IXZ
      /// \return Inertia IYZ value
      public: double GetInertiaIYZ() const;

      /// \brief Qt call back when the part gravity property is toggled.
      private slots: void OnGravity();

      /// \brief Qt call back when the self collide property is toggled.
      private slots: void OnSelfCollide();

      /// \brief Qt call back when the kinematic property is toggled.
      private slots: void OnKinematic();

      /// \brief Spin box for configuring the X position of the part.
      private: QDoubleSpinBox *posXSpinBox;

      /// \brief Spin box for configuring the Y position of the part.
      private: QDoubleSpinBox *posYSpinBox;

      /// \brief Spin box for configuring the Z position of the part.
      private: QDoubleSpinBox *posZSpinBox;

      /// \brief Spin box for configuring the roll of the part.
      private: QDoubleSpinBox *rotRSpinBox;

      /// \brief Spin box for configuring the pitch of the part.
      private: QDoubleSpinBox *rotPSpinBox;

      /// \brief Spin box for configuring the yaw of the part.
      private: QDoubleSpinBox *rotYSpinBox;

      /// \brief Spin box for configuring mass of the part.
      private: QDoubleSpinBox *massSpinBox;

      /// \brief Spin box for configuring the X inertial position of the part.
      private: QDoubleSpinBox *inertialPosXSpinBox;

      /// \brief Spin box for configuring the Y inertial position of the part.
      private: QDoubleSpinBox *inertialPosYSpinBox;

      /// \brief Spin box for configuring the Z inertial position of the part.
      private: QDoubleSpinBox *inertialPosZSpinBox;

      /// \brief Spin box for configuring the roll of the part.
      private: QDoubleSpinBox *inertialRotRSpinBox;

      /// \brief Spin box for configuring the pitch of the part.
      private: QDoubleSpinBox *inertialRotPSpinBox;

      /// \brief Spin box for configuring the yaw of the part.
      private: QDoubleSpinBox *inertialRotYSpinBox;

      /// \brief Spin box for configuring inertia ixx.
      private: QDoubleSpinBox *inertiaIXXSpinBox;

      /// \brief Spin box for configuring inertia ixy.
      private: QDoubleSpinBox *inertiaIXYSpinBox;

      /// \brief Spin box for configuring inertia ixz.
      private: QDoubleSpinBox *inertiaIXZSpinBox;

      /// \brief Spin box for configuring inertia iyy.
      private: QDoubleSpinBox *inertiaIYYSpinBox;

      /// \brief Spin box for configuring inertia iyz.
      private: QDoubleSpinBox *inertiaIYZSpinBox;

      /// \brief Spin box for configuring inertia izz.
      private: QDoubleSpinBox *inertiaIZZSpinBox;

      /// \brief Gravity checkbox, true to enable gravity.
      private: QCheckBox *gravityCheck;

      /// \brief Self collide checkbox, true to enable self-collide.
      private: QCheckBox *selfCollideCheck;

      /// \brief Kinematic checkbox, true to make the part kinematic.
      private: QCheckBox *kinematicCheck;*/
    };
  }
}
#endif
