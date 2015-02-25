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

#ifndef _RTQL8UTILS_HH_
#define _RTQL8UTILS_HH_

#include "gazebo/math/Pose.hh"
#include "gazebo/physics/rtql8/rtql8_inc.h"

namespace gazebo
{
  namespace physics
  {
    /// \ingroup gazebo_physics
    /// \addtogroup gazebo_physics_rtql8 RTQL8 Physics
    /// \brief rtql8 utilities
    /// \{

    /// \brief RTQL8 Utils class
    class RTQL8Utils
    {
      /// \brief Convert from gazebo::math::Pose to Eigen::Matrix4d.
      public: static void ConvPoseToMat(Eigen::Matrix4d* _mat,
                                        const math::Pose& _pose);

      /// \brief Convert from Eigen::Matrix4d to gazebo::math::Pose.
      public: static void ConvMatToPose(math::Pose* _pose,
                                        const Eigen::Matrix4d& _mat);

      /// \brief Create rtql8::kinematics::TrfmTranslate from math::Vector3.
      public: static rtql8::kinematics::TrfmTranslate* createTrfmTranslate(
          const math::Vector3& _vec);

      /// \brief Create rtql8::kinematics::TrfmRotateQuat from math::Vector3.
      public: static rtql8::kinematics::TrfmRotateQuat* createTrfmRotateQuat(
          const math::Quaternion& _quat);

      /// \brief addTransform from math::Pose.
      public: static void addTransformToRTQL8Joint(
          rtql8::kinematics::Joint* _rtl8Joint,
          const math::Pose& _pose);

      /// \brief Add 6dof joint for free floating body.
      public: static void add6DOFToRTQL8Joint(
          rtql8::kinematics::Joint* _rtql8Joint,
          const math::Pose& _initialPose);
    };
    /// \}
  }
}
#endif
