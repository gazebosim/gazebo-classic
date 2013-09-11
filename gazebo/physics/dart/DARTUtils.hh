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

#ifndef _DARTUTILS_HH_
#define _DARTUTILS_HH_

#include "gazebo/math/Pose.hh"
#include "gazebo/physics/dart/dart_inc.h"

namespace gazebo
{
  namespace physics
  {
    /// \ingroup gazebo_physics
    /// \addtogroup gazebo_physics_dart DART Physics
    /// \brief dart utilities
    /// \{

    /// \brief DART Utils class
    class DARTUtils
    {
      /// \brief
      public: static Eigen::Vector3d ConvVec3(const math::Vector3& _vec3)
      {
        return Eigen::Vector3d(_vec3.x, _vec3.y, _vec3.z);
      }

      /// \brief
      public: static math::Vector3 ConvVec3(const Eigen::Vector3d& _vec3)
      {
        return math::Vector3(_vec3.x(), _vec3.y(), _vec3.z());
      }

      /// \brief
      public: static Eigen::Quaterniond ConvQuat(const math::Quaternion& _quat)
      {
        return Eigen::Quaterniond(_quat.w, _quat.x, _quat.y, _quat.z);
      }

      /// \brief
      public: static math::Quaternion ConvQuat(const Eigen::Quaterniond& _quat)
      {
        return math::Quaternion(_quat.w(), _quat.x(), _quat.y(), _quat.z());
      }

      /// \brief
      public: static Eigen::Isometry3d ConvPose(const math::Pose& _pose)
      {
        return Eigen::Translation3d(ConvVec3(_pose.pos)) *
            Eigen::Quaterniond(ConvQuat(_pose.rot));
      }

      /// \brief
      public: static math::Pose ConvPose(const Eigen::Isometry3d& _T)
      {
        math::Pose pose;
        pose.pos = ConvVec3(_T.translation());
        pose.rot = ConvQuat(Eigen::Quaterniond(_T.linear()));
        return pose;
      }
    };
    /// \}
  }
}
#endif
