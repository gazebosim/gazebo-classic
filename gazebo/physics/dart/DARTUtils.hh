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
      /// \brief Convert from gazebo::math::Pose to Eigen::Matrix4d.
      public: static void ConvPoseToMat(Eigen::Matrix4d* _mat,
                                        const math::Pose& _pose);

      /// \brief Convert from Eigen::Matrix4d to gazebo::math::Pose.
      public: static void ConvMatToPose(math::Pose* _pose,
                                        const Eigen::Matrix4d& _mat);

      public: static dart::math::Vec3 ConvertVector3(const math::Vector3& _vec3)
      {
        return dart::math::Vec3(_vec3.x, _vec3.y, _vec3.z);
      }

      public: static math::Vector3 ConvertVector3(const dart::math::Vec3& _vec3)
      {
        return math::Vector3(_vec3(0), _vec3(1), _vec3(2));
      }

      public: static dart::math::Axis ConvertAxis(const math::Vector3& _vec3)
      {
        return dart::math::Axis(_vec3.x, _vec3.y, _vec3.z);
      }

      public: static math::Vector3 ConvertAxis(const dart::math::Axis& _axis)
      {
        return math::Vector3(_axis(0), _axis(1), _axis(2));
      }

      public: static dart::math::SE3 ConvertPose(const math::Pose& _pose)
      {
        dart::math::SE3 T;

        T.setPosition(ConvertVector3(_pose.pos));
        T.setOrientationQuaternion(_pose.rot.w,
                                   _pose.rot.x, _pose.rot.y, _pose.rot.z);

        return T;
      }

      public: static math::Pose ConvertPose(const dart::math::SE3& _T)
      {
        math::Pose pose;
        double quat[4];

        pose.pos = ConvertVector3(_T.getPosition());
        _T.toQuaternion(quat);
        pose.rot.w = quat[0];
        pose.rot.x = quat[1];
        pose.rot.y = quat[2];
        pose.rot.z = quat[3];

        return pose;
      }
    };
    /// \}
  }
}
#endif
