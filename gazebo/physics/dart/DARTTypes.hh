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

#ifndef _GAZEBO_DARTTYPES_HH_
#define _GAZEBO_DARTTYPES_HH_

#include <boost/shared_ptr.hpp>
#include "gazebo/common/Assert.hh"
#include "gazebo/math/Pose.hh"
#include "gazebo/physics/dart/dart_inc.h"
#include "gazebo/util/system.hh"

/// \file
/// \ingroup gazebo_physics
/// \ingroup gazebo_physics_dart
/// \brief DART wrapper forward declarations and typedefs
namespace gazebo
{
  namespace physics
  {
    class DARTPhysics;
    class DARTModel;
    class DARTLink;
    class DARTJoint;
    class DARTCollision;
    class DARTRayShape;
    class DARTSurfaceParams;

    typedef boost::shared_ptr<DARTPhysics>   DARTPhysicsPtr;
    typedef boost::shared_ptr<DARTModel>     DARTModelPtr;
    typedef boost::shared_ptr<DARTLink>      DARTLinkPtr;
    typedef boost::shared_ptr<DARTJoint>     DARTJointPtr;
    typedef boost::shared_ptr<DARTCollision> DARTCollisionPtr;
    typedef boost::shared_ptr<DARTRayShape>  DARTRayShapePtr;
    typedef boost::shared_ptr<DARTSurfaceParams> DARTSurfaceParamsPtr;

    using DARTBodyNodePropPtr =
      std::shared_ptr<dart::dynamics::BodyNode::Properties>;
    using DARTJointPropPtr =
      std::shared_ptr<dart::dynamics::Joint::Properties>;

    /// \addtogroup gazebo_physics_dart
    /// \{

    /// \class DARTTypes DARTTypes.hh
    /// \brief A set of functions for converting between the math types used
    ///        by gazebo and dart.
    class GZ_PHYSICS_VISIBLE DARTTypes
    {
      /// \brief
      public: static Eigen::Vector3d ConvVec3(const math::Vector3 &_vec3)
        {
            return Eigen::Vector3d(_vec3.x, _vec3.y, _vec3.z);
        }

        /// \brief
      public: static math::Vector3 ConvVec3(const Eigen::Vector3d &_vec3)
        {
            return math::Vector3(_vec3.x(), _vec3.y(), _vec3.z());
        }

        /// \brief
      public: static Eigen::Quaterniond ConvQuat(const math::Quaternion &_quat)
        {
            return Eigen::Quaterniond(_quat.w, _quat.x, _quat.y, _quat.z);
        }

        /// \brief
      public: static math::Quaternion ConvQuat(const Eigen::Quaterniond &_quat)
        {
            return math::Quaternion(_quat.w(), _quat.x(), _quat.y(), _quat.z());
        }

        /// \brief
      public: static Eigen::Isometry3d ConvPose(const math::Pose &_pose)
        {
            // Below line doesn't work with 'libeigen3-dev is 3.0.5-1'
            // return Eigen::Translation3d(ConvVec3(_pose.pos)) *
            //        ConvQuat(_pose.rot);

            Eigen::Isometry3d res = Eigen::Isometry3d::Identity();

            res.translation() = ConvVec3(_pose.pos);
            res.linear() = Eigen::Matrix3d(ConvQuat(_pose.rot));

            return res;
        }

        /// \brief
      public: static math::Pose ConvPose(const Eigen::Isometry3d &_T)
        {
            math::Pose pose;
            pose.pos = ConvVec3(_T.translation());
            pose.rot = ConvQuat(Eigen::Quaterniond(_T.linear()));
            return pose;
        }

      /// \brief Invert thread pitch to match the different definitions of
      /// thread pitch in Gazebo and DART.
      ///
      /// [Definitions of thread pitch]
      /// Gazebo: NEGATIVE angular motion per linear motion.
      /// DART  : linear motion per single rotation.
      public: static double InvertThreadPitch(double _pitch)
      {
        GZ_ASSERT(std::abs(_pitch) > 0.0,
                  "Zero thread pitch is not allowed.\n");

        return -2.0 * M_PI / _pitch;
      }
    };
  }
}

#endif
