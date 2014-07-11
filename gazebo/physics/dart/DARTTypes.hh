/*
 * Copyright 2014 Open Source Robotics Foundation
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
#include <ignition/math/Pose3.hh>
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

    typedef boost::shared_ptr<DARTPhysics>   DARTPhysicsPtr;
    typedef boost::shared_ptr<DARTModel>     DARTModelPtr;
    typedef boost::shared_ptr<DARTLink>      DARTLinkPtr;
    typedef boost::shared_ptr<DARTJoint>     DARTJointPtr;
    typedef boost::shared_ptr<DARTCollision> DARTCollisionPtr;
    typedef boost::shared_ptr<DARTRayShape>  DARTRayShapePtr;

    /// \addtogroup gazebo_physics_dart
    /// \{

    /// \class DARTTypes DARTTypes.hh
    /// \brief A set of functions for converting between the math types used
    ///        by gazebo and dart.
    class GAZEBO_VISIBLE DARTTypes
    {
      /// \brief
      public: static Eigen::Vector3d ConvVec3(
                  const ignition::math::Vector3d &_vec3)
        {
            return Eigen::Vector3d(_vec3.X(), _vec3.Y(), _vec3.Z());
        }

        /// \brief
      public: static ignition::math::Vector3d ConvVec3(
                  const Eigen::Vector3d &_vec3)
        {
            return ignition::math::Vector3d(_vec3.X(), _vec3.Y(), _vec3.Z());
        }

        /// \brief
      public: static Eigen::Quaterniond ConvQuat(
                  const ignition::math::Quaterniond &_quat)
        {
            return Eigen::Quaterniond(_quat.w, _quat.X(),
                _quat.Y(), _quat.Z());
        }

        /// \brief
      public: static ignition::math::Quaterniond ConvQuat(
                  const Eigen::Quaterniond &_quat)
        {
            return ignition::math::Quaterniond(_quat.W(), _quat.X(),
                _quat.Y(), _quat.Z());
        }

        /// \brief
      public: static Eigen::Isometry3d ConvPose(
                  const ignition::math::Pose3d &_pose)
        {
            // Below line doesn't work with 'libeigen3-dev is 3.0.5-1'
            // return Eigen::Translation3d(ConvVec3(_pose.Pos())) *
            //        ConvQuat(_pose.Rot());

            Eigen::Isometry3d res;

            res.translation() = ConvVec3(_pose.Pos());
            res.linear() = Eigen::Matrix3d(ConvQuat(_pose.Rot()));

            return res;
        }

        /// \brief
      public: static ignition::math::Pose3d ConvPose(
                  const Eigen::Isometry3d &_T)
        {
            ignition::math::Pose3d pose;
            pose.Pos() = ConvVec3(_T.translation());
            pose.Rot() = ConvQuat(Eigen::Quaterniond(_T.linear()));
            return pose;
        }
    };
  }
}

#endif
