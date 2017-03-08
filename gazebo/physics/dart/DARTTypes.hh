/*
 * Copyright (C) 2014 Open Source Robotics Foundation
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
#ifndef GAZEBO_PHYSICS_DART_DARTTYPES_HH_
#define GAZEBO_PHYSICS_DART_DARTTYPES_HH_

#include <boost/shared_ptr.hpp>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>

#include "gazebo/common/Assert.hh"
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
      /// \brief Convert ignition math vector3d to eigen vector3d.
      /// \param[in] _vec3 Ignition math equalivent object.
      /// \return Eigen vector3 to convert.
      public: static Eigen::Vector3d ConvVec3(
          const ignition::math::Vector3d &_vec3)
      {
        return Eigen::Vector3d(_vec3.X(), _vec3.Y(), _vec3.Z());
      }

      /// \brief Convert eigen vector3d to ignition math vector3d.
      /// \param[in] _vec3 Eigen vector3 to convert.
      /// \return Ignition math equalivent object.
      public: static ignition::math::Vector3d ConvVec3Ign(
                  const Eigen::Vector3d &_vec3)
      {
        return ignition::math::Vector3d(_vec3.x(), _vec3.y(), _vec3.z());
      }

      /// \brief Convert ignition quaternion to eigen quaternion.
      /// \param[in] _quat Ignition object.
      /// \return Equivalent eigen object.
      public: static Eigen::Quaterniond ConvQuat(
          const ignition::math::Quaterniond &_quat)
      {
        return Eigen::Quaterniond(_quat.W(), _quat.X(), _quat.Y(), _quat.Z());
      }

      /// \brief Convert eigen quaternion to ignition quaternion.
      /// \param[in] _quat Eigen object to convert.
      /// \return Equivalent ignition object.
      public: static ignition::math::Quaterniond ConvQuatIgn(
                  const Eigen::Quaterniond &_quat)
      {
        return ignition::math::Quaterniond(
            _quat.w(), _quat.x(), _quat.y(), _quat.z());
      }

      /// \brief
      public: static Eigen::Isometry3d ConvPose(
          const ignition::math::Pose3d &_pose)
      {
          // Below line doesn't work with 'libeigen3-dev is 3.0.5-1'
          // return Eigen::Translation3d(ConvVec3(_pose.pos)) *
          //        ConvQuat(_pose.rot);

          Eigen::Isometry3d res = Eigen::Isometry3d::Identity();

          res.translation() = ConvVec3(_pose.Pos());
          res.linear() = Eigen::Matrix3d(ConvQuat(_pose.Rot()));

          return res;
      }

      /// \brief Convert eigen iosmetry3d to ignition math pose3d.
      /// \param[in] _T eigen object to convert
      /// \return Equvalent ignition math object.
      public: static ignition::math::Pose3d ConvPoseIgn(
                  const Eigen::Isometry3d &_T)
      {
        ignition::math::Pose3d pose;
        pose.Pos() = ConvVec3Ign(_T.translation());
        pose.Rot() = ConvQuatIgn(Eigen::Quaterniond(_T.linear()));
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
