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
#ifndef _GAZEBO_PHYSICS_JOINT_PRIVATE_HH_
#define _GAZEBO_PHYSICS_JOINT_PRIVATE_HH_

#include <array>
#include <string>
#include <vector>

#include <sdf/sdf.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Angle.hh>

#include "gazebo/common/Event.hh"

#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/physics/JointWrench.hh"
#include "gazebo/physics/BasePrivate.hh"

/// \brief maximum number of axis per joint anticipated.
/// Currently, this is 2 as 3-axis joints (e.g. ball)
/// actuation, control is not there yet.
#define MAX_JOINT_AXIS 2

namespace gazebo
{
  namespace physics
  {
    /// \internal
    /// \brief Protected data for joints
    class JointProtected : public BaseProtected
    {
      /// \brief The first link this joint connects to
      public: LinkPtr childLink;

      /// \brief The second link this joint connects to
      public: LinkPtr parentLink;

      /// \brief Pointer to the parent model.
      public: ModelPtr model;

      /// \brief Anchor pose.  This is the xyz offset of the joint frame from
      /// child frame specified in the parent link frame
      public: ignition::math::Vector3d anchorPos;

      /// \brief Anchor pose specified in SDF <joint><pose> tag.
      /// AnchorPose is the transform from child link frame to joint frame
      /// specified in the child link frame.
      /// AnchorPos is more relevant in normal usage, but sometimes,
      /// we do need this (e.g. GetForceTorque and joint visualization).
      public: ignition::math::Pose3d anchorPose;

      /// \brief Anchor pose relative to parent link frame.
      public: ignition::math::Pose3d parentAnchorPose;

      /// \brief Anchor link.
      public: LinkPtr anchorLink;

      /// \brief joint viscous damping coefficient
      public: std::array<double, MAX_JOINT_AXIS> dissipationCoefficient;

      /// \brief joint stiffnessCoefficient
      public: std::array<double, MAX_JOINT_AXIS> stiffnessCoefficient;

      /// \brief joint spring reference (zero load) position
      public: std::array<double, MAX_JOINT_AXIS> springReferencePosition;

      /// \brief apply damping for adding viscous damping forces on updates
      public: gazebo::event::ConnectionPtr applyDamping;

      /// \brief Store Joint effort limit as specified in SDF
      public: std::array<double, MAX_JOINT_AXIS> effortLimit;

      /// \brief Store Joint velocity limit as specified in SDF
      public: std::array<double, MAX_JOINT_AXIS> velocityLimit;

      /// \brief Store Joint position lower limit as specified in SDF
      public: std::array<ignition::math::Angle, MAX_JOINT_AXIS> lowerLimit;

      /// \brief Store Joint position upper limit as specified in SDF
      public: std::array<ignition::math::Angle, MAX_JOINT_AXIS> upperLimit;

      /// \brief Cache Joint force torque values in case physics engine
      /// clears them at the end of update step.
      public: JointWrench wrench;

      /// \brief Flags that are set to true if an axis value is expressed
      /// in the parent model frame. Otherwise use the joint frame.
      /// See issue #494.
      public: std::array<bool, MAX_JOINT_AXIS> axisParentModelFrame;

      /// \brief Provide Feedback data for contact forces
      public: bool provideFeedback;
    };

    /// \internal
    /// \brief Private data for joints
    class JointPrivate
    {
      /// \brief An SDF pointer that allows us to only read the joint.sdf
      /// file once, which in turns limits disk reads.
      public: static sdf::ElementPtr sdfJoint;

      /// \brief Names of all the sensors attached to the link.
      public: std::vector<std::string> sensors;

      /// \brief Joint update event.
      public: event::EventT<void ()> jointUpdate;

      /// \brief Angle used when the joint is parent of a static model.
      public: ignition::math::Angle staticAngle;

      /// \brief Joint stop stiffness
      public: std::array<double, MAX_JOINT_AXIS> stopStiffness;

      /// \brief Joint stop dissipation
      public: std::array<double, MAX_JOINT_AXIS> stopDissipation;
    };
  }
}
#endif
