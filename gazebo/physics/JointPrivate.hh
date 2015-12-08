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
      public: double dissipationCoefficient[MAX_JOINT_AXIS];

      /// \brief joint stiffnessCoefficient
      public: double stiffnessCoefficient[MAX_JOINT_AXIS];

      /// \brief joint spring reference (zero load) position
      public: double springReferencePosition[MAX_JOINT_AXIS];

      /// \brief apply damping for adding viscous damping forces on updates
      public: gazebo::event::ConnectionPtr applyDamping;

      /// \brief Store Joint effort limit as specified in SDF
      public: double effortLimit[MAX_JOINT_AXIS];

      /// \brief Store Joint velocity limit as specified in SDF
      public: double velocityLimit[MAX_JOINT_AXIS];

      /// \brief Store Joint position lower limit as specified in SDF
      public: ignition::math::Angle lowerLimit[MAX_JOINT_AXIS];

      /// \brief Store Joint position upper limit as specified in SDF
      public: ignition::math::Angle upperLimit[MAX_JOINT_AXIS];

      /// \brief Cache Joint force torque values in case physics engine
      /// clears them at the end of update step.
      public: JointWrench wrench;

      /// \brief Flags that are set to true if an axis value is expressed
      /// in the parent model frame. Otherwise use the joint frame.
      /// See issue #494.
      public: bool axisParentModelFrame[MAX_JOINT_AXIS];

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
      public: double stopStiffness[MAX_JOINT_AXIS];

      /// \brief Joint stop dissipation
      public: double stopDissipation[MAX_JOINT_AXIS];
    };
  }
}
#endif
