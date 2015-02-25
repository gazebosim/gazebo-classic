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

#ifndef _RTQL8JOINT_HH_
#define _RTQL8JOINT_HH_

#include <boost/any.hpp>
#include <string>

#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/physics/Joint.hh"
#include "gazebo/physics/rtql8/RTQL8Physics.hh"

namespace gazebo
{
  namespace physics
  {
    /// \brief RTQL8 joint interface
    class RTQL8Joint : public Joint
    {
      /// \brief Constructor.
      /// \param[in] _parent Parent of the Joint.
      public: RTQL8Joint(BasePtr _parent);

      /// \brief Destructor.
      public: virtual ~RTQL8Joint();

      // Documentation inherited.
      public: virtual void Load(sdf::ElementPtr _sdf);

      // Documentation inherited.
      public: virtual void Reset();

      // Documentation inherited.
      public: virtual LinkPtr GetJointLink(int _index) const;

      // Documentation inherited.
      public: virtual bool AreConnected(LinkPtr _one, LinkPtr _two) const;

      // Documentation inherited.
      public: virtual void Attach(LinkPtr _parent, LinkPtr _child);

      // Documentation inherited.
      public: virtual void Detach();

      // Documentation inherited.
      public: virtual void SetHighStop(int _index, const math::Angle &_angle);

      // Documentation inherited.
      public: virtual void SetLowStop(int _index, const math::Angle &_angle);

      // Documentation inherited.
      public: virtual math::Angle GetHighStop(int _index);

      // Documentation inherited.
      public: virtual math::Angle GetLowStop(int _index);

      // Documentation inherited.
      public: virtual math::Vector3 GetLinkForce(unsigned int _index) const;

      // Documentation inherited.
      public: virtual math::Vector3 GetLinkTorque(unsigned int _index) const;

      // Documentation inherited.
      public: virtual void SetAttribute(const std::string &_key, int _index,
                                      const boost::any &_value);

      // Documentation inherited.
      public: virtual double GetAttribute(const std::string &_key,
                                        unsigned int _index);

      /// \brief
      protected: rtql8::kinematics::Joint* rtql8Joint;

      // Documentation inherited.
      public: virtual JointWrench GetForceTorque(int _index);

      // Documentation inherited.
      public: virtual JointWrench GetForceTorque(unsigned int _index);

      // Documentation inherited.
      public: virtual unsigned int GetAngleCount() const;

      /// \brief
      public: math::Pose GetPose_ChildLinkToJoint() const
      {
        return poseChildLinkToJoint;
      }

      /// \brief
      public: math::Pose GetPose_ParentLinkToJoint() const
      {
        return poseParentLinkToJoint;
      }

      /// \brief
      public: math::Pose GetPose_JointToChildLink() const
      {
        return poseJointToChildLink;
      }

      /// \brief
      protected: math::Pose poseChildLinkToJoint;

      /// \brief
      protected: math::Pose poseParentLinkToJoint;

      /// \brief
      protected: math::Pose poseJointToChildLink;
    };
  }
}
#endif
