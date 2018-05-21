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

#ifndef _GAZEBO_DARTMODEL_PRIVATE_HH_
#define _GAZEBO_DARTMODEL_PRIVATE_HH_

#include <string>
#include <utility>
#include <memory>

#include "gazebo/physics/dart/dart_inc.h"

namespace gazebo
{
  namespace physics
  {
    /// \internal
    /// \brief Private data class for DARTModel
    class DARTModelPrivate
    {
      // NOTE: Below untidy type definitions and static functions will be
      // cleand-up DART implements SkeletonBuilder which would play a role
      // similiar to Simbody's MultibodyGraphMaker.

      public: using JointPropPtr =
          std::shared_ptr<dart::dynamics::Joint::Properties>;

      private: template <typename BodyTypeT>
      static std::pair<dart::dynamics::Joint*, dart::dynamics::BodyNode*>
      CreateJointAndNodePair(
          dart::dynamics::SkeletonPtr _skeleton,
          dart::dynamics::BodyNode* _parent,
          const std::string &_jointType,
          JointPropPtr _jointProperties,
          const typename BodyTypeT::Properties &_bodyNodeProperties)
      {
        if (std::string("weld") == _jointType)
        {
          return _skeleton->createJointAndBodyNodePair<
              dart::dynamics::WeldJoint, BodyTypeT>(_parent,
              static_cast<const dart::dynamics::WeldJoint::Properties&>(
              *_jointProperties), _bodyNodeProperties);
        }
        else if (std::string("prismatic") == _jointType)
        {
          return _skeleton->createJointAndBodyNodePair<
              dart::dynamics::PrismaticJoint, BodyTypeT>(_parent,
              static_cast<const dart::dynamics::PrismaticJoint::Properties&>(
              *_jointProperties), _bodyNodeProperties);
        }
        else if (std::string("revolute") == _jointType)
        {
          return _skeleton->createJointAndBodyNodePair<
              dart::dynamics::RevoluteJoint, BodyTypeT>(_parent,
              static_cast<const dart::dynamics::RevoluteJoint::Properties&>(
              *_jointProperties), _bodyNodeProperties);
        }
        else if (std::string("screw") == _jointType)
        {
          return _skeleton->createJointAndBodyNodePair<
              dart::dynamics::ScrewJoint, BodyTypeT>(_parent,
              static_cast<const dart::dynamics::ScrewJoint::Properties&>(
              *_jointProperties), _bodyNodeProperties);
        }
        else if (std::string("universal") == _jointType)
        {
          return _skeleton->createJointAndBodyNodePair<
              dart::dynamics::UniversalJoint, BodyTypeT>(_parent,
              static_cast<const dart::dynamics::UniversalJoint::Properties&>(
              *_jointProperties), _bodyNodeProperties);
        }
        else if (std::string("ball") == _jointType)
        {
          return _skeleton->createJointAndBodyNodePair<
              dart::dynamics::BallJoint, BodyTypeT>(_parent,
              static_cast<const dart::dynamics::BallJoint::Properties&>(
              *_jointProperties), _bodyNodeProperties);
        }
        else if (std::string("euler") == _jointType)
        {
          return _skeleton->createJointAndBodyNodePair<
              dart::dynamics::EulerJoint, BodyTypeT>(_parent,
              static_cast<const dart::dynamics::EulerJoint::Properties&>(
              *_jointProperties), _bodyNodeProperties);
        }
        else if (std::string("translational") == _jointType)
        {
          return _skeleton->createJointAndBodyNodePair<
              dart::dynamics::TranslationalJoint, BodyTypeT>(_parent,
              static_cast<
                  const dart::dynamics::TranslationalJoint::Properties&>(
              *_jointProperties), _bodyNodeProperties);
        }
        else if (std::string("planar") == _jointType)
        {
          return _skeleton->createJointAndBodyNodePair<
              dart::dynamics::PlanarJoint, BodyTypeT>(_parent,
              static_cast<const dart::dynamics::PlanarJoint::Properties&>(
              *_jointProperties), _bodyNodeProperties);
        }
        else if (std::string("free") == _jointType)
        {
          return _skeleton->createJointAndBodyNodePair<
              dart::dynamics::FreeJoint, BodyTypeT>(_parent,
              static_cast<const dart::dynamics::FreeJoint::Properties&>(
              *_jointProperties), _bodyNodeProperties);
        }
        else
        {
          gzerr << "Unsupported Joint type (" << _jointType << ").\n";
          return std::pair<dart::dynamics::Joint*, dart::dynamics::BodyNode*>(
              nullptr, nullptr);
        }
      }

      public: static bool CreateLoopJointAndNodePair(
          dart::simulation::WorldPtr _world,
          dart::dynamics::SkeletonPtr _skeleton,
          dart::dynamics::BodyNode* _parent,
          JointPtr _joint,
          LinkPtr _link)
      {
        DARTLinkPtr dartLink = boost::dynamic_pointer_cast<DARTLink>(_link);
        GZ_ASSERT(dartLink, "DART link is null");

        dart::dynamics::BodyNode* dtChildBodyNode =
            _skeleton->getBodyNode(_link->GetName());
        GZ_ASSERT(dtChildBodyNode, "Child body node is null");

        DARTJointPtr dartJoint = boost::dynamic_pointer_cast<DARTJoint>(_joint);
        GZ_ASSERT(dartJoint, "DART joint is null");

        std::pair<dart::dynamics::Joint*, dart::dynamics::BodyNode*> pair =
            DARTModelPrivate::CreateJointAndNodePair<dart::dynamics::BodyNode>(
            _skeleton, _parent, GetDARTJointType(dartJoint),
            dartJoint->DARTProperties(),
            dart::dynamics::BodyNode::AspectProperties());

        if (pair.first == nullptr || pair.second == nullptr)
          return false;

        dartJoint->SetDARTJoint(pair.first);
        pair.second->setCollidable(false);
        dartLink->AddSlaveBodyNode(pair.second);

        // Developer note (PCH): We can assume that the body nodes have the same
        // frame here because their parent joint transforms will not be set
        // until Model::Init is called.
        // Free joint transforms have been set, but free joints are only created
        // for links with 0 parent joints while loop joints are only created for
        // links with >1 parent joints, so the two conditions will not overlap.
        dart::constraint::WeldJointConstraintPtr dtWeldJointConst;
        dtWeldJointConst.reset(new dart::constraint::WeldJointConstraint(
            dtChildBodyNode, pair.second));
        _world->getConstraintSolver()->addConstraint(dtWeldJointConst);

        return true;
      }

      public: static bool CreateJointAndNodePair(
          dart::dynamics::SkeletonPtr _skeleton,
          dart::dynamics::BodyNode* _parent,
          JointPtr _joint,
          LinkPtr _link)
      {
        DARTLinkPtr dartLink = boost::dynamic_pointer_cast<DARTLink>(_link);
        GZ_ASSERT(dartLink, "DART link is null");
        DARTJointPtr dartJoint = nullptr;

        std::string jointType;
        JointPropPtr jointProperties;
        if (_joint == nullptr)
        {
          jointType = "free";
          // Developer note (PCH): We set free joint transforms here because the
          // free joints do not have corresponding Gazebo DARTJoints and so will
          // not be initialized with the others when Model::Init is called.
          jointProperties =
              std::make_shared<dart::dynamics::FreeJoint::Properties>(
              dart::dynamics::Joint::Properties("root",
              DARTTypes::ConvPose(dartLink->WorldPose())));
        }
        else
        {
          dartJoint = boost::dynamic_pointer_cast<DARTJoint>(_joint);
          GZ_ASSERT(dartJoint, "DART joint is null");

          jointType = GetDARTJointType(dartJoint);
          jointProperties = dartJoint->DARTProperties();
        }

        std::pair<dart::dynamics::Joint*, dart::dynamics::BodyNode*> pair;
        if (dartLink->IsSoftBody())
        {
          pair = CreateJointAndNodePair<dart::dynamics::SoftBodyNode>(
              _skeleton, _parent, jointType, jointProperties,
              static_cast<const dart::dynamics::SoftBodyNode::Properties&>(
              *(dartLink->DARTProperties())));
        }
        else
        {
          pair = CreateJointAndNodePair<dart::dynamics::BodyNode>(
              _skeleton, _parent, jointType, jointProperties,
              static_cast<const dart::dynamics::BodyNode::Properties&>(
              *(dartLink->DARTProperties())));
        }

        if (pair.first == nullptr || pair.second == nullptr)
          return false;

        dart::dynamics::BodyNode* newBodyNode = pair.second;
        dart::dynamics::Joint* newJoint = pair.first;

        dartLink->SetDARTBodyNode(newBodyNode);

        // Gazebo does not create joint for DART FreeJoint
        if (dartJoint)
        {
          dartJoint->SetDARTJoint(newJoint);
        }

        return true;
      }

      public: static std::string GetDARTJointType(
          const DARTJointPtr &_dartJoint)
      {
        if (_dartJoint->HasType(Base::BALL_JOINT))
          return "ball";
        else if (_dartJoint->HasType(Base::HINGE_JOINT))
          return "revolute";
        else if (_dartJoint->HasType(Base::HINGE2_JOINT))
          return "universal";
        else if (_dartJoint->HasType(Base::SLIDER_JOINT))
          return "prismatic";
        else if (_dartJoint->HasType(Base::SCREW_JOINT))
          return "screw";
        else if (_dartJoint->HasType(Base::UNIVERSAL_JOINT))
          return "universal";
        else if (_dartJoint->HasType(Base::FIXED_JOINT))
          return "weld";
        else
        {
          GZ_ASSERT(false, "Unsupported joint type.");
          return "";
        }
      }

      /// \brief Constructor
      public: DARTModelPrivate()
        : dtSkeleton(dart::dynamics::Skeleton::create()),
          genPositions(Eigen::VectorXd()),
          genVelocities(Eigen::VectorXd())
      {
      }

      /// \brief Default destructor
      public: ~DARTModelPrivate() = default;

      /// \brief Pointer to DART Skeleton
      public: dart::dynamics::SkeletonPtr dtSkeleton;

      /// \brief Generalized positions
      public: Eigen::VectorXd genPositions;

      /// \brief Generalized velocities
      public: Eigen::VectorXd genVelocities;

      // To get byte-aligned Eigen vectors
      public: EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
  }
}
#endif
