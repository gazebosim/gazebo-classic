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

#include <map>
#include <string>
#include <utility>

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

      public: using BodyPropPtr =
        std::shared_ptr<dart::dynamics::BodyNode::Properties>;
      public: using JointPropPtr =
        std::shared_ptr<dart::dynamics::Joint::Properties>;

      public: enum NextResult
      {
        VALID,
        CONTINUE,
        BREAK,
        CREATE_FREEJOINT_ROOT
      };

      public: struct BodyNodeBuildData
      {
        DARTLinkPtr dartLink;
        BodyPropPtr properties;
        Eigen::Isometry3d initTransform;
        std::string type;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      };

      public: struct JointBuildData
      {
        DARTJointPtr dartJoint;
        JointPropPtr properties;
        std::string parentName;
        std::string childName;
        std::string type;
      };

      /// \brief first: BodyNode name | second: BodyNode information
      public: using BodyNodeMap =
        Eigen::aligned_map<std::string, BodyNodeBuildData>;

      /// \brief first: Child BodyNode name | second: Joint information
      public: using JointMap = std::map<std::string, JointBuildData>;

      /// \brief first: Order that Joint appears in file | second: Child
      /// BodyNode name
      public: using IndexToChildBodyNodeName =
        std::map<std::size_t, std::string>;

      /// \brief first: Child BodyNode name | second: Order that Joint appears
      /// in file
      public: using ChildBodyNodeNameToIndex =
        std::map<std::string, std::size_t>;

      public: static NextResult getNextJointAndNodePair(
          BodyNodeMap::const_iterator& bodyNodeItr,
          JointMap::const_iterator& parentJointItr,
          dart::dynamics::BodyNode*& parentBody,
          const dart::dynamics::SkeletonPtr skeleton,
          const BodyNodeMap& bodyNodeMap,
          const JointMap& jointMap)
      {
        parentJointItr = jointMap.find(bodyNodeItr->first);
        if (parentJointItr == jointMap.end())
          return CREATE_FREEJOINT_ROOT;

        const std::string& parentBodyName =
            parentJointItr->second.parentName;
        const std::string& parentJointName =
            parentJointItr->second.properties->mName;

        // Check if the parent Body is created yet
        parentBody = skeleton->getBodyNode(parentBodyName);
        if (NULL == parentBody
            && parentBodyName != "world"
            && !parentBodyName.empty())
        {
          // Find the properties of the parent Joint of the current Joint,
          // because it does not seem to be created yet.
          BodyNodeMap::const_iterator check_parent_body =
              bodyNodeMap.find(parentBodyName);

          if (check_parent_body == bodyNodeMap.end())
          {
            // The BodyNode does not exist in the file
            gzerr << "Could not find Link "
                  << "named [" << parentBodyName << "] requested as parent of "
                  << "Joint [" << parentJointName << "]. We will now quit "
                  << "parsing.\n";
            return BREAK;
          }
          else
          {
            bodyNodeItr = check_parent_body;

            // Create the parent before creating the current Joint
            return CONTINUE;
          }
        }

        return VALID;
      }

      public: template <typename BodyTypeT>
      static std::pair<dart::dynamics::Joint*, dart::dynamics::BodyNode*>
      createJointAndNodePair(
          dart::dynamics::SkeletonPtr skeleton,
          dart::dynamics::BodyNode* parent,
          const JointBuildData& jointBuildData,
          const typename BodyTypeT::Properties& bodyNodeProperties)
      {
        if (std::string("weld") == jointBuildData.type)
        {
          return skeleton->createJointAndBodyNodePair<
            dart::dynamics::WeldJoint, BodyTypeT>(parent,
              static_cast<const dart::dynamics::WeldJoint::Properties&>(
                *jointBuildData.properties), bodyNodeProperties);
        }
        else if (std::string("prismatic") == jointBuildData.type)
        {
          return skeleton->createJointAndBodyNodePair<
            dart::dynamics::PrismaticJoint, BodyTypeT>(parent,
              static_cast<const dart::dynamics::PrismaticJoint::Properties&>(
                *jointBuildData.properties), bodyNodeProperties);
        }
        else if (std::string("revolute") == jointBuildData.type)
        {
          return skeleton->createJointAndBodyNodePair<
            dart::dynamics::RevoluteJoint, BodyTypeT>(parent,
              static_cast<const dart::dynamics::RevoluteJoint::Properties&>(
                *jointBuildData.properties), bodyNodeProperties);
        }
        else if (std::string("screw") == jointBuildData.type)
        {
          return skeleton->createJointAndBodyNodePair<
            dart::dynamics::ScrewJoint, BodyTypeT>(parent,
              static_cast<const dart::dynamics::ScrewJoint::Properties&>(
                *jointBuildData.properties), bodyNodeProperties);
        }
        else if (std::string("universal") == jointBuildData.type)
        {
          return skeleton->createJointAndBodyNodePair<
            dart::dynamics::UniversalJoint, BodyTypeT>(parent,
              static_cast<const dart::dynamics::UniversalJoint::Properties&>(
                *jointBuildData.properties), bodyNodeProperties);
        }
        else if (std::string("ball") == jointBuildData.type)
        {
          return skeleton->createJointAndBodyNodePair<
            dart::dynamics::BallJoint, BodyTypeT>(parent,
              static_cast<const dart::dynamics::BallJoint::Properties&>(
                *jointBuildData.properties), bodyNodeProperties);
        }
        else if (std::string("euler") == jointBuildData.type)
        {
          return skeleton->createJointAndBodyNodePair<
            dart::dynamics::EulerJoint, BodyTypeT>(parent,
              static_cast<const dart::dynamics::EulerJoint::Properties&>(
                *jointBuildData.properties), bodyNodeProperties);
        }
        else if (std::string("translational") == jointBuildData.type)
        {
          return skeleton->createJointAndBodyNodePair<
            dart::dynamics::TranslationalJoint, BodyTypeT>(parent,
              static_cast<
                const dart::dynamics::TranslationalJoint::Properties&>(
                  *jointBuildData.properties), bodyNodeProperties);
        }
        else if (std::string("planar") == jointBuildData.type)
        {
          return skeleton->createJointAndBodyNodePair<
            dart::dynamics::PlanarJoint, BodyTypeT>(parent,
              static_cast<const dart::dynamics::PlanarJoint::Properties&>(
                *jointBuildData.properties), bodyNodeProperties);
        }
        else if (std::string("free") == jointBuildData.type)
        {
          return skeleton->createJointAndBodyNodePair<
            dart::dynamics::FreeJoint, BodyTypeT>(parent,
              static_cast<const dart::dynamics::FreeJoint::Properties&>(
                *jointBuildData.properties), bodyNodeProperties);
        }
        else
        {
          gzerr << "[SkelParser::createJointAndNodePair] Unsupported Joint "
                << "type (" << jointBuildData.type << ") for Joint named ["
                << jointBuildData.properties->mName
                << "]! It will be discarded.\n";
          return std::pair<dart::dynamics::Joint*, dart::dynamics::BodyNode*>(
            NULL, NULL);
        }
      }

      public: static bool createJointAndNodePair(
          dart::dynamics::SkeletonPtr skeleton,
          dart::dynamics::BodyNode* parent,
          const JointBuildData& jointBuildData,
          const BodyNodeBuildData& bodyNodeBuildData)
      {
        std::pair<dart::dynamics::Joint*, dart::dynamics::BodyNode*> pair;
        if (bodyNodeBuildData.type.empty())
        {
          pair = createJointAndNodePair<dart::dynamics::BodyNode>(
            skeleton, parent, jointBuildData,
              static_cast<const dart::dynamics::BodyNode::Properties&>(
                *bodyNodeBuildData.properties));
        }
        else if (std::string("soft") == bodyNodeBuildData.type)
        {
          pair = createJointAndNodePair<dart::dynamics::SoftBodyNode>(
            skeleton, parent, jointBuildData,
              static_cast<const dart::dynamics::SoftBodyNode::Properties&>(
                *bodyNodeBuildData.properties));
        }
        else
        {
          gzerr << "[SkelParser::createJointAndNodePair] Invalid type ("
                << bodyNodeBuildData.type << ") for BodyNode named ["
                << bodyNodeBuildData.properties->mName << "]\n";
          return false;
        }

        if (pair.first == NULL || pair.second == NULL)
          return false;

        dart::dynamics::BodyNode* newBodyNode = pair.second;
        dart::dynamics::Joint* newJoint = pair.first;

        bodyNodeBuildData.dartLink->SetDARTBodyNode(newBodyNode);

        // Gazebo does not create joint for DART FreeJoint
        if (jointBuildData.dartJoint)
          jointBuildData.dartJoint->SetDARTJoint(newJoint);

        return true;
      }

      public: static std::string getDARTJointType(
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
    };
  }
}
#endif
