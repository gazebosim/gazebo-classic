/*
 * Copyright (C) 2014-2015 Open Source Robotics Foundation
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

#include "gazebo/common/Assert.hh"

#include "gazebo/physics/World.hh"

#include "gazebo/physics/dart/DARTPhysics.hh"
#include "gazebo/physics/dart/DARTJoint.hh"
#include "gazebo/physics/dart/DARTLink.hh"
#include "gazebo/physics/dart/DARTModel.hh"
#include "gazebo/physics/dart/DARTModelPrivate.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
DARTModel::DARTModel(BasePtr _parent)
  : Model(_parent), dataPtr(new DARTModelPrivate())
{
}

//////////////////////////////////////////////////
DARTModel::~DARTModel()
{
  delete this->dataPtr;
}

//////////////////////////////////////////////////
void DARTModel::Load(sdf::ElementPtr _sdf)
{
  Model::Load(_sdf);
}

//////////////////////////////////////////////////
void DARTModel::Init()
{
  //----------------------------------------------------------------
  // Build DART Skeleton from the list of links and joints
  //
  // NOTE: Below code block will be simplified once DART implements
  // SkeletonBuilder which would play a role similiar to Simbody's
  // MultibodyGraphMaker.
  //----------------------------------------------------------------

  DARTModelPrivate::BodyNodeMap bodyNodeMap;
  DARTModelPrivate::JointMap jointMap;

  Link_V linkList = this->GetLinks();
  for (auto link : linkList)
  {
    DARTLinkPtr dartLink = boost::dynamic_pointer_cast<DARTLink>(link);

    DARTModelPrivate::BodyNodeBuildData bodyNodeBD;
    bodyNodeBD.dartLink = dartLink;
    bodyNodeBD.properties = dartLink->GetDARTProperties();
    bodyNodeBD.initTransform = DARTTypes::ConvPose(dartLink->GetWorldPose());
    bodyNodeBD.type = dartLink->IsSoftBody() ? "soft" : "";

    bodyNodeMap[dartLink->GetName()] = bodyNodeBD;
  }

  Joint_V jointList = this->GetJoints();
  for (auto joint : jointList)
  {
    DARTJointPtr dartJoint = boost::dynamic_pointer_cast<DARTJoint>(joint);

    DARTModelPrivate::JointBuildData jointBD;
    jointBD.dartJoint = dartJoint;
    jointBD.properties = dartJoint->GetDARTProperties();
    if (dartJoint->GetParent())
      jointBD.parentName = dartJoint->GetParent()->GetName();
    if (dartJoint->GetChild())
    {
      jointBD.childName = dartJoint->GetChild()->GetName();
    }
    else
    {
      gzerr << "DART does not allow joint without child link. "
            << "Please see issue #914. "
            << "(https://bitbucket.org/osrf/gazebo/issue/914)"
            << std::endl;
    }
    jointBD.type = DARTModelPrivate::getDARTJointType(dartJoint);

    jointMap[jointBD.childName] = jointBD;
  }

  // Iterate through the collected properties and construct the Skeleton from
  // the root nodes downward the root nodes downward.
  DARTModelPrivate::BodyNodeMap::const_iterator bodyNodeItr =
      bodyNodeMap.begin();
  DARTModelPrivate::JointMap::const_iterator parentJointItr;
  dart::dynamics::BodyNode* dtParentBodyNode = NULL;

  while (bodyNodeItr != bodyNodeMap.end())
  {
    DARTModelPrivate::NextResult result =
        DARTModelPrivate::getNextJointAndNodePair(
          bodyNodeItr, parentJointItr, dtParentBodyNode,
          this->dataPtr->dtSkeleton, bodyNodeMap, jointMap);

    if (DARTModelPrivate::BREAK == result)
    {
      break;
    }
    else if (DARTModelPrivate::CONTINUE == result)
    {
      // Create the parent before creating the current Joint
      continue;
    }
    else if (DARTModelPrivate::CREATE_FREEJOINT_ROOT == result)
    {
      // If a root FreeJoint is needed for the parent of the current joint, then
      // create it
      DARTModelPrivate::JointBuildData rootJoint;
      rootJoint.properties =
          Eigen::make_aligned_shared<dart::dynamics::FreeJoint::Properties>(
            dart::dynamics::Joint::Properties(
              "root", bodyNodeItr->second.initTransform));
      rootJoint.type = "free";

      if (!DARTModelPrivate::createJointAndNodePair(
            this->dataPtr->dtSkeleton, NULL, rootJoint, bodyNodeItr->second))
      {
        break;
      }

      bodyNodeMap.erase(bodyNodeItr);
      bodyNodeItr = bodyNodeMap.begin();

      continue;
    }

    if (!DARTModelPrivate::createJointAndNodePair(
          this->dataPtr->dtSkeleton,
          dtParentBodyNode,
          parentJointItr->second,
          bodyNodeItr->second))
    {
      break;
    }

    bodyNodeMap.erase(bodyNodeItr);
    bodyNodeItr = bodyNodeMap.begin();
  }

  Model::Init();

  //----------------------------------------------
  // Name
  this->dataPtr->dtSkeleton->setName(this->GetName());

  //----------------------------------------------
  // Static
  this->dataPtr->dtSkeleton->setMobile(!this->IsStatic());

  // Self collision
  // Note: This process should be done after this skeleton is added to the
  //       world.

  // Check whether there exist at least one pair of self collidable links.
  int numSelfCollidableLinks = 0;
  bool hasPairOfSelfCollidableLinks = false;
  for (auto link : linkList)
  {
    if (link->GetSelfCollide())
    {
      ++numSelfCollidableLinks;
      if (numSelfCollidableLinks >= 2)
      {
        hasPairOfSelfCollidableLinks = true;
        break;
      }
    }
  }

  // If the skeleton has at least two self collidable links, then we set the
  // skeleton as self collidable. If the skeleton is self collidable, then
  // DART regards that all the links in the skeleton is self collidable. So, we
  // disable all the pairs of which both of the links in the pair is not self
  // collidable.
  if (hasPairOfSelfCollidableLinks)
  {
    this->dataPtr->dtSkeleton->enableSelfCollision();

    dart::simulation::WorldPtr dtWorld =
        this->GetDARTPhysics()->GetDARTWorldPtr();
    dart::collision::CollisionDetector *dtCollDet =
        dtWorld->getConstraintSolver()->getCollisionDetector();

    for (size_t i = 0; i < linkList.size() - 1; ++i)
    {
      for (size_t j = i + 1; j < linkList.size(); ++j)
      {
        dart::dynamics::BodyNode *itdtBodyNode1 =
          boost::dynamic_pointer_cast<DARTLink>(linkList[i])->GetDARTBodyNode();
        dart::dynamics::BodyNode *itdtBodyNode2 =
          boost::dynamic_pointer_cast<DARTLink>(linkList[j])->GetDARTBodyNode();

        // If this->dtBodyNode and itdtBodyNode are connected then don't enable
        // the pair.
        // Please see: https://bitbucket.org/osrf/gazebo/issue/899
        if ((itdtBodyNode1->getParentBodyNode() == itdtBodyNode2) ||
            itdtBodyNode2->getParentBodyNode() == itdtBodyNode1)
        {
          dtCollDet->disablePair(itdtBodyNode1, itdtBodyNode2);
        }

        if (!linkList[i]->GetSelfCollide() || !linkList[j]->GetSelfCollide())
        {
          dtCollDet->disablePair(itdtBodyNode1, itdtBodyNode2);
        }
      }
    }
  }

  // Note: This function should be called after the skeleton is added to the
  //       world.
  this->BackupState();

  // Add the skeleton to the world
  this->GetDARTWorldPtr()->addSkeleton(this->dataPtr->dtSkeleton);
}


//////////////////////////////////////////////////
void DARTModel::Update()
{
  Model::Update();
}

//////////////////////////////////////////////////
void DARTModel::Fini()
{
  Model::Fini();
}

//////////////////////////////////////////////////
void DARTModel::BackupState()
{
  this->dataPtr->genPositions = this->dataPtr->dtSkeleton->getPositions();
  this->dataPtr->genVelocities = this->dataPtr->dtSkeleton->getVelocities();
}

//////////////////////////////////////////////////
void DARTModel::RestoreState()
{
  GZ_ASSERT(static_cast<size_t>(this->dataPtr->genPositions.size()) ==
            this->dataPtr->dtSkeleton->getNumDofs(),
            "Cannot RestoreState, invalid size");
  GZ_ASSERT(static_cast<size_t>(this->dataPtr->genVelocities.size()) ==
            this->dataPtr->dtSkeleton->getNumDofs(),
            "Cannot RestoreState, invalid size");

  this->dataPtr->dtSkeleton->setPositions(this->dataPtr->genPositions);
  this->dataPtr->dtSkeleton->setVelocities(this->dataPtr->genVelocities);
}

//////////////////////////////////////////////////
dart::dynamics::Skeleton *DARTModel::GetDARTSkeleton()
{
  return this->GetDARTSkeletonPtr().get();
}

//////////////////////////////////////////////////
dart::dynamics::SkeletonPtr DARTModel::GetDARTSkeletonPtr()
{
  return this->dataPtr->dtSkeleton;
}

//////////////////////////////////////////////////
DARTPhysicsPtr DARTModel::GetDARTPhysics(void) const
{
  return boost::dynamic_pointer_cast<DARTPhysics>(
    this->GetWorld()->GetPhysicsEngine());
}

//////////////////////////////////////////////////
dart::simulation::World *DARTModel::GetDARTWorld(void) const
{
  return GetDARTPhysics()->GetDARTWorldPtr().get();
}

//////////////////////////////////////////////////
dart::simulation::WorldPtr DARTModel::GetDARTWorldPtr(void) const
{
  return GetDARTPhysics()->GetDARTWorldPtr();
}
