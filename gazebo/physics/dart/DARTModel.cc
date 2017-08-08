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
  this->dataPtr = nullptr;
}

//////////////////////////////////////////////////
void DARTModel::Load(sdf::ElementPtr _sdf)
{
  if (_sdf->HasElement("model"))
  {
    gzerr << "Nested models are not currently supported in DART. ["
      << _sdf->Get<std::string>("name") << "] will not be loaded. "
      << std::endl;
    this->sdf = _sdf;
    return;
  }

  Model::Load(_sdf);
}

//////////////////////////////////////////////////
void DARTModel::Init()
{
  // nested models are not supported for now, issue #1833
  if (this->sdf->HasElement("model"))
    return;

  GZ_ASSERT(this->dataPtr->dtSkeleton, "Skeleton can't be NULL");

  gzdbg << "Initializing DART model " << this->GetName() << "\n";

  //----------------------------------------------------------------
  // Build DART Skeleton from the list of links and joints
  //
  // NOTE: Below code block will be simplified once DART implements
  // SkeletonBuilder which would play a role similiar to Simbody's
  // MultibodyGraphMaker.
  //----------------------------------------------------------------

  // map for link name -> body node build information
  DARTModelPrivate::BodyNodeMap bodyNodeMap;

  Link_V linkList = this->GetLinks();
  for (auto link : linkList)
  {
    DARTLinkPtr dartLink = boost::dynamic_pointer_cast<DARTLink>(link);
    GZ_ASSERT(dartLink, "DART link is null");

    gzdbg << "Adding to body node map: DART link "
          << dartLink->GetName() << "\n";

    // multiple parent joints are not supported: Each BodyNode has one link
    // and joint, and can have only one parent BodyNode
    // (see also dart::dynamics::Skeleton::createJointAndBodyNodePair).
    if (dartLink->GetParentJoints().size() > 1)
    {
      gzerr << "Multiple parent joints for link " << link->GetName()
            << ". This is not supported in DART. Not loading model. \n";
      Joint_V parents = dartLink->GetParentJoints();
      gzerr << "Parents: \n";
      for (auto joint : parents)
        gzerr << joint->GetName() << "\n";
      return;
    }

    DARTModelPrivate::BodyNodeBuildData bodyNodeBD;
    bodyNodeBD.dartLink = dartLink;
    bodyNodeBD.properties = dartLink->DARTProperties();
    bodyNodeBD.initTransform = DARTTypes::ConvPose(dartLink->WorldPose());
    bodyNodeBD.type = dartLink->IsSoftBody() ? "soft" : "";

    bodyNodeMap[dartLink->GetName()] = bodyNodeBD;
  }

  // map for link name -> parent joint
  DARTModelPrivate::JointMap jointMap;
  Joint_V jointList = this->GetJoints();
  for (auto joint : jointList)
  {
    DARTJointPtr dartJoint = boost::dynamic_pointer_cast<DARTJoint>(joint);
    GZ_ASSERT(dartJoint, "DART joint is null");

    DARTModelPrivate::JointBuildData jointBD;
    jointBD.dartJoint = dartJoint;
    jointBD.properties = dartJoint->DARTProperties();
    if (dartJoint->GetParent())
    {
      jointBD.parentName = dartJoint->GetParent()->GetName();
    }
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

    gzdbg << "Adding to joint map for link " << jointBD.childName
          << ": DART joint " << dartJoint->GetName() << "\n";

    jointMap[jointBD.childName] = jointBD;
  }

  // Iterate through the collected properties and construct the Skeleton from
  // the root nodes downward.
  DARTModelPrivate::BodyNodeMap::const_iterator bodyNodeItr =
      bodyNodeMap.begin();
  DARTModelPrivate::JointMap::const_iterator parentJointItr;
  dart::dynamics::BodyNode* dtParentBodyNode = nullptr;

  while (bodyNodeItr != bodyNodeMap.end())
  {
    GZ_ASSERT(bodyNodeItr->second.dartLink, "Link is nullptr");
    // multiple parent joints are not supported (this case should have
    // been caught before, therefore we make the assertion here).
    // Should this be supported in future versions of DART, we have to make
    // sure here that body nodes are created for all links. The
    // current approach with this loop implementation only considers each
    // link once (and therefore supports only one parent joint of it).
    GZ_ASSERT(bodyNodeItr->second.dartLink->GetParentJoints().size() <= 1,
              "Multiple parent joints for links are not supported in DART");

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
      // If a root FreeJoint is needed for the parent of the current joint,
      // then create it
      DARTModelPrivate::JointBuildData rootJoint;
      rootJoint.properties =
          Eigen::make_aligned_shared<dart::dynamics::FreeJoint::Properties>(
            dart::dynamics::Joint::Properties(
              "root", bodyNodeItr->second.initTransform));
      rootJoint.type = "free";

      gzdbg << "Building DART BodyNode for link "
            << bodyNodeItr->second.dartLink->GetName()
            << " with a free joint.\n";

      if (!DARTModelPrivate::createJointAndNodePair(
            this->dataPtr->dtSkeleton, nullptr, rootJoint, bodyNodeItr->second))
      {
        gzdbg << "Could not create joint and node.\n";
        break;
      }

      bodyNodeMap.erase(bodyNodeItr);
      bodyNodeItr = bodyNodeMap.begin();

      continue;
    }

    gzdbg << "Building DART BodyNode for link "
            << bodyNodeItr->second.dartLink->GetName()
            << "and joint "
            << parentJointItr->second.properties->mName << ".\n";

    if (!DARTModelPrivate::createJointAndNodePair(
          this->dataPtr->dtSkeleton,
          dtParentBodyNode,
          parentJointItr->second,
          bodyNodeItr->second))
    {
      gzdbg << "Could not create joint and node.\n";
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
    this->dataPtr->dtSkeleton->enableSelfCollisionCheck();
    this->dataPtr->dtSkeleton->setAdjacentBodyCheck(false);
  }

  // Note: This function should be called after the skeleton is added to the
  //       world.
  this->BackupState();

  // Add the skeleton to the world
  this->DARTWorld()->addSkeleton(this->dataPtr->dtSkeleton);
}


//////////////////////////////////////////////////
void DARTModel::Update()
{
  Model::Update();
}

//////////////////////////////////////////////////
void DARTModel::Fini()
{
  // get a backup of the world, because Model::Fini() (eventually
  // calling Base::Fini()) will reset the world pointer
  dart::simulation::WorldPtr _world = this->DARTWorld();
  // remove all links and joints properly
  Model::Fini();
  // remove the skeleton from the world
  if (_world && this->dataPtr->dtSkeleton)
  {
    _world->removeSkeleton(this->dataPtr->dtSkeleton);
  }
}

//////////////////////////////////////////////////
void DARTModel::BackupState()
{
  GZ_ASSERT(this->dataPtr->dtSkeleton, "Skeleton can't be NULL");
  this->dataPtr->genPositions = this->dataPtr->dtSkeleton->getPositions();
  this->dataPtr->genVelocities = this->dataPtr->dtSkeleton->getVelocities();
}

//////////////////////////////////////////////////
void DARTModel::RestoreState()
{
  GZ_ASSERT(this->dataPtr->dtSkeleton, "Skeleton can't be NULL");

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
dart::dynamics::SkeletonPtr DARTModel::DARTSkeleton()
{
  return this->dataPtr->dtSkeleton;
}

//////////////////////////////////////////////////
DARTPhysicsPtr DARTModel::GetDARTPhysics(void) const
{
  if (!this->GetWorld()) return nullptr;
  return boost::dynamic_pointer_cast<DARTPhysics>(
    this->GetWorld()->Physics());
}

//////////////////////////////////////////////////
dart::simulation::WorldPtr DARTModel::DARTWorld(void) const
{
  DARTPhysicsPtr physics = GetDARTPhysics();
  if (!physics) return nullptr;
  return physics->DARTWorld();
}
