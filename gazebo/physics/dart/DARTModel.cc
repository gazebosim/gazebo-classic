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

#include "gazebo/physics/World.hh"

#include "gazebo/physics/dart/DARTPhysics.hh"
#include "gazebo/physics/dart/DARTLink.hh"
#include "gazebo/physics/dart/DARTModel.hh"
#include "gazebo/physics/dart/DARTUtils.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
DARTModel::DARTModel(BasePtr _parent)
  : Model(_parent), dtSkeleton(NULL)
{
}

//////////////////////////////////////////////////
DARTModel::~DARTModel()
{
  if (dtSkeleton)
    delete dtSkeleton;
}

//////////////////////////////////////////////////
void DARTModel::Load(sdf::ElementPtr _sdf)
{
  // create skeletonDynamics of DART
  this->dtSkeleton = new dart::dynamics::Skeleton();

  Model::Load(_sdf);

}

//////////////////////////////////////////////////
void DARTModel::Init()
{
  Model::Init();

  // Name
  std::string modelName = this->GetName();
  this->dtSkeleton->setName(modelName.c_str());

  // Static
  this->dtSkeleton->setMobile(!this->IsStatic());

  // Check if this link is free floating body
  Link_V linkList = this->GetLinks();

  for (unsigned int i = 0; i < linkList.size(); ++i)
  {
    dart::dynamics::BodyNode* dtBodyNode
        = boost::static_pointer_cast<DARTLink>(linkList[i])->getDARTBodyNode();

    if (dtBodyNode->getParentJoint() == NULL)
    {
      // If this link has no parent joint, then we add 6-dof free joint to the
      // link.
      dart::dynamics::FreeJoint* newFreeJoint = new dart::dynamics::FreeJoint;

      newFreeJoint->setTransformFromParentBodyNode(
            DARTTypes::ConvPose(linkList[i]->GetWorldPose()));
      newFreeJoint->setTransformFromChildBodyNode(Eigen::Isometry3d::Identity());

      dtBodyNode->setParentJoint(newFreeJoint);
    }

    dtSkeleton->addBodyNode(dtBodyNode);
  }

  // add skeleton to world
  this->GetDARTWorld()->addSkeleton(dtSkeleton);
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
dart::dynamics::Skeleton*DARTModel::GetDARTSkeleton()
{
  return dtSkeleton;
}

//////////////////////////////////////////////////
DARTPhysicsPtr DARTModel::GetDARTPhysics(void) const {
  return boost::shared_dynamic_cast<DARTPhysics>(this->GetWorld()->GetPhysicsEngine());
}

//////////////////////////////////////////////////
dart::simulation::World* DARTModel::GetDARTWorld(void) const
{
  return GetDARTPhysics()->GetDARTWorld();
}
