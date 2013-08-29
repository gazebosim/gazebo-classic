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
  : Model(_parent), dartSkeleton(NULL),
    dartCanonicalJoint(NULL)
{
}

//////////////////////////////////////////////////
DARTModel::~DARTModel()
{
  if (dartSkeleton)
    delete dartSkeleton;
}

//////////////////////////////////////////////////
void DARTModel::Load(sdf::ElementPtr _sdf)
{
  // create skeletonDynamics of DART
  this->dartSkeleton = new dart::dynamics::Skeleton();

  Model::Load(_sdf);

}

//////////////////////////////////////////////////
void DARTModel::Init()
{
  Model::Init();

  // Name
  std::string modelName = this->GetName();
  this->dartSkeleton->setName(modelName.c_str());

  // Static
  this->dartSkeleton->setImmobileState(this->IsStatic());

  // Check if this link is free floating body
  Link_V linkList = this->GetLinks();

  for (unsigned int i = 0; i < linkList.size(); ++i)
  {
    dart::dynamics::BodyNode* dartBodyNode
        = boost::static_pointer_cast<DARTLink>(linkList[i])->getDARTBodyNode();

    if (dartBodyNode->getParentJoint() == NULL)
    {
      // If this link has no parent joint, then we add 6-dof free joint.
      dart::dynamics::FreeJoint* newFreeJoint = new dart::dynamics::FreeJoint;

      newFreeJoint->setParentBody(NULL);
      newFreeJoint->setTransformFromParentBody(
            DARTUtils::ConvertPose(linkList[i]->GetWorldPose()));

      newFreeJoint->setChildBody(dartBodyNode);
      newFreeJoint->setTransformFromChildBody(dart::math::SE3::Identity());

      this->GetSkeleton()->addJoint(newFreeJoint);
    }
  }

  // add skeleton to world
  this->GetDARTWorld()->addSkeleton(dartSkeleton);
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
//void DARTModel::Reset()
//{
//  Model::Reset();

//}

//////////////////////////////////////////////////
DARTPhysicsPtr DARTModel::GetDARTPhysics(void) const {
  return boost::shared_dynamic_cast<DARTPhysics>(this->GetWorld()->GetPhysicsEngine());
}

//////////////////////////////////////////////////
dart::simulation::World* DARTModel::GetDARTWorld(void) const
{
  return GetDARTPhysics()->GetDARTWorld();
}
