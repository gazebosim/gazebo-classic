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
  : Model(_parent), dartSkeletonDynamics(NULL),
    dartCanonicalJoint(NULL)
{
}

//////////////////////////////////////////////////
DARTModel::~DARTModel()
{
  if (dartSkeletonDynamics)
    delete dartSkeletonDynamics;
}

//////////////////////////////////////////////////
void DARTModel::Load(sdf::ElementPtr _sdf)
{
  // create skeletonDynamics of DART
  this->dartSkeletonDynamics = new dynamics::SkeletonDynamics();

  Model::Load(_sdf);

  // Name
  std::string modelName = this->GetName();
  this->dartSkeletonDynamics->setName(modelName.c_str());

  //  if (this->IsStatic())
  //    dartSkeletonDynamics->setImmobileState(true);
  //  else
  //    dartSkeletonDynamics->setImmobileState(false);
  dartSkeletonDynamics->setImmobileState(this->IsStatic());
}

//////////////////////////////////////////////////
void DARTModel::Init()
{
  Model::Init();

  // DART must have a joint that connects the canonnical link (body node) and
  // the world (space). Let's call is a this joint as a cannonical joint.
  // However, in gazebo, this joint is optional. If the canonnical link is free
  // floating in the world, then we don't need a cannonical joint. Otherwise,
  // if the canonnical link is connected to the world by a joint such as hinge
  // joint, ball joint, and so on, then we need the joint.
  //
  // Therefore, we need to check whether this model has a cannonical joint. If
  // there is no, then we must create a cannonical joint here even though the
  // joint is not described in the sdf file.
//  if (dartCanonicalJoint == 0)
//  {
//    // TODO: need to access to Model::canonicalLink
//    //       the member is private for now. this should be protected.
//    LinkPtr canonicalLink_ = this->GetLink("canonical");

//    kinematics::BodyNode* parentBodyNode = NULL;
//    kinematics::BodyNode* childBodyNode
//        = boost::shared_dynamic_cast<DARTLink>(canonicalLink_)->GetBodyNode();

//    this->dartCanonicalJoint
//        = new kinematics::Joint(parentBodyNode, childBodyNode);
//    this->dartSkeletonDynamics->addJoint(dartCanonicalJoint);

//    //---- Step 1. Transformation from rotated joint frame to child link frame.
//    math::Pose canonicalLinkPose = canonicalLink_->GetWorldPose();
////    DARTUtils::addTransformToDARTJoint(this->dartCanonicalJoint,
////                                         canonicalLinkPose);

//    //---- Step 2. Transformation by the rotate axis.
//    kinematics::Dof* tranX = new kinematics::Dof(0, -10000, 10000);
//    kinematics::Dof* tranY = new kinematics::Dof(0, -10000, 10000);
//    kinematics::Dof* tranZ = new kinematics::Dof(0, -10000, 10000);
////    kinematics::Dof* rotX = new kinematics::Dof(0, -6.1416, 6.1416);
////    kinematics::Dof* rotY = new kinematics::Dof(0, -6.1416, 6.1416);
////    kinematics::Dof* rotZ = new kinematics::Dof(0, -6.1416, 6.1416);
//    kinematics::Dof* rotX = new kinematics::Dof(0, -60.1416, 60.1416);
//    kinematics::Dof* rotY = new kinematics::Dof(0, -60.1416, 60.1416);
//    kinematics::Dof* rotZ = new kinematics::Dof(0, -60.1416, 60.1416);

//    kinematics::TrfmTranslate* trfmTranslateCanonical
//        = new kinematics::TrfmTranslate(tranX, tranY, tranZ);
//    kinematics::TrfmRotateExpMap* trfmRotateCanonical
//        = new kinematics::TrfmRotateExpMap(rotX, rotY, rotZ);

//    // Set the initial pose (transformation) of bodies.
//    tranX->setValue(canonicalLinkPose.pos.x);
//    tranY->setValue(canonicalLinkPose.pos.y);
//    tranZ->setValue(canonicalLinkPose.pos.z);

//    Eigen::Quaterniond eigenQuat(canonicalLinkPose.rot.w,
//                                canonicalLinkPose.rot.x,
//                                canonicalLinkPose.rot.y,
//                                canonicalLinkPose.rot.z);

//    //Eigen::Quaterniond expToQuat(Eigen::Vector3d& v);
//    Eigen::Vector3d eigenVec3 = dart_math::quatToExp(eigenQuat);

//    rotX->setValue(eigenVec3(0));
//    rotY->setValue(eigenVec3(1));
//    rotZ->setValue(eigenVec3(2));

//    // Get the model associated with
//    // Add the transform to the skeletone in the model.
//    // add to model because it's variable
//    this->dartCanonicalJoint->addTransform(trfmTranslateCanonical, true);
//    this->dartCanonicalJoint->addTransform(trfmRotateCanonical, true);

//    this->GetSkeletonDynamics()->addTransform(trfmTranslateCanonical);
//    this->GetSkeletonDynamics()->addTransform(trfmRotateCanonical);
//  }

  // init the kinematics and dynamics
  dartSkeletonDynamics->initSkel();
  //dartSkeletonDynamics->initDynamics();

  // add skeleton to world
  this->GetDARTWorld()->addSkeleton(dartSkeletonDynamics);
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
simulation::World* DARTModel::GetDARTWorld(void) const
{
  return GetDARTPhysics()->GetDARTWorld();
}
