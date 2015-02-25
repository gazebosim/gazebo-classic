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

#include "gazebo/physics/rtql8/RTQL8Physics.hh"
#include "gazebo/physics/rtql8/RTQL8Link.hh"
#include "gazebo/physics/rtql8/RTQL8Model.hh"
#include "gazebo/physics/rtql8/RTQL8Utils.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
RTQL8Model::RTQL8Model(BasePtr _parent)
  : Model(_parent), rtql8SkeletonDynamics(NULL),
    rtql8CanonicalJoint(NULL)
{
  
}

//////////////////////////////////////////////////
RTQL8Model::~RTQL8Model()
{
  if (rtql8SkeletonDynamics)
    delete rtql8SkeletonDynamics;
}

//////////////////////////////////////////////////
void RTQL8Model::Load(sdf::ElementPtr _sdf)
{
  // create skeletonDynamics of RTQL8
  this->rtql8SkeletonDynamics = new rtql8::dynamics::SkeletonDynamics();

  // add skeleton to world
  //this->GetRTQL8World()->addSkeleton(rtql8SkeletonDynamics);

  Model::Load(_sdf);

  //  if (this->IsStatic())
  //    rtql8SkeletonDynamics->setImmobileState(true);
  //  else
  //    rtql8SkeletonDynamics->setImmobileState(false);
  rtql8SkeletonDynamics->setImmobileState(this->IsStatic());
}

//////////////////////////////////////////////////
void RTQL8Model::Init()
{
  Model::Init();

  // RTQL8 must have a joint that connects the canonnical link (body node) and
  // the world (space). Let's call is a this joint as a cannonical joint.
  // However, in gazebo, this joint is optional. If the canonnical link is free
  // floating in the world, then we don't need a cannonical joint. Otherwise,
  // if the canonnical link is connected to the world by a joint such as hinge
  // joint, ball joint, and so on, then we need the joint.
  //
  // Therefore, we need to check whether this model has a cannonical joint. If
  // there is no, then we must create a cannonical joint here even though the
  // joint is not described in the sdf file.
  if (rtql8CanonicalJoint == 0)
  {
    // TODO: need to access to Model::canonicalLink
    //       the member is private for now. this should be protected.
    LinkPtr canonicalLink_ = this->GetLink("canonical");

    rtql8::kinematics::BodyNode* parentBodyNode = NULL;
    rtql8::kinematics::BodyNode* childBodyNode
        = boost::shared_dynamic_cast<RTQL8Link>(canonicalLink_)->GetBodyNode();

    this->rtql8CanonicalJoint
        = new rtql8::kinematics::Joint(parentBodyNode, childBodyNode);
    this->rtql8SkeletonDynamics->addJoint(rtql8CanonicalJoint);

    //---- Step 1. Transformation from rotated joint frame to child link frame.
    math::Pose canonicalLinkPose = canonicalLink_->GetWorldPose();
//    RTQL8Utils::addTransformToRTQL8Joint(this->rtql8CanonicalJoint,
//                                         canonicalLinkPose);

    //---- Step 2. Transformation by the rotate axis.
    rtql8::kinematics::Dof* tranX = new rtql8::kinematics::Dof(0, -10000, 10000);
    rtql8::kinematics::Dof* tranY = new rtql8::kinematics::Dof(0, -10000, 10000);
    rtql8::kinematics::Dof* tranZ = new rtql8::kinematics::Dof(0, -10000, 10000);
//    rtql8::kinematics::Dof* rotX = new rtql8::kinematics::Dof(0, -6.1416, 6.1416);
//    rtql8::kinematics::Dof* rotY = new rtql8::kinematics::Dof(0, -6.1416, 6.1416);
//    rtql8::kinematics::Dof* rotZ = new rtql8::kinematics::Dof(0, -6.1416, 6.1416);
    rtql8::kinematics::Dof* rotX = new rtql8::kinematics::Dof(0, -60.1416, 60.1416);
    rtql8::kinematics::Dof* rotY = new rtql8::kinematics::Dof(0, -60.1416, 60.1416);
    rtql8::kinematics::Dof* rotZ = new rtql8::kinematics::Dof(0, -60.1416, 60.1416);

    rtql8::kinematics::TrfmTranslate* trfmTranslateCanonical
        = new rtql8::kinematics::TrfmTranslate(tranX, tranY, tranZ);
    rtql8::kinematics::TrfmRotateExpMap* trfmRotateCanonical
        = new rtql8::kinematics::TrfmRotateExpMap(rotX, rotY, rotZ);

    // Set the initial pose (transformation) of bodies.
    tranX->setValue(canonicalLinkPose.pos.x);
    tranY->setValue(canonicalLinkPose.pos.y);
    tranZ->setValue(canonicalLinkPose.pos.z);

    Eigen::Quaterniond eigenQuat(canonicalLinkPose.rot.w,
                                canonicalLinkPose.rot.x,
                                canonicalLinkPose.rot.y,
                                canonicalLinkPose.rot.z);

    //Eigen::Quaterniond expToQuat(Eigen::Vector3d& v);
    Eigen::Vector3d eigenVec3 = rtql8::utils::rotation::quatToExp(eigenQuat);

    rotX->setValue(eigenVec3(0));
    rotY->setValue(eigenVec3(1));
    rotZ->setValue(eigenVec3(2));

    // Get the model associated with
    // Add the transform to the skeletone in the model.
    // add to model because it's variable
    this->rtql8CanonicalJoint->addTransform(trfmTranslateCanonical, true);
    this->rtql8CanonicalJoint->addTransform(trfmRotateCanonical, true);

    this->GetSkeletonDynamics()->addTransform(trfmTranslateCanonical);
    this->GetSkeletonDynamics()->addTransform(trfmRotateCanonical);
  }

  // init the kinematics and dynamics
  rtql8SkeletonDynamics->initSkel();
  rtql8SkeletonDynamics->initDynamics();

  // add skeleton to world
  this->GetRTQL8World()->addSkeleton(rtql8SkeletonDynamics);
}


//////////////////////////////////////////////////
void RTQL8Model::Update()
{
  Model::Update();
  
}

//////////////////////////////////////////////////
void RTQL8Model::Fini()
{
  Model::Fini();
  
}

//////////////////////////////////////////////////
//void RTQL8Model::Reset()
//{
//  Model::Reset();

//}

//////////////////////////////////////////////////
RTQL8PhysicsPtr RTQL8Model::GetRTQL8Physics(void) const {
  return boost::shared_dynamic_cast<RTQL8Physics>(this->GetWorld()->GetPhysicsEngine());
}

//////////////////////////////////////////////////
rtql8::simulation::World* RTQL8Model::GetRTQL8World(void) const
{
  return GetRTQL8Physics()->GetRTQL8World();
}
