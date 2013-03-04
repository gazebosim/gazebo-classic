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
  rtql8SkeletonDynamics = new rtql8::dynamics::SkeletonDynamics();

  // add skeleton to world
  this->GetRTQL8World()->addSkeleton(rtql8SkeletonDynamics);

  Model::Load(_sdf);

  // TODO: This should be set by sdf.
  rtql8SkeletonDynamics->setImmobileState(false);
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
        = boost::shared_dynamic_cast<RTQL8Link>(this->GetLink("canonical"))->GetBodyNode();

    rtql8CanonicalJoint
        = new rtql8::kinematics::Joint(parentBodyNode, childBodyNode);

    this->rtql8SkeletonDynamics->addJoint(rtql8CanonicalJoint);






    //---- Step 2. Transformation by the rotate axis.
    rtql8::kinematics::Dof* tranX = new rtql8::kinematics::Dof(0, -10, 10);
    rtql8::kinematics::Dof* tranY = new rtql8::kinematics::Dof(0, -10, 10);
    rtql8::kinematics::Dof* tranZ = new rtql8::kinematics::Dof(0, -10, 10);

    rtql8::kinematics::TrfmTranslate* trfmCanonicalLink
        = new rtql8::kinematics::TrfmTranslate(tranX, tranY, tranZ);

    //
    rtql8CanonicalJoint->addTransform(trfmCanonicalLink, true);

    // Get the model associated with
    // Add the transform to the skeletone in the model.
    // add to model because it's variable
  //    boost::shared_dynamic_cast<RTQL8Model>(this->model)->GetSkeletonDynamics()->addTransform(rotHinge);
    this->GetSkeletonDynamics()->addTransform(trfmCanonicalLink);

    rtql8::kinematics::Dof* rotX = new rtql8::kinematics::Dof(0, -3.1416, 3.1416);
    rtql8::kinematics::Dof* rotY = new rtql8::kinematics::Dof(0, -6.1416, 6.1416);
    rtql8::kinematics::Dof* rotZ = new rtql8::kinematics::Dof(0, -3.1416, 3.1416);

    rtql8::kinematics::TrfmTranslate* trfmRotate
        = new rtql8::kinematics::TrfmTranslate(rotX, rotY, rotZ);

    rtql8CanonicalJoint->addTransform(trfmRotate, true);
    this->GetSkeletonDynamics()->addTransform(trfmRotate);



    //---- Step 3. Transformation from rotated joint frame to child link frame.
    math::Pose canonicalLinkPose = canonicalLink_->GetWorldPose();

    rtql8::kinematics::Dof* tranJ2CL_X = new rtql8::kinematics::Dof(canonicalLinkPose.pos.x);
    rtql8::kinematics::Dof* tranJ2CL_Y = new rtql8::kinematics::Dof(canonicalLinkPose.pos.y);
    rtql8::kinematics::Dof* tranJ2CL_Z = new rtql8::kinematics::Dof(canonicalLinkPose.pos.z);

    rtql8::kinematics::TrfmTranslate* tranJ2CL
        = new rtql8::kinematics::TrfmTranslate(tranJ2CL_X, tranJ2CL_Y, tranJ2CL_Z);

    rtql8CanonicalJoint->addTransform(tranJ2CL, false);

    rtql8::kinematics::Dof* rotJ2CL_W = new rtql8::kinematics::Dof(canonicalLinkPose.rot.w);
    rtql8::kinematics::Dof* rotJ2CL_X = new rtql8::kinematics::Dof(canonicalLinkPose.rot.x);
    rtql8::kinematics::Dof* rotJ2CL_Y = new rtql8::kinematics::Dof(canonicalLinkPose.rot.y);
    rtql8::kinematics::Dof* rotJ2CL_Z = new rtql8::kinematics::Dof(canonicalLinkPose.rot.z);

    rtql8::kinematics::TrfmRotateQuat* rotJ2CL
        = new rtql8::kinematics::TrfmRotateQuat(rotJ2CL_W, rotJ2CL_X, rotJ2CL_Y, rotJ2CL_Z);

    rtql8CanonicalJoint->addTransform(rotJ2CL, false);
  }

  rtql8SkeletonDynamics->initSkel();
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
RTQL8PhysicsPtr RTQL8Model::GetRTQL8Physics(void) const {
  return boost::shared_dynamic_cast<RTQL8Physics>(this->GetWorld()->GetPhysicsEngine());
}

//////////////////////////////////////////////////
rtql8::simulation::World* RTQL8Model::GetRTQL8World(void) const
{
  return GetRTQL8Physics()->GetRTQL8World();
}
