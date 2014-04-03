/*
 * Copyright (C) 2012-2013 Open Source Robotics Foundation
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

#include <kdl/chainiksolver.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolverpos_lma.hpp>

#include "gazebo/physics/Link.hh"
#include "gazebo/physics/Joint.hh"
#include "SimpleIKPlugin.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(SimpleIKPlugin)

/////////////////////////////////////////////////
SimpleIKPlugin::SimpleIKPlugin()
{
  this->iksolverVel = NULL;
}

/////////////////////////////////////////////////
SimpleIKPlugin::~SimpleIKPlugin()
{
  event::Events::DisconnectWorldUpdateBegin(this->updateConnection);
}

/////////////////////////////////////////////////
void SimpleIKPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  this->restart = false;

  // Get the world name.
  this->model = _parent;
  this->world = this->model->GetWorld();

  this->goalModel = this->model->GetLink(_sdf->Get<std::string>("goal_model"));
  this->baseLink = this->model->GetLink(_sdf->Get<std::string>("base_link"));

  //bool useCollisionFrame = _sdf->Get<bool>("use_collision_frame");
  this->useCollisionFrame = true;

  this->jointController = this->model->GetJointController();

  sdf::ElementPtr jointElem = _sdf->GetElement("joint");
  while (jointElem)
  {
    physics::JointPtr joint = this->model->GetJoint(
        jointElem->Get<std::string>());
    this->joints.push_back(joint);

    jointElem = jointElem->GetNextElement("joint");
  }

  math::Vector3 frame;

  math::Pose child, parent;
  unsigned int colIndex = 0;
  /*if (useCollisionFrame
      && joints[0]->GetChild()->GetCollisions().size() > 0
      && this->baseLink->GetCollisions().size() > 0)
  {
    child = joints[0]->GetChild()->GetCollision(colIndex)->GetRelativePose() +
        joints[0]->GetChild()->GetRelativePose();
    parent = this->baseLink->GetCollision(colIndex)->GetRelativePose() +
        this->baseLink->GetRelativePose();
    std::cerr << " child " << joints[0]->GetChild()->GetName()
        << " " << child << std::endl;
    std::cerr << " parent " << this->baseLink->GetName()
        << " " << parent << std::endl;
  }
  else*/
  {

    child = joints[0]->GetChild()->GetRelativePose();
    parent = this->baseLink->GetRelativePose();
  }
  frame = (child-parent).pos;


  this->chain.addSegment(KDL::Segment(
          KDL::Joint(KDL::Joint::None),
          KDL::Frame(KDL::Vector(frame.x, frame.y, frame.z))));

  for (unsigned int i = 0; i < this->joints.size(); ++i)
  {
    if (i < joints.size()-1)
    {
      frame = (joints[i+1]->GetChild()->GetRelativePose() -
              joints[i]->GetChild()->GetRelativePose()).pos;
      /*if (useCollisionFrame
          && (joints[i]->GetChild()->GetCollisions().size() > 0
          || joints[i+1]->GetChild()->GetCollisions().size() > 0))
      {
        if (joints[i+1]->GetChild()->GetCollisions().size() > 0)
          child =
              joints[i+1]->GetChild()->GetCollision(colIndex)->GetRelativePose()
              + joints[i+1]->GetChild()->GetRelativePose();

        if (joints[i]->GetChild()->GetCollisions().size() > 0)
        parent =
            joints[i]->GetChild()->GetCollision(colIndex)->GetRelativePose()
             + joints[i]->GetChild()->GetRelativePose();

        if (joints[i+1]->GetChild()->GetCollisions().size() > 0)
          std::cerr << " child " << joints[i+1]->GetChild()->GetName()
              << " " << child << std::endl;
        if (joints[i]->GetChild()->GetCollisions().size() > 0)
    	    std::cerr << " parent " << joints[i]->GetParent()->GetName()
    	    << " " << parent << std::endl;
      }
      else*/
      {
        child = joints[i+1]->GetChild()->GetRelativePose();
        parent = joints[i]->GetChild()->GetRelativePose();
      }
      frame = (child-parent).pos;
    }
    else
      frame = _sdf->Get<math::Vector3>("ef_pos");


    math::Vector3 axis = joints[i]->GetLocalAxis(0);

    this->chain.addSegment(KDL::Segment(
          KDL::Joint(KDL::Vector(0, 0, 0),
            KDL::Vector(axis.x, axis.y, axis.z), KDL::Joint::RotAxis),
          KDL::Frame(KDL::Vector(frame.x, frame.y, frame.z))));
  }

  this->fkSolver = new KDL::ChainFkSolverPos_recursive(
      KDL::ChainFkSolverPos_recursive(this->chain));

  // Create joint array
  unsigned int nj = this->chain.getNrOfJoints();
  this->jointpositions = new KDL::JntArray(nj);

  for(unsigned int i = 0; i < nj; ++i)
  {
    (*this->jointpositions)(i) = 0;
  }

  // Create the frame that will contain the results
  KDL::Frame cartpos;

  // Calculate forward position kinematics
  this->fkSolver->JntToCart(*this->jointpositions, cartpos);

  math::Vector3 result(cartpos.p.x(), cartpos.p.y(), cartpos.p.z());
  math::Vector3 basePos;

  basePos = this->baseLink->GetRelativePose().pos;
  this->goalPos = this->goalModel->GetRelativePose().pos;
  this->goalPos = this->goalPos - basePos;

  this->iksolverVel = new KDL::ChainIkSolverVel_wdls(this->chain, 0.00001, 200);

  Eigen::Matrix<double,6,1> L;
  L(0)=1;L(1)=1;L(2)=1;
  L(3)=0.01;L(4)=0.01;L(5)=0.01;

  this->iksolverPos = new KDL::ChainIkSolverPos_LMA(this->chain, L);

  KDL::JntArray qResult(this->chain.getNrOfJoints());
  KDL::Frame destFrame(KDL::Vector(this->goalPos.x,
        this->goalPos.y, this->goalPos.z));
  this->iksolverPos->CartToJnt(*this->jointpositions, destFrame, qResult);

  for(unsigned int i = 0; i < nj; ++i)
  {
    this->jointController->SetJointPosition(
        this->joints[i]->GetScopedName(), qResult(i));

    this->jointController->SetPositionPID(this->joints[i]->GetScopedName(),
        common::PID(1220, 1, 25));
  }
  this->fkSolver->JntToCart(qResult, cartpos);

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&SimpleIKPlugin::Update, this, _1));
}

/////////////////////////////////////////////////
void SimpleIKPlugin::Reset()
{
  this->restart = true;
}

/////////////////////////////////////////////////
void SimpleIKPlugin::Update(const common::UpdateInfo & /*_info*/)
{
  if (this->restart)
  {
    math::Vector3 basePos = this->baseLink->GetRelativePose().pos;
    this->goalPos = this->goalModel->GetRelativePose().pos;
    this->goalPos = this->goalPos - basePos;

    for(unsigned int i = 0; i < this->joints.size(); ++i)
    {
      (*this->jointpositions)(i) = 0;
    }

    KDL::JntArray qResult(this->chain.getNrOfJoints());
    KDL::Frame destFrame(KDL::Vector(this->goalPos.x,
          this->goalPos.y, this->goalPos.z));
    this->iksolverPos->CartToJnt(*this->jointpositions, destFrame, qResult);

    this->jointController->Reset();

    for(unsigned int i = 0; i < this->joints.size(); ++i)
    {
      this->jointController->SetJointPosition(
          this->joints[i]->GetScopedName(), qResult(i));
    }

    this->restart = false;
    return;
  }

  unsigned int nj = this->chain.getNrOfJoints();
  KDL::JntArray qResult(nj);

  for(unsigned int i = 0; i < nj; ++i)
  {
    (*jointpositions)(i) =  this->joints[i]->GetAngle(0).Radian();
  }

  this->goalPos = this->goalModel->GetRelativePose().pos -
    this->baseLink->GetRelativePose().pos;

  math::Matrix3 mat = this->goalModel->GetRelativePose().rot.GetAsMatrix3();
  KDL::Rotation kdlRot(mat[0][0],mat[0][1], mat[0][2],
                       mat[1][0],mat[1][1], mat[1][2],
                       mat[2][0],mat[2][1], mat[2][2]);

  KDL::Vector kdlVec(this->goalPos.x, this->goalPos.y, this->goalPos.z);
  KDL::Frame destFrame(kdlRot, kdlVec);

  // if CartToHnt result < 0 then it couldn't find a result.
  // However, this is not very useful since the result will get the end
  // effector close to the goal.
  this->iksolverPos->CartToJnt(*this->jointpositions, destFrame, qResult);

  for(unsigned int i = 0; i < nj; ++i)
  {
    this->jointController->SetPositionTarget(
        this->joints[i]->GetScopedName(), qResult(i));
  }
}
