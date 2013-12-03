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
#include <kdl/chainiksolverpos_nr.hpp>

#include "gazebo/physics/Link.hh"
#include "gazebo/physics/Joint.hh"
#include "AtlasPlugin.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(AtlasPlugin)

/////////////////////////////////////////////////
AtlasPlugin::AtlasPlugin()
{
}

/////////////////////////////////////////////////
AtlasPlugin::~AtlasPlugin()
{
  event::Events::DisconnectWorldUpdateBegin(this->updateConnection);
}

/////////////////////////////////////////////////
void AtlasPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
{
  // Get the world name.
  this->model = _parent;
  this->world = this->model->GetWorld();

  this->boxModel = this->world->GetModel("box_goal");

  this->pinJoint = this->world->GetPhysicsEngine()->CreateJoint(
      "revolute", this->model);

  this->pinJoint->SetModel(this->model);
  this->pinJoint->Load(physics::LinkPtr(), this->model->GetLink("utorso"),
      math::Pose());
  this->pinJoint->SetUpperLimit(0,0);
  this->pinJoint->SetLowerLimit(0,0);
  this->pinJoint->Init();

  this->rightHand = this->model->GetLink("r_hand");
  if (!this->rightHand)
    gzerr << "Unable to get right hand\n";

  physics::JointPtr backJoint = this->model->GetJoint("back_bkz");
  std::cout << "BackBKZ[" << backJoint->GetAngle(0) << "]\n";

  // this->jointController = this->model->GetJointController();

  return;

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&AtlasPlugin::Update, this, _1));


  // r_arm_shy: utorso -> r_clav
  this->rChain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY),
       KDL::Frame(KDL::Vector(0.06441, -0.13866, 0.10718))));

  // r_arm_shx: rclav -> r_scap
  this->rChain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX),
        KDL::Frame(KDL::Vector(0, -0.14035, 0.19609))));

  // r_arm_ely: r_scap -> r_uarm
  this->rChain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY),
        KDL::Frame(KDL::Vector(0, -0.187, 0.016))));

  // r_arm_elx: r_uarm -> r_larm
  this->rChain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX),
        KDL::Frame(KDL::Vector(0, -0.119, 0.00921))));

  // r_arm_wry: r_larm -> r_farm
  this->rChain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY),
        KDL::Frame(KDL::Vector(0, -0.187, -0.00921))));

  // r_arm_wrx: r_farm -> r_hand
  this->rChain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX),
        KDL::Frame(KDL::Vector(0, -0.119, 0.00921))));

  this->rFkSolver = new KDL::ChainFkSolverPos_recursive(
      KDL::ChainFkSolverPos_recursive(this->rChain));

  // Create joint array
  unsigned int nj = this->rChain.getNrOfJoints();
  this->rJointpositions = new KDL::JntArray(nj);

  for(unsigned int i = 0; i < nj; ++i)
  {
    (*this->rJointpositions)(i) = 0.0;
  }

  // Create the frame that will contain the results
  KDL::Frame cartpos;

  // Calculate forward position kinematics
  bool kinematics_status;
  kinematics_status =
    this->rFkSolver->JntToCart(*this->rJointpositions, cartpos);

  KDL::ChainIkSolverVel_pinv iksolverVel(this->rChain);
  KDL::ChainIkSolverPos_NR iksolverPos(this->rChain, *this->rFkSolver, iksolverVel);

  math::Vector3 result(cartpos.p.x(), cartpos.p.y(), cartpos.p.z());
  //result += this->model->GetLink("utorso")->GetWorldPose().pos;
  std::cout << "RESULT[" << result << "]\n";

  this->goalPos.x = 0.7;//0.7;
  this->goalPos.y = -1.2;
  this->goalPos.z = 1.0;

  this->IK();

  /*this->goalPos.x = 0.06;
  this->goalPos.y = -0.8;
  this->goalPos.z = -0.3;

  this->goalPos.x = 0.31191 * -1.0;
  this->goalPos.y = -0.4;
  this->goalPos.z = 1.2;
  */

  std::cout << "UTorsoPos[" << this->model->GetLink("utorso")->GetWorldPose().pos << "]\n";
  std::cout << "RCLAV[" << this->model->GetLink("r_clav")->GetWorldPose().pos << "]\n";

  /*this->goalPos = this->goalPos -
    this->model->GetLink("r_clav")->GetWorldPose().pos +
    math::Vector3(0.06441, -0.13866, 0.10718);
    */

  /*std::cout << "Goal Pos Global[" << this->goalPos << "]\n";
  this->goalPos -= this->model->GetLink("r_clav")->GetWorldPose().pos;

  std::cout << "Goal Pos[" << this->goalPos << "]\n";


  if (kinematics_status>=0)
  {
    std::cout << cartpos << std::endl;
    printf("%s \n","Succes, thanks KDL!");
  }
  else
  {
    printf("%s \n","Error: could not calculate forward kinematics :(");
  }

  KDL::JntArray qResult(this->rChain.getNrOfJoints());
  KDL::Frame destFrame(KDL::Vector(this->goalPos.x, this->goalPos.y, this->goalPos.z));
  int ikResult = iksolverPos.CartToJnt(*this->rJointpositions, destFrame,
      qResult);

  std::cout << "IKResult[" << ikResult << "]\n";
  if (ikResult < 0)
    std::cerr << "Maximum number of iterations reached.";

  for(unsigned int i = 0; i < nj; ++i)
  {
    std::cout << qResult(i) << std::endl;
  }
  for(unsigned int i = 0; i < nj; ++i)
  {
    std::cout << (*this->rJointpositions)(i) << std::endl;
  }
  */

  std::vector<std::string> joints;
  joints.push_back("r_arm_shy");
  joints.push_back("r_arm_shx");
  joints.push_back("r_arm_ely");
  joints.push_back("r_arm_elx");
  joints.push_back("r_arm_wry");
  joints.push_back("r_arm_wrx");

  int i = 0;
  for (std::vector<std::string>::iterator iter = joints.begin();
      iter != joints.end(); ++iter, ++i)
  {
    /*this->jointController->SetPositionPID("atlas::" + *iter,
        common::PID(580, 0.5, 5.2));
    if (ikResult >= 0)
      this->jointController->SetPositionTarget("atlas::" + *iter,
          qResult(i));
    else
      this->jointController->SetPositionTarget("atlas::" + *iter, 0);
      */

    //this->jointController->SetJointPosition("atlas::" + *iter, (*this->rJointpositions)(i));
  }
}

/////////////////////////////////////////////////
void AtlasPlugin::Update(const common::UpdateInfo & /*_info*/)
{
  std::vector<std::string> joints;
  joints.push_back("r_arm_shy");
  joints.push_back("r_arm_shx");
  joints.push_back("r_arm_ely");
  joints.push_back("r_arm_elx");
  joints.push_back("r_arm_wry");
  joints.push_back("r_arm_wrx");

  double diff = 0.001;
    int maxI = 0;
  KDL::Frame cartpos;

  math::Vector3 rPos, plusDelta, minusDelta;
  math::Vector3 basePos = this->model->GetLink("r_clav")->GetWorldPose().pos;

  this->goalPos = this->boxModel->GetWorldPose().pos - basePos;

    for (int i = 5; i >= 0; --i)
    {
      double finalDelta = 0;
      double maxDiff = GZ_DBL_MAX;
      double origAngle = (*this->rJointpositions)(i);

      (*this->rJointpositions)(i) = origAngle+diff;
      this->rFkSolver->JntToCart(*this->rJointpositions, cartpos);
      rPos.Set(cartpos.p.x(), cartpos.p.y(), cartpos.p.z());
      plusDelta =  rPos - this->goalPos;
      double plusLength = plusDelta.GetLength();
      if (plusLength < maxDiff)
      {
        maxDiff = plusLength;
        maxI = i;
        finalDelta = diff;
      }

      (*this->rJointpositions)(i) = origAngle-diff;
      this->rFkSolver->JntToCart(*this->rJointpositions, cartpos);
      rPos.Set(cartpos.p.x(), cartpos.p.y(), cartpos.p.z());
      minusDelta =  rPos - this->goalPos;
      double minusLength = minusDelta.GetLength();
      if (minusLength < maxDiff)
      {
        maxDiff = minusLength;
        maxI = i;
        finalDelta = -diff;
      }

      std::cout << "Joint[" << i << "] Plus[" << plusLength << "] Minus[" << minusLength << "]\n";
      //(*this->rJointpositions)(i) = origAngle;
      (*this->rJointpositions)(i) = origAngle + finalDelta;
      this->jointController->SetJointPosition("atlas::" + joints[i], (*this->rJointpositions)(i));
    }

  return;
/*
  bool kinematics_status;



  unsigned int i = 0;


  printf("Process\n");
  // Calculate the current delta to the goal
  this->rFkSolver->JntToCart(*this->rJointpositions, cartpos);
  math::Vector3 rPos(cartpos.p.x(), cartpos.p.y(), cartpos.p.z());
  math::Vector3 currentDelta = this->goalPos - rPos;

  i = 0;
  for (std::vector<std::string>::iterator iter = joints.begin();
      iter != joints.end(); ++iter, ++i)
  {
    this->rFkSolver->JntToCart(*this->rJointpositions, cartpos);
    math::Vector3 rPos(cartpos.p.x(), cartpos.p.y(), cartpos.p.z());
    math::Vector3 currentDelta = this->goalPos - rPos;

    (*this->rJointpositions)(i) =
      this->model->GetJoint(*iter)->GetAngle(0).Radian() + diff;
    this->rFkSolver->JntToCart(*this->rJointpositions, cartpos);
    math::Vector3 rPosPlus(cartpos.p.x(), cartpos.p.y(), cartpos.p.z());

    (*this->rJointpositions)(i) =
      this->model->GetJoint(*iter)->GetAngle(0).Radian() - diff;
    this->rFkSolver->JntToCart(*this->rJointpositions, cartpos);
    math::Vector3 rPosMinus(cartpos.p.x(), cartpos.p.y(), cartpos.p.z());

    math::Vector3 deltaPlus = this->goalPos - rPosPlus;
    math::Vector3 deltaMinus = this->goalPos - rPosMinus;

    if (deltaPlus.GetLength() < currentDelta.GetLength())
    {
      this->jointController->SetPositionTarget("atlas::" + *iter,
          this->model->GetJoint(*iter)->GetAngle(0).Radian() + diff);

      std::cout << "Plus\n";
      (*this->rJointpositions)(i) =
        this->model->GetJoint(*iter)->GetAngle(0).Radian() + diff;
    }
    else if (deltaMinus.GetLength() < currentDelta.GetLength())
    {
      this->jointController->SetPositionTarget("atlas::" + *iter,
          this->model->GetJoint(*iter)->GetAngle(0).Radian() - diff);
      std::cout << "Minus\n";

      (*this->rJointpositions)(i) =
        this->model->GetJoint(*iter)->GetAngle(0).Radian() - diff;
    }
  }
  */
}

void AtlasPlugin::IK()
{
  std::vector<std::string> joints;
  joints.push_back("r_arm_shy");
  joints.push_back("r_arm_shx");
  joints.push_back("r_arm_ely");
  joints.push_back("r_arm_elx");
  joints.push_back("r_arm_wry");
  joints.push_back("r_arm_wrx");

  std::cout << "---------- IK START ------------\n";
  KDL::Frame cartpos;

  // Get the current end effector position
  this->rFkSolver->JntToCart(*this->rJointpositions, cartpos);
  math::Vector3 rPos(cartpos.p.x(), cartpos.p.y(), cartpos.p.z());

  math::Vector3 basePos = this->model->GetLink("r_clav")->GetWorldPose().pos;

  // COmpute the delta
  math::Vector3 currentDelta = this->goalPos - (rPos+basePos);

  std::cout << "Goal Pos[" << this->goalPos << "]\n";
  std::cout << "Current EPos[" << rPos << "]\n";
  std::cout << "Delta[" << currentDelta << "]\n";


  math::Vector3 plusDelta, minusDelta;
  double diff = 0.001;
  double origLength = currentDelta.GetLength();

  int iters = 0;
  while (origLength > 0.03 && iters < 1000)
  {
    int maxI = 0;
    double finalDelta = 0;

    for (int i = 5; i >= 0; --i)
    {
      double maxDiff = GZ_DBL_MAX;
      double origAngle = (*this->rJointpositions)(i);

      (*this->rJointpositions)(i) = origAngle+diff;
      this->rFkSolver->JntToCart(*this->rJointpositions, cartpos);
      rPos.Set(cartpos.p.x(), cartpos.p.y(), cartpos.p.z());
      plusDelta =  this->goalPos - (rPos+basePos);
      double plusLength = plusDelta.GetLength();
      if (plusLength < maxDiff)
      {
        maxDiff = plusLength;
        maxI = i;
        finalDelta = diff;
      }

      (*this->rJointpositions)(i) = origAngle-diff;
      this->rFkSolver->JntToCart(*this->rJointpositions, cartpos);
      rPos.Set(cartpos.p.x(), cartpos.p.y(), cartpos.p.z());
      minusDelta =  this->goalPos - (rPos+basePos);
      double minusLength = minusDelta.GetLength();
      if (minusLength < maxDiff)
      {
        maxDiff = minusLength;
        maxI = i;
        finalDelta = -diff;
      }

      std::cout << "Joint[" << i << "] Plus[" << plusLength << "] Minus[" << minusLength << "]\n";
      //(*this->rJointpositions)(i) = origAngle;
      (*this->rJointpositions)(maxI) = origAngle + finalDelta;
    }
    //(*this->rJointpositions)(maxI) += finalDelta;
    this->rFkSolver->JntToCart(*this->rJointpositions, cartpos);
    rPos.Set(cartpos.p.x(), cartpos.p.y(), cartpos.p.z());
    currentDelta = this->goalPos - (rPos+basePos);


    origLength = currentDelta.GetLength();

    std::cout << "Final Delta[" << finalDelta << "] I[" << maxI << "] Length[" << origLength << "] RPos[" << rPos << " Global[" << rPos + this->model->GetLink("r_clav")->GetWorldPose().pos << "]\n";
    iters++;
  }

  std::cout << "---------- IK END ------------\n";
}
