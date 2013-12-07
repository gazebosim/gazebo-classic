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
  // Get the world name.
  this->model = _parent;
  this->world = this->model->GetWorld();

  this->goalModel = this->world->GetModel(_sdf->Get<std::string>("goal_model"));
  this->baseLink = this->model->GetLink(_sdf->Get<std::string>("base_link"));

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
  frame = (joints[0]->GetChild()->GetWorldPose() -
           this->baseLink->GetWorldPose()).pos;

  this->chain.addSegment(KDL::Segment(
          KDL::Joint(KDL::Joint::None),
          KDL::Frame(KDL::Vector(frame.x, frame.y, frame.z))));

  for (unsigned int i = 0; i < this->joints.size(); ++i)
  {
    math::Vector3 frame;
    if (i < joints.size()-1)
    {
      frame = (joints[i+1]->GetChild()->GetWorldPose() -
              joints[i]->GetChild()->GetWorldPose()).pos;
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


//);
  math::Vector3 basePos = this->baseLink->GetWorldPose().pos;

  this->goalPos = this->goalModel->GetWorldPose().pos;

  this->goalPos = this->goalPos - basePos;

  std::cout << "Diff[" <<  this->goalPos << "]\n";

  this->iksolverVel = new KDL::ChainIkSolverVel_wdls(this->chain, 0.00001, 200);

  Eigen::Matrix<double,6,1> L;
  L(0)=1;L(1)=1;L(2)=1;
  L(3)=0.01;L(4)=0.01;L(5)=0.01;

  KDL::ChainIkSolverPos_LMA iksolverPos(this->chain, L);

  KDL::JntArray qResult(this->chain.getNrOfJoints());
  KDL::Frame destFrame(KDL::Vector(this->goalPos.x,
        this->goalPos.y, this->goalPos.z));
  int ikResult = iksolverPos.CartToJnt(*this->jointpositions, destFrame,
      qResult);

  for(unsigned int i = 0; i < nj; ++i)
  {
    this->jointController->SetJointPosition(
        this->joints[i]->GetScopedName(), qResult(i));

    this->jointController->SetPositionPID(this->joints[i]->GetScopedName(),
        common::PID(585, 8, 25.0));
  }
  this->fkSolver->JntToCart(qResult, cartpos);

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&SimpleIKPlugin::Update, this, _1));
}

/////////////////////////////////////////////////
void SimpleIKPlugin::Reset()
{
  this->jointController->SetJointPosition("atlas::r_arm_shy", 1.6896);
  this->jointController->SetJointPosition("atlas::r_arm_shx", 0.0803817);
  this->jointController->SetJointPosition("atlas::r_arm_ely", -0.0821502);
  this->jointController->SetJointPosition("atlas::r_arm_elx", -0.0392042);
  this->jointController->SetJointPosition("atlas::r_arm_wry", -0.128001);

  this->jointController->SetJointPosition("atlas::l_arm_shy", -1.6896);
  this->jointController->SetJointPosition("atlas::l_arm_shx", 0.0803817);
  this->jointController->SetJointPosition("atlas::l_arm_ely", 0.0821502);
  this->jointController->SetJointPosition("atlas::l_arm_elx", -0.0392042);
  this->jointController->SetJointPosition("atlas::l_arm_wry", 0.128001);
}

/////////////////////////////////////////////////
void SimpleIKPlugin::Update(const common::UpdateInfo & /*_info*/)
{
  int nj = 6;

  for(unsigned int i = 0; i < nj; ++i)
  {
    (*jointpositions)(i) =  this->joints[i]->GetAngle(0).Radian();
  }

  Eigen::Matrix<double,6,1> L;

  L(0)=1;L(1)=1;L(2)=1;
  L(3)=0.01;L(4)=0.01;L(5)=0.01;

  KDL::ChainIkSolverPos_LMA iksolverPos(this->chain, L);

  math::Vector3 basePos = this->baseLink->GetWorldPose().pos;
  this->goalPos = this->goalModel->GetWorldPose().pos - basePos;
  KDL::JntArray qResult(this->chain.getNrOfJoints());

  math::Quaternion rot = this->goalModel->GetWorldPose().rot;
  math::Matrix3 mat = rot.GetAsMatrix3();
  KDL::Rotation kdlRot(mat[0][0],mat[0][1], mat[0][2],
                       mat[1][0],mat[1][1], mat[1][2],
                       mat[2][0],mat[2][1], mat[2][2]);

  KDL::Vector kdlVec(this->goalPos.x, this->goalPos.y, this->goalPos.z);
  KDL::Frame destFrame(kdlRot, kdlVec);

  // if CartToHnt result < 0 then it couldn't find a result.
  // However, this is not very useful since the result will get the end
  // effector close to the goal.
  iksolverPos.CartToJnt(*this->jointpositions, destFrame, qResult);

  for(unsigned int i = 0; i < nj; ++i)
  {
    this->jointController->SetPositionTarget(
        this->joints[i]->GetScopedName(), qResult(i));
    /*this->jointController->SetJointPosition(
        this->joints[i]->GetScopedName(), qResult(i));
    (*jointpositions)(i) =  qResult(i);
    */
  }
}


/////////////////////////////////////////////////
/*void SimpleIKPlugin::Update(const common::UpdateInfo &_info)
{
  if ((_info.realTime - this->lastUpdateTime) <= this->updateRate)
  {
    for(unsigned int i = 0; i < 6; ++i)
    {
      double p = (*this->jointpositions)(i) +
        math::clamp(this->vels[i],-1.0, 1.0) * 0.001;
      (*this->jointpositions)(i) = p;

      this->jointController->SetJointPosition(
          this->joints[i]->GetScopedName(), (*this->jointpositions)(i));

    }
    return;
  }

  this->lastUpdateTime = _info.realTime;

  KDL::Frame cartpos;
  math::Vector3 efLocalPos, efGlobalPos;
  math::Vector3 goalGlobalPos, goalLocalPos;
  math::Vector3 velLocal, velGlobal;
  KDL::JntArray qResult(this->chain.getNrOfJoints());


  // Compute the current end effector location.
  this->fkSolver->JntToCart(*this->jointpositions, cartpos);

  // Get the local and global position of the end effector
  efLocalPos.Set(cartpos.p.x(), cartpos.p.y(), cartpos.p.z());
  efGlobalPos = efLocalPos + this->baseLink->GetWorldPose().pos;

  // Get the local and global position of the goal
  goalGlobalPos = this->goalModel->GetWorldPose().pos;
  goalLocalPos = goalGlobalPos - this->baseLink->GetWorldPose().pos;

  velLocal = goalLocalPos - efLocalPos;
  velGlobal = (goalGlobalPos - efGlobalPos);

  std::cout << "------------------------------------\n";
  std::cout << "EF Local Pos[" << efLocalPos << "]\n";
  std::cout << "Goal Local Pos[" << goalLocalPos << "]\n";
  std::cout << "Vel Local Pos[" << velLocal << "]\n";
  std::cout << "EF Global Pos[" << efGlobalPos << "]\n";
  std::cout << "Goal Global Pos[" << goalGlobalPos << "]\n";
  std::cout << "Vel Global Pos[" << velGlobal << "]\n";

  //for (int i = 0; i < this->joints.size(); ++i)
  //  {
  //  (*this->jointpositions)(i) = this->joints[i]->GetAngle(0).Radian();
  //  }

  KDL::Twist twist(KDL::Vector(velGlobal.x, velGlobal.y, velGlobal.z),
      KDL::Vector(0,0,0));
  int ikResult = this->iksolverVel->CartToJnt(*this->jointpositions,
      twist, qResult);
  if (ikResult < 0)
    gzerr << "Maximum number of iterations reached.";

  std::cout << "Number of Chains[" << this->chain.getNrOfJoints() << "]\n";

  double max = 0;
  int nj = this->chain.getNrOfJoints();
  for(unsigned int i = 0; i < nj; ++i)
  {
    std::cout << qResult(i) << std::endl;
    this->vels[i] = qResult(i);
    max = std::max(max, fabs(this->vels[i]));
  }

  std::cout << "max[" << max << "]\n";

  for(unsigned int i = 0; i < 6; ++i)
  {
    double p = (*this->jointpositions)(i) + math::clamp(this->vels[i],-1.0, 1.0) * 0.01;
    (*this->jointpositions)(i) = p;

    this->jointController->SetJointPosition(
        this->joints[i]->GetScopedName(), (*this->jointpositions)(i));
  }
    //this->jointController->SetVelocityTarget(
    //    this->joints[i]->GetScopedName(), qResult(i));

    //this->jointController->SetJointPosition(
    //    this->joints[i]->GetScopedName(), p);
}*/


  /*printf("\n");
  for(int i = 5; i >= 0; --i)
  {
    this->jointController->SetJointPosition(
        this->joints[i]->GetScopedName(), 0.0);
    this->jointController->Update();
    std::cout << "Name[" << this->joints[i]->GetScopedName() << "] Angle[" << this->joints[i]->GetAngle(0).Radian() << "]\n";
  }
  return;
  */

  /*
   double diff = 0.003;
   KDL::Frame cartpos;

   math::Vector3 pos, delta;
   math::Vector3 basePos = this->baseLink->GetWorldPose().pos;
   this->goalPos = this->goalModel->GetWorldPose().pos - basePos;
   */

 /*
   for (int i = this->joints.size()-1; i >= 0; --i)
   {
     double origAngle = (*this->jointpositions)(i);

     (*this->jointpositions)(i) = origAngle + diff;
     this->fkSolver->JntToCart(*this->jointpositions, cartpos);
     pos.Set(cartpos.p.x(), cartpos.p.y(), cartpos.p.z());
     delta =  pos - this->goalPos;
     double plusLength = delta.GetLength();

     (*this->jointpositions)(i) = origAngle - diff;
     this->fkSolver->JntToCart(*this->jointpositions, cartpos);
     pos.Set(cartpos.p.x(), cartpos.p.y(), cartpos.p.z());
     delta =  pos - this->goalPos;
     double minusLength = delta.GetLength();

     if (minusLength < plusLength)
       (*this->jointpositions)(i) = origAngle - diff;
     else
       (*this->jointpositions)(i) = origAngle + diff;

     // std::cout << "Joint[" << this->joints[i]->GetScopedName() << "]\n";
     this->jointController->SetJointPosition(
         this->joints[i]->GetScopedName(), (*this->jointpositions)(i));
   }

   this->fkSolver->JntToCart(*this->jointpositions, cartpos);

   pos.Set(cartpos.p.x(), cartpos.p.y(), cartpos.p.z());
   std::cout << "Hacked local[" << pos  << "] global[" << pos + this->baseLink->GetWorldPose().pos << "] Err[" << (this->goalPos - pos).GetLength() << "]\n";
   */

   /*
   KDL::JntArray qResult(this->chain.getNrOfJoints());
   KDL::Frame destFrame(KDL::Vector(this->goalPos.x,
         this->goalPos.y, this->goalPos.z));
   int ikResult = iksolverPos.CartToJnt(*this->jointpositions, destFrame,
       qResult);
       */

 //   unsigned int nj = this->chain.getNrOfJoints();
 //   for(unsigned int i = 0; i < nj; ++i)
 //   {
 //     (*this->jointpositions)(i) = this->joints[i]->GetAngle(0).Radian();
 //     std::cout << "Angle[" << i << "] [" << this->joints[i]->GetAngle(0).Radian() << "]\n";
 //   }
 //
 //   KDL::JntArray qResult(this->chain.getNrOfJoints());
 //   KDL::ChainIkSolverVel_pinv iksolverVel(this->chain);
 //
 //   this->fkSolver->JntToCart(*this->jointpositions, cartpos);
 //   math::Vector3 result(cartpos.p.x(), cartpos.p.y(), cartpos.p.z());
 //
 //   this->world->GetModel("left_arm_goal")->SetWorldPose(
 //       math::Pose(result+this->baseLink->GetWorldPose().pos,math::Quaternion()));
 //
 //   this->goalPos = this->goalModel->GetWorldPose().pos - (result+this->baseLink->GetWorldPose().pos);
 //
 //   //this->goalPos = this->goalPos - result;
 //
 //   std::cout << "EF Pos Local[" << result << "]\n";
 //   std::cout << "Goal Pos Global[" << this->goalModel->GetWorldPose().pos << "]\n";
 //   delta = this->goalPos;// - result;
 //
 //   std::cout << "Delta[" << delta << "]\n";
 // /*
 //   math::Vector3 rpyCurr = this->model->GetLink("r_hand")->GetWorldPose().rot.GetAsEuler();
 //   math::Vector3 rpyGoal =this->goalModel->GetWorldPose().rot.GetAsEuler();
 //   math::Vector3 rpyVel = rpyGoal - rpyCurr;
 //
 //   double qx, qy, qz, qw;
 //   cartpos.M.GetQuaternion(qx, qy, qz, qw);
 //   math::Quaternion cartQ(qw, qx, qy, qz);
 //   cartQ = cartQ * math::Quaternion(0,0,-1.5707);
 //
 //   math::Quaternion qDiff = this->goalModel->GetWorldPose().rot * cartQ.GetInverse();
 //
 //   std::cout << "EF Local Orient[" <<  cartQ.GetAsEuler() << "]\n";
 //   std::cout << "Goal Local Orient[" << (this->goalModel->GetWorldPose().rot * this->baseLink->GetWorldPose().rot).GetAsEuler() << "]\n";
 //   std::cout << "Orient Diff[" << qDiff.GetAsEuler() << "]\n";
 //
 //   math::Matrix3 mat = qDiff.GetAsMatrix3();
 //   KDL::Rotation kdlRot(mat[0][0],mat[0][1], mat[0][2],
 //                        mat[1][0],mat[1][1], mat[1][2],
 //                        mat[2][0],mat[2][1], mat[2][2]);
 //
 //   KDL::Vector twistOVel(qDiff.GetAsEuler().x *0.1, qDiff.GetAsEuler().y *0.1, qDiff.GetAsEuler().z*0.1);
 //
 //   std::cout << "Twist[" << twistOVel.data[0] << " " << twistOVel.data[1] << " " <<twistOVel.data[2] << "]\n";
 //
 //   KDL::Twist twist(KDL::Vector(delta.x, delta.y, delta.z), twistOVel);
 //   */
 //   KDL::Twist twist(KDL::Vector(delta.x, delta.y, delta.z), KDL::Vector(0,0,0));
 //
 //   int ikResult = iksolverVel.CartToJnt(*this->jointpositions, twist, qResult);
 //   if (ikResult < 0)
 //     std::cerr << "Maximum number of iterations reached.";

//   std::cout << "Number of joints[" << nj << "]\n";
//   for(unsigned int i = 0; i < nj; ++i)
//   {
//     //std::cout << qResult(i)*10. << std::endl;
//     // double p = (*this->jointpositions)(i) + qResult(i)*0.01;
//     // this->joints[i]->SetVelocity(0,qResult(i));
//
//      std::cout << "Name[" << this->joints[i]->GetScopedName() << "] Angle[" << i << "] [" << this->joints[i]->GetAngle(0).Radian() << "]\n";
//     this->jointController->SetJointPosition(
//         this->joints[i]->GetScopedName(), 0);
//     std::cout << "Name[" << this->joints[i]->GetScopedName() << "] Angle[" << i << "] [" << this->joints[i]->GetAngle(0).Radian() << "]\n";
//   }
//   for(unsigned int i = 0; i < nj; ++i)
//     std::cout << "Name[" << this->joints[i]->GetScopedName() << "] Angle[" << i << "] [" << this->joints[i]->GetAngle(0).Radian() << "]\n";
// }
