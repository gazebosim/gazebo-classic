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
#include "SimpleIKPlugin.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(SimpleIKPlugin)

/////////////////////////////////////////////////
SimpleIKPlugin::SimpleIKPlugin()
{
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

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&SimpleIKPlugin::Update, this, _1));

  sdf::ElementPtr jointElem = _sdf->GetElement("joint");
  while (jointElem)
  {
    std::cout << "Get Joint[" << jointElem->Get<std::string>() << "]\n";
    this->joints.push_back(
        this->model->GetJoint(jointElem->Get<std::string>()));
    jointElem = jointElem->GetNextElement("joint");
  }

  /*math::Vector3 frame;
  frame = (this->baseLink->GetWorldPose() -
           joints[0]->GetChild()->GetWorldPose()).pos;

    std::cout << "Frame[" << frame << "]\n";
    math::Vector3 axis = joints[0]->GetLocalAxis(0);

    if (axis == math::Vector3(1, 0, 0))
      this->chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX),
            KDL::Frame(KDL::Vector(frame.x, frame.y, frame.z))));
    else if (axis == math::Vector3(0, 1, 0))
      this->chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY),
            KDL::Frame(KDL::Vector(frame.x, frame.y, frame.z))));
    else if (axis == math::Vector3(0, 0, 1))
      this->chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
            KDL::Frame(KDL::Vector(frame.x, frame.y, frame.z))));

            */
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
    std::cout << "Frame[" << frame << "] Axis[" << axis << "]\n";

    this->chain.addSegment(KDL::Segment(
          KDL::Joint(KDL::Vector(0, 0, 0), KDL::Vector(axis.x, axis.y, axis.z), KDL::Joint::RotAxis),
          KDL::Frame(KDL::Vector(frame.x, frame.y, frame.z))));
/*
    if (axis == math::Vector3(1, 0, 0))
      this->chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX),
            KDL::Frame(KDL::Vector(frame.x, frame.y, frame.z))));
    else if (axis == math::Vector3(0, 1, 0))
      this->chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY),
            KDL::Frame(KDL::Vector(frame.x, frame.y, frame.z))));
    else if (axis == math::Vector3(0, 0, 1))
      this->chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
            KDL::Frame(KDL::Vector(frame.x, frame.y, frame.z))));
            */
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
  bool kinematics_status;
  kinematics_status =
    this->fkSolver->JntToCart(*this->jointpositions, cartpos);

  math::Vector3 result(cartpos.p.x(), cartpos.p.y(), cartpos.p.z());
  std::cout << "EF Pos[" << result + this->baseLink->GetWorldPose().pos << "]\n";

  math::Vector3 basePos = this->baseLink->GetWorldPose().pos;
  this->goalPos = this->goalModel->GetWorldPose().pos - basePos;

  std::cout << "GoalPos[" << this->goalPos << "]\n";

  KDL::ChainIkSolverVel_pinv iksolverVel(this->chain);
  KDL::ChainIkSolverPos_NR iksolverPos(this->chain,
      *this->fkSolver, iksolverVel);

  KDL::JntArray qResult(this->chain.getNrOfJoints());
  KDL::Frame destFrame(KDL::Vector(this->goalPos.x,
        this->goalPos.y, this->goalPos.z));
  int ikResult = iksolverPos.CartToJnt(*this->jointpositions, destFrame,
      qResult);

  std::cout << "IKResult[" << ikResult << "]\n";
  if (ikResult < 0)
    std::cerr << "Maximum number of iterations reached.";

  for(unsigned int i = 0; i < nj; ++i)
  {
    std::cout << qResult(i) << std::endl;
  }
}

/////////////////////////////////////////////////
void SimpleIKPlugin::Update(const common::UpdateInfo & /*_info*/)
{
  double diff = 0.001;
  KDL::Frame cartpos;

  math::Vector3 pos, delta;
  math::Vector3 basePos = this->baseLink->GetWorldPose().pos;

  this->goalPos = this->goalModel->GetWorldPose().pos - basePos;

  // physics::Joint_V joints = this->model->GetJoints();
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
}
