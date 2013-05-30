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

#include <boost/bind.hpp>

#include "gazebo/gazebo_config.h"

#include "gazebo/common/Console.hh"

#include "gazebo/physics/Link.hh"
#include "gazebo/physics/dart/DARTModel.hh"
#include "gazebo/physics/dart/DARTHingeJoint.hh"
#include "gazebo/physics/dart/DARTUtils.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
DARTHingeJoint::DARTHingeJoint(BasePtr _parent)
  : HingeJoint<DARTJoint>(_parent)
{
}

//////////////////////////////////////////////////
DARTHingeJoint::~DARTHingeJoint()
{
}

//////////////////////////////////////////////////
void DARTHingeJoint::Load(sdf::ElementPtr _sdf)
{
  HingeJoint<DARTJoint>::Load(_sdf);

  // Name
  std::string jointName = this->GetName();
  this->dartJoint->setName(jointName.c_str());

  //----------------------------------------------------------------------------
  // Step 0.
  //----------------------------------------------------------------------------
  poseChildLinkToJoint = this->anchorPose;

  //----------------------------------------------------------------------------
  // Step 1. Transformation from parent link frame to joint link frame.
  //----------------------------------------------------------------------------
  // Set Pose: offset from child link origin in child link frame.
  if (this->parentLink == NULL)
  {
    Eigen::Matrix4d matChildLink;
    Eigen::Matrix4d matChildLinkToJoint;

    DARTUtils::ConvPoseToMat(&matChildLink, this->childLink->GetWorldPose());
    DARTUtils::ConvPoseToMat(&matChildLinkToJoint, this->poseChildLinkToJoint);
    Eigen::Matrix4d matJointToChildLink = matChildLinkToJoint.inverse();

    Eigen::Matrix4d matParentLinkToJoint = matChildLink * matChildLinkToJoint;

    DARTUtils::ConvMatToPose(&poseParentLinkToJoint, matParentLinkToJoint);
    DARTUtils::ConvMatToPose(&poseJointToChildLink, matJointToChildLink);
  }
  else
  {
    Eigen::Matrix4d matParentLink;
    Eigen::Matrix4d matParentLinkInv;
    Eigen::Matrix4d matChildLink;
    Eigen::Matrix4d matChildLinkToJoint;

    DARTUtils::ConvPoseToMat(&matParentLink, this->parentLink->GetWorldPose());
    matParentLinkInv = matParentLink.inverse();
    DARTUtils::ConvPoseToMat(&matChildLink, this->childLink->GetWorldPose());
    DARTUtils::ConvPoseToMat(&matChildLinkToJoint, this->poseChildLinkToJoint);
    Eigen::Matrix4d matJointToChildLink = matChildLinkToJoint.inverse();

    Eigen::Matrix4d matParentLinkToJoint = matParentLinkInv
                           * matChildLink
                           * matChildLinkToJoint;

    DARTUtils::ConvMatToPose(&poseParentLinkToJoint, matParentLinkToJoint);
    DARTUtils::ConvMatToPose(&poseJointToChildLink, matJointToChildLink);
  }

  //----------------------------------------------------------------------------
  // Step 2. Transformation by the rotate axis.
  //----------------------------------------------------------------------------
  //sdf::ElementPtr axisElem = this->sdf->GetElement("axis");
  //math::Vector3 xyz = axisElem->GetValueVector3("xyz");
  //Eigen::Vector3d axisHinge(xyz.x, xyz.y, xyz.z);
  Eigen::Vector3d axisHinge;
  kinematics::Dof* dofHinge = new kinematics::Dof(0);
  rotHinge = new kinematics::TrfmRotateAxis(axisHinge, dofHinge);

  // Get the model associated with
  // Add the transform to the skeletone in the model.
  // add to model because it's variable
  DARTModelPtr dartModel
      = boost::shared_dynamic_cast<DARTModel>(this->model);
  dartModel->GetSkeletonDynamics()->addTransform(rotHinge);

  //----------------------------------------------------------------------------
  // Step 3. Transformation from rotated joint frame to child link frame.
  //----------------------------------------------------------------------------
  //poseJointToChildLink = poseChildLinkToJoint.GetInverse();

  DARTUtils::AddTransformToDARTJoint(this->dartJoint, poseParentLinkToJoint);

  this->dartJoint->addTransform(rotHinge, true);

  DARTUtils::AddTransformToDARTJoint(this->dartJoint, poseJointToChildLink);

  dartJoint->setDampingCoefficient(0, dampingCoefficient);
}

//////////////////////////////////////////////////
math::Vector3 DARTHingeJoint::GetAnchor(int /*index*/) const
{
  //TODO: need test

  math::Vector3 result;
  //math::Pose poseChildLinkToJoint = -(this->poseJointToChildLink);

  // setting anchor relative to gazebo link frame pose
  if (this->childLink)
    result = poseChildLinkToJoint.pos + this->childLink->GetWorldPose().pos;
  else
    result = math::Vector3(0, 0, 0);

  return result;
}

//////////////////////////////////////////////////
void DARTHingeJoint::SetAnchor(int /*index*/, const math::Vector3& /*_anchor*/)
{
  // TODO: We do not do anything here because DART does not store the positon
  // of the joint.
}

//////////////////////////////////////////////////
math::Vector3 DARTHingeJoint::GetGlobalAxis(int /*_index*/) const
{
  // Axis in local frame of this joint
  const Eigen::Vector3d& localAxis_Eigen = rotHinge->getAxis();
  Eigen::Vector3d globalAxis_Eigen;

  math::Vector3 localAxis(localAxis_Eigen.x(), localAxis_Eigen.y(), localAxis_Eigen.z());
  math::Vector3 globalAxis;
  Eigen::Matrix4d worldToJointTransform = Eigen::Matrix4d::Identity();

  if (this->parentLink)
  {
    worldToJointTransform = dartJoint->getParentNode()->getWorldTransform();
  }
  Eigen::Matrix4d matParentLinkToJoint;
  DARTUtils::ConvPoseToMat(&matParentLinkToJoint, poseParentLinkToJoint);
  worldToJointTransform = worldToJointTransform * matParentLinkToJoint;
  globalAxis_Eigen = worldToJointTransform.topLeftCorner<3,3>() * localAxis_Eigen;
  globalAxis.Set(globalAxis_Eigen(0), globalAxis_Eigen(1), globalAxis_Eigen(2));


  //////////////////////////////////////////////////////////////////////////////
//  Eigen::Matrix4d matParentLink = Eigen::Matrix4d::Identity();
//  if (parentLink)
//  {
//    DARTUtils::ConvPoseToMat(&matParentLink, parentLink->GetWorldPose());
//  }
//  Eigen::Matrix3d rotParentLink = matParentLink.topLeftCorner<3,3>();
//  globalAxis_Eigen = rotParentLink * localAxis_Eigen;
//  globalAxis.Set(globalAxis_Eigen(0), globalAxis_Eigen(1), globalAxis_Eigen(2));

  // TODO: Issue #494
  // See: https://bitbucket.org/osrf/gazebo/issue/494/joint-axis-reference-frame-doesnt-match
  return globalAxis;
  //return this->poseParentLinkToJoint.rot * localAxis;
}

//////////////////////////////////////////////////
void DARTHingeJoint::SetAxis(int /*index*/, const math::Vector3& _axis)
{
  // TODO: Issue #494
  // See: https://bitbucket.org/osrf/gazebo/issue/494/joint-axis-reference-frame-doesnt-match

  // For now the _axis is represented in global frame.
  //math::Pose childWorldPose = this->childLink->GetWorldPose();
  //math::Pose jointFrameInWorld = childWorldPose * this->poseChildLinkToJoint;
  //math::Vector3 axisInJointFrame = (jointFrameInWorld.rot.GetInverse()) * _axis;
  Eigen::Vector3d axisInJointFrame;
  //  if (this->sdf->GetElement("parent")->GetValueString("link_name")
  //      == std::string("world"))
  //  {
  //    axisInJointFrame = _axis;
  //    gzwarn << "Parent link is world.\n";
  //  }
  //  else
  //  {
  Eigen::Matrix4d matJointToParent = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d matParentToJoint = Eigen::Matrix4d::Identity();
  DARTUtils::ConvPoseToMat(&matParentToJoint, poseParentLinkToJoint);
  matJointToParent = matParentToJoint.inverse();
  //  }


//  Eigen::Matrix4d matParentLink = Eigen::Matrix4d::Identity();
//  Eigen::Matrix3d rot;

//  if (parentLink)
//  {
//    DARTUtils::ConvPoseToMat(&matParentLink, this->parentLink->GetWorldPose());
//  }
//  else
//  {

//  }

//  rot = matParentLink.topLeftCorner<3,3>().transpose();

  Eigen::Vector3d axis;
  axis(0) = _axis[0];
  axis(1) = _axis[1];
  axis(2) = _axis[2];

  axisInJointFrame = matJointToParent.topLeftCorner<3,3>() * axis;

  //axisInJointFrame = rot * axis;

//  axis(0) = 1;
//  axis(1) = 0;
//  axis(2) = 0;
//  axisInJointFrame = axis;

  rotHinge->setAxis(axisInJointFrame);

  // At some point, we need to change blow code.
  //rotHinge->setAxis(Eigen::Vector3d(_axis.x, _axis.y, _axis.z));
}

//////////////////////////////////////////////////
void DARTHingeJoint::SetDamping(int _index, double _damping)
{
  assert(_index == 0);
  assert(_damping >= 0.0);

  this->dampingCoefficient = _damping;

  dartJoint->setDampingCoefficient(_index, _damping);
}

//////////////////////////////////////////////////
math::Angle DARTHingeJoint::GetAngleImpl(int /*index*/) const
{
  math::Angle result;

  assert(this->dartJoint);
  assert(this->dartJoint->getNumDofs() == 1);

  // Hinge joint has only one dof.
  kinematics::Dof* dof = this->dartJoint->getDof(0);

  assert(dof);

  result.SetFromRadian(dof->getValue());

  return result;
}

//////////////////////////////////////////////////
double DARTHingeJoint::GetVelocity(int /*index*/) const
{
  double result;

  result = this->dartJoint->getDof(0)->dq;

  return result;
}

//////////////////////////////////////////////////
void DARTHingeJoint::SetVelocity(int /*index*/, double /*_vel*/)
{
  // TODO: Do nothing because DART accept only torques (forces) of joint as
  // input.
    gzwarn << "Not implemented!\n";
}

//////////////////////////////////////////////////
void DARTHingeJoint::SetMaxForce(int /*index*/, double /*_force*/)
{
  gzwarn << "Not implemented!\n";
}

//////////////////////////////////////////////////
double DARTHingeJoint::GetMaxForce(int /*index*/)
{
  gzwarn << "Not implemented!\n";
  return 0.0;
}

//////////////////////////////////////////////////
void DARTHingeJoint::SetForce(int _index, double _torque)
{
  DARTJoint::SetForce(_index, _torque);

  dartJoint->getDof(0)->tau = _torque;
}
