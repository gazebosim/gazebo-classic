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
  std::string name = this->GetName();
  this->dartJoint->setName(name.c_str());

  //----------------------------------------------------------------------------
  // Step 0.
  //----------------------------------------------------------------------------
  poseChildLinkToJoint = this->anchorPose;

//  if (this->sdf->HasElement("pose"))
//  {
//    sdf::ElementPtr poseElem = this->sdf->GetElement("pose");

//    // The pose of this joint represented in the child link.
//    poseChildLinkToJoint = poseElem->GetValuePose();
//  }
//  else
//  {
//    poseChildLinkToJoint;
//  }

  //----------------------------------------------------------------------------
  // Step 1. Transformation from parent link frame to joint link frame.
  //----------------------------------------------------------------------------
  // Set Pose: offset from child link origin in child link frame.
  if (this->parentLink == NULL)
  {
    Eigen::Matrix4d matChildLink;
    Eigen::Matrix4d matChildLinkToJoint;
    Eigen::Matrix4d matParentLinkToJoint;

    DARTUtils::ConvPoseToMat(&matChildLink, this->childLink->GetWorldPose());
    DARTUtils::ConvPoseToMat(&matChildLinkToJoint, this->poseChildLinkToJoint);

    matParentLinkToJoint = matChildLink * matChildLinkToJoint;

    DARTUtils::ConvMatToPose(&poseParentLinkToJoint, matParentLinkToJoint);
  }
  else
  {
    Eigen::Matrix4d matParentLink;
    Eigen::Matrix4d matParentLinkInv;
    Eigen::Matrix4d matChildLink;
    Eigen::Matrix4d matChildLinkToJoint;
    Eigen::Matrix4d matParentLinkToJoint;

    DARTUtils::ConvPoseToMat(&matParentLink, this->parentLink->GetWorldPose());
    matParentLinkInv = matParentLink.inverse();
    DARTUtils::ConvPoseToMat(&matChildLink, this->childLink->GetWorldPose());
    DARTUtils::ConvPoseToMat(&matChildLinkToJoint, this->poseChildLinkToJoint);

    matParentLinkToJoint = matParentLinkInv
                           * matChildLink
                           * matChildLinkToJoint;

    DARTUtils::ConvMatToPose(&poseParentLinkToJoint, matParentLinkToJoint);
  }

//  if (_sdf->GetElement("parent")->GetValueString("link_name") == std::string("world"))
//  {
//    Eigen::Matrix4d matChildLink;
//    Eigen::Matrix4d matChildLinkToJoint;
//    Eigen::Matrix4d matParentLinkToJoint;

//    DARTUtils::ConvPoseToMat(&matChildLink, this->childLink->GetWorldPose());
//    DARTUtils::ConvPoseToMat(&matChildLinkToJoint, this->poseChildLinkToJoint);

//    matParentLinkToJoint = matChildLink * matChildLinkToJoint;

//    DARTUtils::ConvMatToPose(&poseParentLinkToJoint, matParentLinkToJoint);
//  }
//  else
//  {
//    Eigen::Matrix4d matParentLink;
//    Eigen::Matrix4d matParentLinkInv;
//    Eigen::Matrix4d matChildLink;
//    Eigen::Matrix4d matChildLinkToJoint;
//    Eigen::Matrix4d matParentLinkToJoint;

//    DARTUtils::ConvPoseToMat(&matParentLink, this->parentLink->GetWorldPose());
//    matParentLinkInv = matParentLink.inverse();
//    DARTUtils::ConvPoseToMat(&matChildLink, this->childLink->GetWorldPose());
//    DARTUtils::ConvPoseToMat(&matChildLinkToJoint, this->poseChildLinkToJoint);

//    matParentLinkToJoint = matParentLinkInv
//                           * matChildLink
//                           * matChildLinkToJoint;

//    DARTUtils::ConvMatToPose(&poseParentLinkToJoint, matParentLinkToJoint);
//  }

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
  poseJointToChildLink = poseChildLinkToJoint.GetInverse();

  DARTUtils::AddTransformToDARTJoint(this->dartJoint, poseParentLinkToJoint);

  this->dartJoint->addTransform(rotHinge, true);

  DARTUtils::AddTransformToDARTJoint(this->dartJoint, poseJointToChildLink);
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
  Eigen::Vector3d localAxis_Eigen = rotHinge->getAxis();
  math::Vector3 localAxis(localAxis_Eigen.x(), localAxis_Eigen.y(), localAxis_Eigen.z());
  math::Vector3 globalAxis;

  if (this->parentLink)
  {
    math::Pose worldPoseOfParentLink = this->parentLink->GetWorldPose();
    globalAxis = worldPoseOfParentLink.rot * localAxis;
  }
  else
  {
    globalAxis = localAxis;
  }

  // TODO: Issue #494
  // See: https://bitbucket.org/osrf/gazebo/issue/494/joint-axis-reference-frame-doesnt-match
  //return globalAxis;
  return this->poseParentLinkToJoint.rot * localAxis;
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
  math::Vector3 axisInJointFrame;
  //  if (this->sdf->GetElement("parent")->GetValueString("link_name")
  //      == std::string("world"))
  //  {
  //    axisInJointFrame = _axis;
  //    gzwarn << "Parent link is world.\n";
  //  }
  //  else
  //  {
  axisInJointFrame = this->poseParentLinkToJoint.rot.GetInverse() * _axis;
  //  }

  rotHinge->setAxis(Eigen::Vector3d(axisInJointFrame.x,
                                    axisInJointFrame.y,
                                    axisInJointFrame.z));

  // At some point, we need to change blow code.
  //rotHinge->setAxis(Eigen::Vector3d(_axis.x, _axis.y, _axis.z));
}

//////////////////////////////////////////////////
void DARTHingeJoint::SetDamping(int /*index*/, double /*_damping*/)
{
  //   this->damping_coefficient = _damping;
  //   dJointSetDamping(this->jointId, this->damping_coefficient);
  gzwarn << "Not implemented...\n";
}

//////////////////////////////////////////////////
//void DARTHingeJoint::ApplyDamping()
//{
//   double damping_force = this->damping_coefficient * this->GetVelocity(0);
//   this->SetForce(0, damping_force);
//}

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
  //   double result = dJointGetHingeAngleRate(this->jointId);
  //
  //   return result;
  gzwarn << "DARTHingeJoint::GetVelocity(...): Not implemented...\n";
  return 0;
}

//////////////////////////////////////////////////
void DARTHingeJoint::SetVelocity(int /*index*/, double /*_vel*/)
{
  // TODO: Do nothing because DART accept only torques (forces) of joint as
  // input.
}

//////////////////////////////////////////////////
void DARTHingeJoint::SetMaxForce(int /*index*/, double /*_force*/)
{
  //   return this->SetParam(dParamFMax, _t);
  gzwarn << "DARTHingeJoint::SetMaxForce(...): Not implemented...\n";
}

//////////////////////////////////////////////////
double DARTHingeJoint::GetMaxForce(int /*index*/)
{
  //   return this->GetParam(dParamFMax);
  gzwarn << "DARTHingeJoint::GetMaxForce(...): Not implemented...\n";
  return 0.0;
}

//////////////////////////////////////////////////
void DARTHingeJoint::SetForce(int /*index*/, double /*_torque*/)
{
  //   if (this->childLink)
  //     this->childLink->SetEnabled(true);
  //   if (this->parentLink)
  //     this->parentLink->SetEnabled(true);
  //   dJointAddHingeTorque(this->jointId, _torque);
  gzwarn << "DARTHingeJoint::SetForce(...): Not implemented...\n";
}

//////////////////////////////////////////////////
//double DARTHingeJoint::GetParam(int _parameter) const
//{
//   double result = dJointGetHingeParam(this->jointId, _parameter);
// 
//   return result;
//  return 0;
//}

//////////////////////////////////////////////////
//void DARTHingeJoint::SetParam(int _parameter, double _value)
//{
//   ODEJoint::SetParam(_parameter, _value);
// 
//   dJointSetHingeParam(this->jointId, _parameter, _value);
//}
