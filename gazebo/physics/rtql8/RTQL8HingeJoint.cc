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
#include "gazebo/physics/rtql8/RTQL8Model.hh"
#include "gazebo/physics/rtql8/RTQL8HingeJoint.hh"
#include "gazebo/physics/rtql8/RTQL8Utils.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
RTQL8HingeJoint::RTQL8HingeJoint(BasePtr _parent)
    : HingeJoint<RTQL8Joint>(_parent)
{

}

//////////////////////////////////////////////////
RTQL8HingeJoint::~RTQL8HingeJoint()
{

}

//////////////////////////////////////////////////
void RTQL8HingeJoint::Load(sdf::ElementPtr _sdf)
{
  HingeJoint<RTQL8Joint>::Load(_sdf);

  //----------------------------------------------------------------------------
  // Step 0.
  //----------------------------------------------------------------------------
  math::Pose poseChildLinkToJoint;

  if (this->sdf->HasElement("pose"))
  {
    sdf::ElementPtr poseElem = this->sdf->GetElement("pose");

    // The pose of this joint represented in the child link.
    poseChildLinkToJoint = poseElem->GetValuePose();
  }
  else
  {
    poseChildLinkToJoint;
  }

  //----------------------------------------------------------------------------
  // Step 1. Transformation from parent link frame to joint link frame.
  //----------------------------------------------------------------------------
  // Set Pose: offset from child link origin in child link frame.
  if (_sdf->GetValueString("parent") == std::string("world"))
  {
    poseParentLinkToJoint = /*-(this->parentLink->GetWorldPose())*/
        /*+ */(this->childLink->GetWorldPose())
        + poseChildLinkToJoint;
  }
  else
  {
    poseParentLinkToJoint = -(this->parentLink->GetWorldPose())
        + (this->childLink->GetWorldPose())
        + poseChildLinkToJoint;
  }


  //----------------------------------------------------------------------------
  // Step 2. Transformation by the rotate axis.
  //----------------------------------------------------------------------------
  //sdf::ElementPtr axisElem = this->sdf->GetElement("axis");
  //math::Vector3 xyz = axisElem->GetValueVector3("xyz");
  //Eigen::Vector3d axisHinge(xyz.x, xyz.y, xyz.z);
  Eigen::Vector3d axisHinge;
  rtql8::kinematics::Dof* dofHinge = new rtql8::kinematics::Dof(0);
  rotHinge = new rtql8::kinematics::TrfmRotateAxis(axisHinge, dofHinge);

  // Get the model associated with
  // Add the transform to the skeletone in the model.
  // add to model because it's variable
  RTQL8ModelPtr rtql8Model
          = boost::shared_dynamic_cast<RTQL8Model>(this->model);
  rtql8Model->GetSkeletonDynamics()->addTransform(rotHinge);

  //----------------------------------------------------------------------------
  // Step 3. Transformation from rotated joint frame to child link frame.
  //----------------------------------------------------------------------------
  poseJointToChildLink = -poseChildLinkToJoint;

RTQL8Utils::addTransformToRTQL8Joint(this->rtql8Joint, poseParentLinkToJoint);
  //RTQL8Utils::addTransformToRTQL8Joint(this->rtql8Joint, poseJointToChildLink);
  this->rtql8Joint->addTransform(rotHinge, true);
  RTQL8Utils::addTransformToRTQL8Joint(this->rtql8Joint, poseJointToChildLink);
}

//////////////////////////////////////////////////
math::Vector3 RTQL8HingeJoint::GetAnchor(int /*index*/) const
{
  //TODO: need test

  math::Vector3 result;
  math::Pose poseChildLinkToJoint = -(this->poseJointToChildLink);

  // setting anchor relative to gazebo link frame pose
  if (this->childLink)
    result = poseChildLinkToJoint.pos + this->childLink->GetWorldPose().pos;
  else
    result = math::Vector3(0, 0, 0);

  return result;
}

//////////////////////////////////////////////////
void RTQL8HingeJoint::SetAnchor(int /*index*/, const math::Vector3& /*_anchor*/)
{
  // TODO: We do not do anything here because RTQL8 does not store the positon
  // of the joint.
}

//////////////////////////////////////////////////
math::Vector3 RTQL8HingeJoint::GetGlobalAxis(int /*_index*/) const
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

  return globalAxis;
}

//////////////////////////////////////////////////
void RTQL8HingeJoint::SetAxis(int /*index*/, const math::Vector3& _axis)
{
  rotHinge->setAxis(Eigen::Vector3d(_axis.x, _axis.y, _axis.z));
}

//////////////////////////////////////////////////
void RTQL8HingeJoint::SetDamping(int /*index*/, double /*_damping*/)
{
//   this->damping_coefficient = _damping;
//   dJointSetDamping(this->jointId, this->damping_coefficient);
  gzerr << "RTQL8HingeJoint::SetDamping(...): Not implemented...\n";
}

//////////////////////////////////////////////////
//void RTQL8HingeJoint::ApplyDamping()
//{
//   double damping_force = this->damping_coefficient * this->GetVelocity(0);
//   this->SetForce(0, damping_force);
//}

//////////////////////////////////////////////////
math::Angle RTQL8HingeJoint::GetAngleImpl(int /*index*/) const
{
   math::Angle result;

   // TODO: need test
   assert(this->rtql8Joint);
   assert(this->rtql8Joint->getNumDofs() == 1);

   // Hinge joint has only one dof.
   rtql8::kinematics::Dof* dof = this->rtql8Joint->getDof(0);

   assert(dof);

   result.SetFromRadian(dof->getValue());

   return result;
}

//////////////////////////////////////////////////
double RTQL8HingeJoint::GetVelocity(int /*index*/) const
{
//   double result = dJointGetHingeAngleRate(this->jointId);
// 
//   return result;
  gzerr << "RTQL8HingeJoint::GetVelocity(...): Not implemented...\n";
  return 0;
}

//////////////////////////////////////////////////
void RTQL8HingeJoint::SetVelocity(int /*index*/, double /*_angle*/)
{
//   this->SetParam(dParamVel, _angle);
  gzerr << "RTQL8HingeJoint::SetVelocity(...): Not implemented...\n";
}

//////////////////////////////////////////////////
void RTQL8HingeJoint::SetMaxForce(int /*index*/, double /*_t*/)
{
//   return this->SetParam(dParamFMax, _t);
  gzerr << "RTQL8HingeJoint::SetMaxForce(...): Not implemented...\n";
}

//////////////////////////////////////////////////
double RTQL8HingeJoint::GetMaxForce(int /*index*/)
{
//   return this->GetParam(dParamFMax);
  gzerr << "RTQL8HingeJoint::GetMaxForce(...): Not implemented...\n";
  return 0.0;
}

//////////////////////////////////////////////////
void RTQL8HingeJoint::SetForce(int /*index*/, double /*_torque*/)
{
//   if (this->childLink)
//     this->childLink->SetEnabled(true);
//   if (this->parentLink)
//     this->parentLink->SetEnabled(true);
//   dJointAddHingeTorque(this->jointId, _torque);
  gzerr << "RTQL8HingeJoint::SetForce(...): Not implemented...\n";
}

//////////////////////////////////////////////////
//double RTQL8HingeJoint::GetParam(int _parameter) const
//{
//   double result = dJointGetHingeParam(this->jointId, _parameter);
// 
//   return result;
//  return 0;
//}

//////////////////////////////////////////////////
//void RTQL8HingeJoint::SetParam(int _parameter, double _value)
//{
//   ODEJoint::SetParam(_parameter, _value);
// 
//   dJointSetHingeParam(this->jointId, _parameter, _value);
//}
