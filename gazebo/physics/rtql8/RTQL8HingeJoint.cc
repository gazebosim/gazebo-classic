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

#include "rtql8/kinematics/Dof.h"
#include "rtql8/kinematics/TrfmRotateEuler.h"

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

  if (this->sdf->HasElement("axis"))
  {
    sdf::ElementPtr axisElem = this->sdf->GetElement("axis");

    math::Vector3 xyz = axisElem->GetValueVector3("xyz");
    Eigen::Vector3d axisHinge(xyz.x, xyz.y, xyz.z);

    rtql8::kinematics::Dof* dofHinge = new rtql8::kinematics::Dof(0);

    rtql8::kinematics::TrfmRotateAxis1* rotHinge
        = new rtql8::kinematics::TrfmRotateAxis1(axisHinge, dofHinge);

    this->rtql8Joint->addTransform(rotHinge);

    // Get the model associated with
    // Add the transform to the skeletone in the model.
    // add to model because it's variable
//    boost::shared_dynamic_cast<RTQL8Model>(this->model)->GetSkeletonDynamics()->addTransform(rotHinge);
    RTQL8ModelPtr rtql8Model = boost::shared_dynamic_cast<RTQL8Model>(this->model);
    rtql8Model->GetSkeletonDynamics()->addTransform(rotHinge);

    if (axisElem->HasElement("limit"))
    {
      sdf::ElementPtr limitElem = axisElem->GetElement("limit");
      dofHinge->setMin(limitElem->GetValueDouble("lower"));
      dofHinge->setMax(limitElem->GetValueDouble("upper"));
    }
  }

  //---- Step 3. Third pose
  math::Pose pose;// = poseElem->GetValuePose();

  if (this->sdf->HasElement("pose"))
  {
    sdf::ElementPtr poseElem = this->sdf->GetElement("pose");

    math::Pose poseL1toJoint = poseElem->GetValuePose();

    pose -= poseL1toJoint;
  }

  pose -= this->childLink->GetWorldPose();
  pose += this->parentLink->GetWorldPose();

  rtql8::kinematics::Dof* tranX = new rtql8::kinematics::Dof(pose.pos.x);
  rtql8::kinematics::Dof* tranY = new rtql8::kinematics::Dof(pose.pos.y);
  rtql8::kinematics::Dof* tranZ = new rtql8::kinematics::Dof(pose.pos.z);

  rtql8::kinematics::TrfmTranslate* tran
      = new rtql8::kinematics::TrfmTranslate(tranX, tranY, tranZ);

  this->rtql8Joint->addTransform(tran, false);

  rtql8::kinematics::Dof* rotW = new rtql8::kinematics::Dof(pose.rot.w);
  rtql8::kinematics::Dof* rotX = new rtql8::kinematics::Dof(pose.rot.x);
  rtql8::kinematics::Dof* rotY = new rtql8::kinematics::Dof(pose.rot.y);
  rtql8::kinematics::Dof* rotZ = new rtql8::kinematics::Dof(pose.rot.z);

  rtql8::kinematics::TrfmRotateQuat* rot
      = new rtql8::kinematics::TrfmRotateQuat(rotW, rotX, rotY, rotZ);

  this->rtql8Joint->addTransform(rot, false);
}

//////////////////////////////////////////////////
math::Vector3 RTQL8HingeJoint::GetAnchor(int /*index*/) const
{
//   dVector3 result;
// 
//   dJointGetHingeAnchor(this->jointId, result);
// 
//   return math::Vector3(result[0], result[1], result[2]);

  gzerr << "Not implemented...\n";

  return math::Vector3(0, 0, 0);
}

//////////////////////////////////////////////////
void RTQL8HingeJoint::SetAnchor(int /*index*/, const math::Vector3 &/*_anchor*/)
{
//   if (this->childLink)
//     this->childLink->SetEnabled(true);
//   if (this->parentLink)
//     this->parentLink->SetEnabled(true);
// 
//   dJointSetHingeAnchor(this->jointId, _anchor.x, _anchor.y, _anchor.z);
  gzerr << "Not implemented...\n";
}


//////////////////////////////////////////////////
math::Vector3 RTQL8HingeJoint::GetGlobalAxis(int /*_index*/) const
{
//     dVector3 result;
//     dJointGetHingeAxis(this->jointId, result);
//     return math::Vector3(result[0], result[1], result[2]);
  return math::Vector3(0, 0, 0);
}

//////////////////////////////////////////////////
void RTQL8HingeJoint::SetAxis(int /*index*/, const math::Vector3 &/*_axis*/)
{
//   if (this->childLink)
//     this->childLink->SetEnabled(true);
//   if (this->parentLink)
//     this->parentLink->SetEnabled(true);
// 
//   dJointSetHingeAxis(this->jointId, _axis.x, _axis.y, _axis.z);
}

//////////////////////////////////////////////////
void RTQL8HingeJoint::SetDamping(int /*index*/, double /*_damping*/)
{
//   this->damping_coefficient = _damping;
//   dJointSetDamping(this->jointId, this->damping_coefficient);
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
  return 0;
}

//////////////////////////////////////////////////
void RTQL8HingeJoint::SetVelocity(int /*index*/, double /*_angle*/)
{
//   this->SetParam(dParamVel, _angle);
}

//////////////////////////////////////////////////
void RTQL8HingeJoint::SetMaxForce(int /*index*/, double /*_t*/)
{
//   return this->SetParam(dParamFMax, _t);
}

//////////////////////////////////////////////////
double RTQL8HingeJoint::GetMaxForce(int /*index*/)
{
//   return this->GetParam(dParamFMax);
  return 0;
}

//////////////////////////////////////////////////
void RTQL8HingeJoint::SetForce(int /*index*/, double /*_torque*/)
{
//   if (this->childLink)
//     this->childLink->SetEnabled(true);
//   if (this->parentLink)
//     this->parentLink->SetEnabled(true);
//   dJointAddHingeTorque(this->jointId, _torque);
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
