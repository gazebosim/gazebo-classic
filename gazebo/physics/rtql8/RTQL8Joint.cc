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

#include "gazebo/common/Exception.hh"
#include "gazebo/common/Console.hh"

#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/physics/Link.hh"
#include "gazebo/physics/PhysicsEngine.hh"
#include "gazebo/physics/rtql8/RTQL8Link.hh"
#include "gazebo/physics/rtql8/RTQL8Model.hh"
#include "gazebo/physics/rtql8/RTQL8Joint.hh"
//#include "physics/ScrewJoint.hh"

#include "rtql8/kinematics/Dof.h"
#include "rtql8/kinematics/Joint.h"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
RTQL8Joint::RTQL8Joint(BasePtr _parent)
  : Joint(_parent), rtql8Joint(NULL)
{
}

//////////////////////////////////////////////////
RTQL8Joint::~RTQL8Joint()
{
  this->Detach();

  if (rtql8Joint)
    delete rtql8Joint;
}

//////////////////////////////////////////////////
void RTQL8Joint::Load(sdf::ElementPtr _sdf)
{
  // In Joint::Load(sdf::ElementPtr), this joint stored the information of the
  // parent link and child link.
  Joint::Load(_sdf);

  // In case this joint is already loaded, we delete rtql8 joint if it is
  // created.
  if (rtql8Joint)
  {
    delete rtql8Joint;
    rtql8Joint = NULL;
  }

  // TODO: need test
  BasePtr myBase = shared_from_this();
  if (this->parentLink)
    boost::shared_dynamic_cast<RTQL8Link>(this->parentLink)->AddRTQL8ChildJoint(boost::shared_static_cast<RTQL8Joint>(myBase));
  boost::shared_dynamic_cast<RTQL8Link>(this->childLink)->SetRTQL8ParentJoint(boost::shared_static_cast<RTQL8Joint>(myBase));

  // In order to create rtql8 joint, we need to know this joint's parent
  // and child link so we create rtql8 joint after the joint is loaded with sdf
  // .
  rtql8::kinematics::BodyNode* childBodyNode
      = boost::shared_dynamic_cast<RTQL8Link>(this->childLink)->GetBodyNode();
  RTQL8ModelPtr rtql8Model
      = boost::shared_dynamic_cast<RTQL8Model>(this->model);

  if (_sdf->GetValueString("parent") == std::string("world"))
  {
    // a) create a rtql8 joint whose parent link is null pointer.
    rtql8Joint = new rtql8::kinematics::Joint(NULL, childBodyNode);

    // b) set the canonical joint of the model as this joint
    rtql8Model->SetCanonicalJoint(rtql8Joint);
  }
  else
  {
    // a) create a rtql8 joint.
    rtql8::kinematics::BodyNode* parentBodyNode
        = boost::shared_dynamic_cast<RTQL8Link>(this->parentLink)->GetBodyNode();
    rtql8Joint = new rtql8::kinematics::Joint(parentBodyNode, childBodyNode);
  }

  rtql8Model->GetSkeletonDynamics()->addJoint(rtql8Joint);
}

//////////////////////////////////////////////////
void RTQL8Joint::Reset()
{
  Joint::Reset();
}

//////////////////////////////////////////////////
LinkPtr RTQL8Joint::GetJointLink(int _index) const
{
  LinkPtr result;

  if (_index == 0)
  {
    RTQL8LinkPtr rtql8Link1
        = boost::shared_static_cast<RTQL8Link>(this->parentLink);

    if (rtql8Link1 != NULL)
      return this->parentLink;
  }

  if (_index == 1)
  {
    RTQL8LinkPtr rtql8Link2
        = boost::shared_static_cast<RTQL8Link>(this->childLink);

    if (rtql8Link2 != NULL)
      return this->childLink;
  }

  return result;
}

//////////////////////////////////////////////////
bool RTQL8Joint::AreConnected(LinkPtr _one, LinkPtr _two) const
{
  //  RTQL8LinkPtr rtql8Link1 = boost::shared_dynamic_cast<RTQL8Link>(_one);
  //  RTQL8LinkPtr rtql8Link2 = boost::shared_dynamic_cast<RTQL8Link>(_two);

  //  if (rtql8Link1 == NULL || rtql8Link2 == NULL)
  //    gzthrow("RTQL8Joint requires RTQL8 bodies\n");

  gzerr << "Not implemented...\n";

  //  return dAreConnected(odeLink1->GetODEId(), odeLink2->GetODEId());
  return (this->childLink.get() == _one.get() && this->parentLink.get() == _two.get())
      || (this->childLink.get() == _two.get() && this->parentLink.get() == _one.get());
}

//////////////////////////////////////////////////
void RTQL8Joint::Attach(LinkPtr _parent, LinkPtr _child)
{
  Joint::Attach(_parent, _child);

  RTQL8LinkPtr rtql8child = boost::shared_dynamic_cast<RTQL8Link>(this->childLink);
  RTQL8LinkPtr rtql8parent = boost::shared_dynamic_cast<RTQL8Link>(this->parentLink);

  if (rtql8child == NULL && rtql8parent == NULL)
    gzthrow("RTQL8Joint requires at least one RTQL8 link\n");

  // TODO: RTQL8's joint can't change their links connected.
  // For now, recreating the joint is the only way.

  // TODO: We need to add the functionality, attach/detaach, into rtql8's joint
  // class.
  //   if (this->rtql8Joint)
  //     delete this->rtql8Joint;

  //   rtql8::kinematics::BodyNode* parentBodyNode = boost::shared_dynamic_cast<RTQL8Link>(
  //         this->parentLink)->GetBodyNode();
  //   rtql8::kinematics::BodyNode* childBodyNode = boost::shared_dynamic_cast<RTQL8Link>(
  //         this->childLink)->GetBodyNode();

  //   this->rtql8Joint = new rtql8::kinematics::Joint(parentBodyNode, childBodyNode);

  gzerr << "Not implemented...\n";
}

//////////////////////////////////////////////////
void RTQL8Joint::Detach()
{
  this->childLink.reset();
  this->parentLink.reset();

  // TODO: RTQL8's joint can't change their links connected.
  // For now, recreating the joint is the only way.
  //  if (this->rtql8Joint)
  //    delete this->rtql8Joint;

  //  kinematics::BodyNode* parentBodyNode = boost::shared_dynamic_cast<RTQL8Link>(
  //        this->parentLink)->GetBodyNode();
  //  kinematics::BodyNode* childBodyNode = boost::shared_dynamic_cast<RTQL8Link>(
  //        this->childLink)->GetBodyNode();

  //  this->rtql8Joint = new rtql8::kinematics::Joint(NULL, NULL);

  gzerr << "Not implemented...\n";
}

//////////////////////////////////////////////////
void RTQL8Joint::SetHighStop(int _index, const math::Angle& _angle)
{
  switch (_index)
  {
    case 0:
      this->rtql8Joint->getDof(_index)->setMax(_angle.Radian());
      break;
    case 1:
      this->rtql8Joint->getDof(_index)->setMax(_angle.Radian());
      break;
    case 2:
      this->rtql8Joint->getDof(_index)->setMax(_angle.Radian());
      break;
    default:
      gzerr << "Invalid index[" << _index << "]\n";
      break;
  };
}

//////////////////////////////////////////////////
void RTQL8Joint::SetLowStop(int _index, const math::Angle& _angle)
{
  switch (_index)
  {
  case 0:
    this->rtql8Joint->getDof(_index)->setMin(_angle.Radian());
    break;
  case 1:
    this->rtql8Joint->getDof(_index)->setMin(_angle.Radian());
    break;
  case 2:
    this->rtql8Joint->getDof(_index)->setMin(_angle.Radian());
    break;
  default:
    gzerr << "Invalid index[" << _index << "]\n";
  };
}

//////////////////////////////////////////////////
math::Angle RTQL8Joint::GetHighStop(int _index)
{
  switch (_index)
  {
  case 0:
    return this->rtql8Joint->getDof(_index)->getMax();
  case 1:
    return this->rtql8Joint->getDof(_index)->getMax();
  case 2:
    return this->rtql8Joint->getDof(_index)->getMax();
  default:
    gzerr << "Invalid index[" << _index << "]\n";
  };

  return 0;
}

//////////////////////////////////////////////////
math::Angle RTQL8Joint::GetLowStop(int _index)
{
  switch (_index)
  {
  case 0:
    return this->rtql8Joint->getDof(_index)->getMin();
  case 1:
    return this->rtql8Joint->getDof(_index)->getMin();
  case 2:
    return this->rtql8Joint->getDof(_index)->getMin();
  default:
    gzerr << "Invalid index[" << _index << "]\n";
  };

  return 0;
}

//////////////////////////////////////////////////
math::Vector3 RTQL8Joint::GetLinkForce(unsigned int /*_index*/) const
{
  math::Vector3 result;
  //   dJointFeedback *jointFeedback = dJointGetFeedback(this->jointId);
  //
  //   if (_index == 0)
  //     result.Set(jointFeedback->f1[0], jointFeedback->f1[1],
  //                jointFeedback->f1[2]);
  //   else
  //     result.Set(jointFeedback->f2[0], jointFeedback->f2[1],
  //                jointFeedback->f2[2]);

  gzerr << "RTQL8Joint::GetLinkForce(...): Not implemented...\n";

  return result;
}

//////////////////////////////////////////////////
math::Vector3 RTQL8Joint::GetLinkTorque(unsigned int /*_index*/) const
{
  math::Vector3 result;
  //   dJointFeedback *jointFeedback = dJointGetFeedback(this->jointId);
  //
  //   if (_index == 0)
  //     result.Set(jointFeedback->t1[0], jointFeedback->t1[1],
  //                jointFeedback->t1[2]);
  //   else
  //     result.Set(jointFeedback->t2[0], jointFeedback->t2[1],
  //                jointFeedback->t2[2]);

  gzerr << "RTQL8Joint::GetLinkTorque(...): Not implemented...\n";

  return result;
}

//////////////////////////////////////////////////
void RTQL8Joint::SetAttribute(Attribute /*_attr*/, int /*_index*/, double /*_value*/)
{
  //   switch (_attr)
  //   {
  //     case FUDGE_FACTOR:
  //       this->SetParam(dParamFudgeFactor, _value);
  //       break;
  //     case SUSPENSION_ERP:
  //       this->SetParam(dParamSuspensionERP, _value);
  //       break;
  //     case SUSPENSION_CFM:
  //       this->SetParam(dParamSuspensionCFM, _value);
  //       break;
  //     case STOP_ERP:
  //       this->SetParam(dParamStopERP, _value);
  //       break;
  //     case STOP_CFM:
  //       this->SetParam(dParamStopCFM, _value);
  //       break;
  //     case ERP:
  //       this->SetParam(dParamERP, _value);
  //       break;
  //     case CFM:
  //       this->SetParam(dParamCFM, _value);
  //       break;
  //     case FMAX:
  //       this->SetParam(dParamFMax, _value);
  //       break;
  //     case VEL:
  //       this->SetParam(dParamVel, _value);
  //       break;
  //     case HI_STOP:
  //       this->SetParam(dParamHiStop, _value);
  //       break;
  //     case LO_STOP:
  //       this->SetParam(dParamLoStop, _value);
  //       break;
  //     default:
  //       gzerr << "Unable to handle joint attribute[" << _attr << "]\n";
  //       break;
  //   };
  gzerr << "Not implemented...\n";
}

//////////////////////////////////////////////////
void RTQL8Joint::SetAttribute(const std::string &/*_key*/, int /*_index*/,
                              const boost::any &/*_value*/)
{
  //   if (_key == "fudge_factor")
  //   {
  //     try
  //     {
  //       this->SetParam(dParamFudgeFactor, boost::any_cast<double>(_value));
  //     }
  //     catch(boost::bad_any_cast &e)
  //     {
  //       gzerr << "boost any_cast error:" << e.what() << "\n";
  //     }
  //   }
  //   else if (_key == "suspension_erp")
  //   {
  //     try
  //     {
  //       this->SetParam(dParamSuspensionERP, boost::any_cast<double>(_value));
  //     }
  //     catch(boost::bad_any_cast &e)
  //     {
  //       gzerr << "boost any_cast error:" << e.what() << "\n";
  //     }
  //   }
  //   else if (_key == "suspension_cfm")
  //   {
  //     try
  //     {
  //       this->SetParam(dParamSuspensionCFM, boost::any_cast<double>(_value));
  //     }
  //     catch(boost::bad_any_cast &e)
  //     {
  //       gzerr << "boost any_cast error:" << e.what() << "\n";
  //     }
  //   }
  //   else if (_key == "stop_erp")
  //   {
  //     try
  //     {
  //       this->SetParam(dParamStopERP, boost::any_cast<double>(_value));
  //     }
  //     catch(boost::bad_any_cast &e)
  //     {
  //       gzerr << "boost any_cast error:" << e.what() << "\n";
  //     }
  //   }
  //   else if (_key == "stop_cfm")
  //   {
  //     try
  //     {
  //       this->SetParam(dParamStopCFM, boost::any_cast<double>(_value));
  //     }
  //     catch(boost::bad_any_cast &e)
  //     {
  //       gzerr << "boost any_cast error:" << e.what() << "\n";
  //     }
  //   }
  //   else if (_key == "erp")
  //   {
  //     try
  //     {
  //       this->SetParam(dParamERP, boost::any_cast<double>(_value));
  //     }
  //     catch(boost::bad_any_cast &e)
  //     {
  //       gzerr << "boost any_cast error:" << e.what() << "\n";
  //     }
  //   }
  //   else if (_key == "cfm")
  //   {
  //     try
  //     {
  //       this->SetParam(dParamCFM, boost::any_cast<double>(_value));
  //     }
  //     catch(boost::bad_any_cast &e)
  //     {
  //       gzerr << "boost any_cast error:" << e.what() << "\n";
  //     }
  //   }
  //   else if (_key == "fmax")
  //   {
  //     try
  //     {
  //       this->SetParam(dParamFMax, boost::any_cast<double>(_value));
  //     }
  //     catch(boost::bad_any_cast &e)
  //     {
  //       gzerr << "boost any_cast error:" << e.what() << "\n";
  //     }
  //   }
  //   else if (_key == "vel")
  //   {
  //     try
  //     {
  //       this->SetParam(dParamVel, boost::any_cast<double>(_value));
  //     }
  //     catch(boost::bad_any_cast &e)
  //     {
  //       gzerr << "boost any_cast error:" << e.what() << "\n";
  //     }
  //   }
  //   else if (_key == "hi_stop")
  //   {
  //     try
  //     {
  //       this->SetParam(dParamHiStop, boost::any_cast<double>(_value));
  //     }
  //     catch(boost::bad_any_cast &e)
  //     {
  //       gzerr << "boost any_cast error:" << e.what() << "\n";
  //     }
  //   }
  //   else if (_key == "lo_stop")
  //   {
  //     try
  //     {
  //       this->SetParam(dParamLoStop, boost::any_cast<double>(_value));
  //     }
  //     catch(boost::bad_any_cast &e)
  //     {
  //       gzerr << "boost any_cast error:" << e.what() << "\n";
  //     }
  //   }
  //   else if (_key == "thread_pitch")
  //   {
  //     ScrewJoint<RTQL8Joint>* screwJoint =
  //       dynamic_cast<ScrewJoint<RTQL8Joint>* >(this);
  //     if (screwJoint != NULL)
  //     {
  //       try
  //       {
  //         screwJoint->SetThreadPitch(0, boost::any_cast<double>(_value));
  //       }
  //       catch(boost::bad_any_cast &e)
  //       {
  //         gzerr << "boost any_cast error:" << e.what() << "\n";
  //       }
  //     }
  //   }
  //   else
  //   {
  //     try
  //     {
  //       gzerr << "Unable to handle joint attribute["
  //             << boost::any_cast<std::string>(_value) << "]\n";
  //     }
  //     catch(boost::bad_any_cast &e)
  //     {
  //       gzerr << "boost any_cast error:" << e.what() << "\n";
  //     }
  //   }
  gzerr << "Not implemented...\n";
}

JointWrench RTQL8Joint::GetForceTorque(int /*_index*/)
{
  JointWrench wrench;
  //  // Note that:
  //  // f2, t2 are the force torque measured on parent body's cg
  //  // f1, t1 are the force torque measured on child body's cg
  //  dJointFeedback* fb = this->GetFeedback();
  //  if (fb)
  //  {
  //    wrench.body1Force.Set(fb->f1[0], fb->f1[1], fb->f1[2]);
  //    wrench.body1Torque.Set(fb->t1[0], fb->t1[1], fb->t1[2]);
  //    wrench.body2Force.Set(fb->f2[0], fb->f2[1], fb->f2[2]);
  //    wrench.body2Torque.Set(fb->t2[0], fb->t2[1], fb->t2[2]);

  //    if (this->childLink)
  //    {
  //      // convert torque from about child CG to joint anchor location
  //      // cg position specified in child link frame
  //      math::Vector3 cgPos = this->childLink->GetInertial()->GetPose().pos;
  //      // moment arm rotated into world frame (given feedback is in world frame)
  //      math::Vector3 childMomentArm =
  //        this->childLink->GetWorldPose().rot.RotateVector(
  //        this->anchorPos - cgPos);

  //      // gzerr << "anchor [" << anchorPos
  //      //       << "] iarm[" << this->childLink->GetInertial()->GetPose().pos
  //      //       << "] childMomentArm[" << childMomentArm
  //      //       << "] f1[" << wrench.body1Force
  //      //       << "] t1[" << wrench.body1Torque
  //      //       << "] fxp[" << wrench.body1Force.Cross(childMomentArm)
  //      //       << "]\n";

  //      wrench.body1Torque += wrench.body1Force.Cross(childMomentArm);
  //    }

  //    // convert torque from about parent CG to joint anchor location
  //    if (this->parentLink)
  //    {
  //      // parent cg specified in child link frame
  //      math::Vector3 cgPos = ((this->parentLink->GetInertial()->GetPose() +
  //                            this->parentLink->GetWorldPose()) -
  //                            this->childLink->GetWorldPose()).pos;

  //      // rotate moement arms into world frame
  //      math::Vector3 parentMomentArm =
  //        this->childLink->GetWorldPose().rot.RotateVector(
  //        this->anchorPos - cgPos);

  //      wrench.body2Torque -= wrench.body2Force.Cross(parentMomentArm);

  //      // A good check is that
  //      // the computed body2Torque shoud in fact be opposite of body1Torque
  //    }
  //    else
  //    {
  //      // convert torque from about child CG to joint anchor location
  //      // or simply use equal opposite force as body1 wrench
  //      wrench.body2Force = -wrench.body1Force;
  //      wrench.body2Torque = -wrench.body1Torque;
  //    }
  //  }
  //  else
  //  {
  //    // forgot to set provide_feedback?
  //    gzwarn << "GetForceTorque: forget to set <provide_feedback>?\n";
  //  }

  gzerr << "Not implemented...\n";

  return wrench;
}
