/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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
#include <functional>
#include "gazebo/common/Exception.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Assert.hh"

#include "gazebo/physics/World.hh"
#include "gazebo/physics/Link.hh"
#include "gazebo/physics/PhysicsEngine.hh"
#include "gazebo/physics/ode/ODELink.hh"
#include "gazebo/physics/GearboxJoint.hh"
#include "gazebo/physics/ScrewJoint.hh"
#include "gazebo/physics/JointWrench.hh"
#include "gazebo/physics/Inertial.hh"

#include "gazebo/physics/ode/ODEJointPrivate.hh"
#include "gazebo/physics/ode/ODEJoint.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
ODEJoint::ODEJoint(BasePtr _parent)
: Joint(*new ODEJointPrivate, _parent),
  odeJointDPtr(static_cast<ODEJointPrivate*>(this->jointDPtr))
{
  this->odeJointDPtr->jointId = NULL;
  this->odeJointDPtr->implicitDampingState[0] = ODEJoint::NONE;
  this->odeJointDPtr->implicitDampingState[1] = ODEJoint::NONE;
  this->odeJointDPtr->stiffnessDampingInitialized = false;
  this->odeJointDPtr->feedback = NULL;
  this->odeJointDPtr->currentKd[0] = 0;
  this->odeJointDPtr->currentKd[1] = 0;
  this->odeJointDPtr->currentKp[0] = 0;
  this->odeJointDPtr->currentKp[1] = 0;
  this->odeJointDPtr->forceApplied[0] = 0;
  this->odeJointDPtr->forceApplied[1] = 0;
  this->odeJointDPtr->useImplicitSpringDamper = false;
  this->odeJointDPtr->stopERP = 0.0;
  this->odeJointDPtr->stopCFM = 0.0;
}

//////////////////////////////////////////////////
ODEJoint::~ODEJoint()
{
  this->Fini();
}

//////////////////////////////////////////////////
void ODEJoint::Fini()
{
  if (this->applyDamping)
    physics::Joint::DisconnectJointUpdate(this->applyDamping);
  this->applyDamping.reset();

  if (this->feedback)
    delete this->feedback;
  this->feedback = NULL;

  this->Detach();

  if (this->jointId)
    dJointDestroy(this->jointId);
  this->jointId = NULL;

  Joint::Fini();
}

//////////////////////////////////////////////////
void ODEJoint::Load(sdf::ElementPtr _sdf)
{
  Joint::Load(_sdf);

  if (this->jointDPtr->sdf->HasElement("physics") &&
      this->jointDPtr->sdf->GetElement("physics")->HasElement("ode"))
  {
    sdf::ElementPtr elem =
      this->jointDPtr->sdf->GetElement("physics")->GetElement("ode");

    if (elem->HasElement("implicit_spring_damper"))
    {
      this->odeJointDPtr->useImplicitSpringDamper =
        elem->Get<bool>("implicit_spring_damper");
    }

    // initializa both axis, \todo: make cfm, erp per axis
    this->odeJointDPtr->stopERP = elem->GetElement("limit")->Get<double>("erp");
    for (unsigned int i = 0; i < this->AngleCount(); ++i)
      this->SetParam("stop_erp", i, this->odeJointDPtr->stopERP);

    // initializa both axis, \todo: make cfm, erp per axis
    this->odeJointDPtr->stopCFM = elem->GetElement("limit")->Get<double>("cfm");
    for (unsigned int i = 0; i < this->AngleCount(); ++i)
      this->SetParam("stop_cfm", i, this->odeJointDPtr->stopCFM);

    if (elem->HasElement("suspension"))
    {
      this->SetParam(dParamSuspensionERP,
          elem->GetElement("suspension")->Get<double>("erp"));
      this->SetParam(dParamSuspensionCFM,
          elem->GetElement("suspension")->Get<double>("cfm"));
    }

    if (elem->HasElement("fudge_factor"))
      this->SetParam(dParamFudgeFactor,
          elem->GetElement("fudge_factor")->Get<double>());

    if (elem->HasElement("cfm"))
        this->SetParam("cfm", 0, elem->Get<double>("cfm"));

    if (elem->HasElement("erp"))
        this->SetParam("erp", 0, elem->Get<double>("erp"));

    if (elem->HasElement("bounce"))
        this->SetParam(dParamBounce,
          elem->GetElement("bounce")->Get<double>());

    if (elem->HasElement("max_force"))
      this->SetParam(dParamFMax,
          elem->GetElement("max_force")->Get<double>());

    if (elem->HasElement("velocity"))
      this->SetParam(dParamVel,
          elem->GetElement("velocity")->Get<double>());
  }
}

//////////////////////////////////////////////////
LinkPtr ODEJoint::JointLink(const unsigned int _index) const
{
  LinkPtr result;
  if (!this->odeJointDPtr->jointId)
  {
    gzerr << "ODE Joint ID is invalid\n";
    return result;
  }

  if (_index == 0 || _index == 1)
  {
    ODELinkPtr odeLink1 =
      std::static_pointer_cast<ODELink>(this->jointDPtr->childLink);
    ODELinkPtr odeLink2 =
      std::static_pointer_cast<ODELink>(this->jointDPtr->parentLink);
    if (odeLink1 != NULL &&
        dJointGetBody(this->odeJointDPtr->jointId, _index) == odeLink1->ODEId())
    {
      result = this->jointDPtr->childLink;
    }
    else if (odeLink2)
    {
      result = this->jointDPtr->parentLink;
    }
  }

  return result;
}

//////////////////////////////////////////////////
bool ODEJoint::AreConnected(LinkPtr _one, LinkPtr _two) const
{
  ODELinkPtr odeLink1 = std::dynamic_pointer_cast<ODELink>(_one);
  ODELinkPtr odeLink2 = std::dynamic_pointer_cast<ODELink>(_two);

  if (odeLink1 == NULL || odeLink2 == NULL)
    gzthrow("ODEJoint requires ODE bodies\n");

  return dAreConnected(odeLink1->ODEId(), odeLink2->ODEId());
}

//////////////////////////////////////////////////
// child classes where appropriate
double ODEJoint::Param(const unsigned int /*parameter*/) const
{
  return 0;
}

//////////////////////////////////////////////////
void ODEJoint::Attach(LinkPtr _parent, LinkPtr _child)
{
  Joint::Attach(_parent, _child);

  ODELinkPtr odechild = std::dynamic_pointer_cast<ODELink>(
      this->jointDPtr->childLink);
  ODELinkPtr odeparent = std::dynamic_pointer_cast<ODELink>(
      this->jointDPtr->parentLink);

  if (odechild == NULL && odeparent == NULL)
    gzthrow("ODEJoint requires at least one ODE link\n");

  if (!this->odeJointDPtr->jointId)
    gzerr << "ODE Joint ID is invalid\n";

  if (this->HasType(Base::HINGE2_JOINT) &&
      (odechild == NULL || odeparent == NULL))
    gzthrow("ODEHinge2Joint cannot be connected to the world");

  if (!odechild && odeparent)
  {
    dJointAttach(this->odeJointDPtr->jointId, 0, odeparent->ODEId());
  }
  else if (odechild && !odeparent)
  {
    dJointAttach(this->odeJointDPtr->jointId, odechild->ODEId(), 0);
  }
  else if (odechild && odeparent)
  {
    if (this->HasType(Base::HINGE2_JOINT))
    {
      dJointAttach(
          this->odeJointDPtr->jointId, odeparent->ODEId(), odechild->ODEId());
    }
    else
    {
      dJointAttach(
          this->odeJointDPtr->jointId, odechild->ODEId(), odeparent->ODEId());
    }
  }
}

//////////////////////////////////////////////////
void ODEJoint::Detach()
{
  auto odeChild = boost::dynamic_pointer_cast<ODELink>(this->childLink);
  auto odeParent = boost::dynamic_pointer_cast<ODELink>(this->parentLink);

  Joint::Detach();
  this->childLink.reset();
  this->parentLink.reset();

  // By the time we get here, links and ODEIds might have already been
  // cleaned up
  if ((odeParent == NULL || odeParent->GetODEId() == NULL) ||
      (odeChild == NULL || odeChild->GetODEId() == NULL))
  {
    return;
  }

  if (this->jointId)
    dJointAttach(this->jointId, 0, 0);
  else
    gzerr << "ODE Joint ID is invalid\n";
}

//////////////////////////////////////////////////
// where appropriate
void ODEJoint::SetParam(const unsigned int /*parameter*/,
    const double /*value*/)
{
  if (this->jointDPtr->childLink)
    this->jointDPtr->childLink->SetEnabled(true);
  if (this->jointDPtr->parentLink)
    this->jointDPtr->parentLink->SetEnabled(true);
}

//////////////////////////////////////////////////
void ODEJoint::SetERP(const double _newERP)
{
  this->SetParam(dParamSuspensionERP, _newERP);
}

//////////////////////////////////////////////////
double ODEJoint::GetERP()
{
  return this->ERP();
}

//////////////////////////////////////////////////
double ODEJoint::ERP()
{
  return this->Param(dParamSuspensionERP);
}

//////////////////////////////////////////////////
void ODEJoint::SetCFM(const double _newCFM)
{
  this->SetParam(dParamSuspensionCFM, _newCFM);
}

//////////////////////////////////////////////////
double ODEJoint::GetCFM()
{
  return this->CFM();
}

//////////////////////////////////////////////////
double ODEJoint::CFM() const
{
  return this->Param(dParamSuspensionCFM);
}

//////////////////////////////////////////////////
dJointFeedback *ODEJoint::GetFeedback()
{
  return this->Feedback();
}

//////////////////////////////////////////////////
dJointFeedback *ODEJoint::Feedback() const
{
  if (this->odeJointDPtr->jointId)
    return dJointGetFeedback(this->odeJointDPtr->jointId);
  else
    gzerr << "ODE Joint ID is invalid\n";
  return NULL;
}

//////////////////////////////////////////////////
bool ODEJoint::SetHighStop(const unsigned int _index,
    const ignition::math::Angle &_angle)
{
  Joint::SetHighStop(_index, _angle);
  switch (_index)
  {
    case 0:
      this->SetParam(dParamHiStop, _angle.Radian());
      return true;
    case 1:
      this->SetParam(dParamHiStop2, _angle.Radian());
      return true;
    case 2:
      this->SetParam(dParamHiStop3, _angle.Radian());
      return true;
    default:
      gzerr << "Invalid index[" << _index << "]\n";
      return false;
  };
}

//////////////////////////////////////////////////
bool ODEJoint::SetLowStop(const unsigned int _index,
    const ignition::math::Angle &_angle)
{
  Joint::SetLowStop(_index, _angle);
  switch (_index)
  {
    case 0:
      this->SetParam(dParamLoStop, _angle.Radian());
      return true;
    case 1:
      this->SetParam(dParamLoStop2, _angle.Radian());
      return true;
    case 2:
      this->SetParam(dParamLoStop3, _angle.Radian());
      return true;
    default:
      gzerr << "Invalid index[" << _index << "]\n";
      return false;
  };
}

//////////////////////////////////////////////////
ignition::math::Angle ODEJoint::HighStop(const unsigned int _index) const
{
  return this->UpperLimit(_index);
}

//////////////////////////////////////////////////
ignition::math::Angle ODEJoint::LowStop(const unsigned int _index) const
{
  return this->LowerLimit(_index);
}

//////////////////////////////////////////////////
ignition::math::Vector3d ODEJoint::LinkForce(const unsigned int _index) const
{
  ignition::math::Vector3d result;

  if (!this->odeJointDPtr->jointId)
  {
    gzerr << "ODE Joint ID is invalid\n";
    return result;
  }

  dJointFeedback *jointFeedback = dJointGetFeedback(
      this->odeJointDPtr->jointId);

  if (_index == 0)
    result.Set(jointFeedback->f1[0], jointFeedback->f1[1],
               jointFeedback->f1[2]);
  else
    result.Set(jointFeedback->f2[0], jointFeedback->f2[1],
               jointFeedback->f2[2]);

  return result;
}

//////////////////////////////////////////////////
ignition::math::Vector3d ODEJoint::LinkTorque(const unsigned int _index) const
{
  ignition::math::Vector3d result;

  if (!this->odeJointDPtr->jointId)
  {
    gzerr << "ODE Joint ID is invalid\n";
    return result;
  }

  dJointFeedback *jointFeedback =
    dJointGetFeedback(this->odeJointDPtr->jointId);

  if (!jointFeedback)
  {
    gzerr << "Joint feedback uninitialized" << std::endl;
    return result;
  }

  if (_index == 0)
    result.Set(jointFeedback->t1[0], jointFeedback->t1[1],
               jointFeedback->t1[2]);
  else
    result.Set(jointFeedback->t2[0], jointFeedback->t2[1],
               jointFeedback->t2[2]);

  return result;
}

//////////////////////////////////////////////////
bool ODEJoint::SetParam(const std::string &_key, const unsigned int _index,
                        const boost::any &_value)
{
  // Axis parameters for multi-axis joints use a group bitmask
  // to identify the variable.
  unsigned int group;
  switch (_index)
  {
    case 0:
      group = dParamGroup1;
      break;
    case 1:
      group = dParamGroup2;
      break;
    case 2:
      group = dParamGroup3;
      break;
    default:
      gzerr << "Invalid index[" << _index << "]\n";
      return false;
  };

  // try because boost::any_cast can throw
  try
  {
    if (_key == "fudge_factor")
    {
      this->SetParam(dParamFudgeFactor, boost::any_cast<double>(_value));
    }
    else if (_key == "suspension_erp")
    {
      this->SetParam(dParamSuspensionERP, boost::any_cast<double>(_value));
    }
    else if (_key == "suspension_cfm")
    {
      this->SetParam(dParamSuspensionCFM, boost::any_cast<double>(_value));
    }
    else if (_key == "stop_erp")
    {
      this->SetParam(dParamStopERP | group, boost::any_cast<double>(_value));
    }
    else if (_key == "stop_cfm")
    {
      this->SetParam(dParamStopCFM | group, boost::any_cast<double>(_value));
    }
    else if (_key == "erp")
    {
      this->SetParam(dParamERP, boost::any_cast<double>(_value));
    }
    else if (_key == "cfm")
    {
      this->SetParam(dParamCFM, boost::any_cast<double>(_value));
    }
    else if (_key == "fmax")
    {
      this->SetParam(dParamFMax | group, boost::any_cast<double>(_value));
    }
    else if (_key == "friction")
    {
      // To represent Coulomb friction,
      //  set FMax to friction value
      //  set Vel to 0
      this->SetParam(group | dParamFMax, boost::any_cast<double>(_value));
      this->SetParam(group | dParamVel, 0.0);
    }
    else if (_key == "vel")
    {
      this->SetParam(dParamVel | group, boost::any_cast<double>(_value));
    }
    else if (_key == "hi_stop")
    {
      this->SetParam(dParamHiStop | group, boost::any_cast<double>(_value));
    }
    else if (_key == "lo_stop")
    {
      this->SetParam(dParamLoStop | group, boost::any_cast<double>(_value));
    }
    else if (_key == "thread_pitch")
    {
      ScrewJoint<ODEJoint>* screwJoint =
        dynamic_cast<ScrewJoint<ODEJoint>* >(this);
      if (screwJoint != NULL)
      {
        screwJoint->SetThreadPitch(boost::any_cast<double>(_value));
      }
      else
      {
        gzerr << "Trying to set " << _key << " for non-screw joints."
              << std::endl;
        return 0;
      }
    }
    else if (_key == "gearbox_ratio")
    {
      GearboxJoint<ODEJoint>* gearboxJoint =
        dynamic_cast<GearboxJoint<ODEJoint>* >(this);
      if (gearboxJoint != NULL)
      {
        gearboxJoint->SetGearboxRatio(boost::any_cast<double>(_value));
      }
      else
      {
        gzerr << "Trying to set " << _key << " for non-gearbox joints."
              << std::endl;
        return 0;
      }
    }
    else
    {
      gzerr << "Unable to handle joint attribute[" << _key << "]\n";
      return false;
    }
  }
  catch(const boost::bad_any_cast &e)
  {
    gzerr << "SetParam(" << _key << ")"
          << " boost any_cast error:" << e.what()
          << std::endl;
    return false;
  }
  return true;
}

//////////////////////////////////////////////////
double ODEJoint::Param(const std::string &_key, const unsigned int _index) const
{
  // Axis parameters for multi-axis joints use a group bitmask
  // to identify the variable.
  unsigned int group;
  switch (_index)
  {
    case 0:
      group = dParamGroup1;
      break;
    case 1:
      group = dParamGroup2;
      break;
    case 2:
      group = dParamGroup3;
      break;
    default:
      gzerr << "Invalid index[" << _index << "]\n";
      return false;
  }

  try
  {
    if (_key == "fudge_factor")
    {
      return this->Param(dParamFudgeFactor);
    }
    else if (_key == "suspension_erp")
    {
      return this->Param(dParamSuspensionERP);
    }
    else if (_key == "suspension_cfm")
    {
      return this->Param(dParamSuspensionCFM);
    }
    else if (_key == "stop_erp")
    {
      return this->Param(dParamStopERP | group);
    }
    else if (_key == "stop_cfm")
    {
      return this->Param(dParamStopCFM | group);
    }
    else if (_key == "erp")
    {
      return this->Param(dParamERP);
    }
    else if (_key == "cfm")
    {
      return this->Param(dParamCFM);
    }
    else if (_key == "fmax")
    {
      return this->Param(dParamFMax | group);
    }
    else if (_key == "friction")
    {
      return this->Param(dParamFMax | group);
    }
    else if (_key == "vel")
    {
      return this->Param(dParamVel | group);
    }
    else if (_key == "hi_stop")
    {
      return this->Param(dParamHiStop | group);
    }
    else if (_key == "lo_stop")
    {
      return this->Param(dParamLoStop | group);
    }
  }
  catch(common::Exception &e)
  {
    gzerr << "Param(" << _key << ") error:" << e.GetErrorStr() << std::endl;
    return 0;
  }

  gzerr << "Unable to get joint attribute[" << _key << "]" << std::endl;
  return 0;
}

//////////////////////////////////////////////////
void ODEJoint::Reset()
{
  if (this->odeJointDPtr->jointId)
    dJointReset(this->odeJointDPtr->jointId);
  else
    gzerr << "ODE Joint ID is invalid\n";

  this->forceAppliedTime = common::Time::Zero;

  Joint::Reset();
}

//////////////////////////////////////////////////
void ODEJoint::CacheForceTorque()
{
  // Does nothing for now, will add when recovering pull request #1721
}

//////////////////////////////////////////////////
JointWrench ODEJoint::ForceTorque(const unsigned int /*_index*/) const
{
  // Note that:
  // f2, t2 are the force torque measured on parent body's cg
  // f1, t1 are the force torque measured on child body's cg
  dJointFeedback *fb = this->Feedback();
  if (fb)
  {
    // kind of backwards here, body1 (parent) corresponds go f2, t2
    // and body2 (child) corresponds go f1, t1
    this->jointDPtr->wrench.body2Force.Set(fb->f1[0], fb->f1[1], fb->f1[2]);
    this->jointDPtr->wrench.body2Torque.Set(fb->t1[0], fb->t1[1], fb->t1[2]);
    this->jointDPtr->wrench.body1Force.Set(fb->f2[0], fb->f2[1], fb->f2[2]);
    this->jointDPtr->wrench.body1Torque.Set(fb->t2[0], fb->t2[1], fb->t2[2]);

    // get force applied through SetForce
    physics::JointWrench wrenchAppliedWorld;
    if (this->HasType(physics::Base::HINGE_JOINT))
    {
      // rotate force into child link frame
      // GetLocalAxis is the axis specified in parent link frame!!!
      wrenchAppliedWorld.body2Torque =
        this->Force(0u) * this->LocalAxis(0u);

      // gzerr << "body2Torque [" << wrenchAppliedWorld.body2Torque
      //       << "] axis [" << this->LocalAxis(0u)
      //       << "]\n";

      wrenchAppliedWorld.body1Torque = -wrenchAppliedWorld.body2Torque;
    }
    else if (this->HasType(physics::Base::SLIDER_JOINT))
    {
      // rotate force into child link frame
      wrenchAppliedWorld.body2Force =
        this->Force(0u) * this->LocalAxis(0u);
      wrenchAppliedWorld.body1Force = -wrenchAppliedWorld.body2Force;
    }
    else if (this->HasType(physics::Base::FIXED_JOINT))
    {
      // no correction are necessary for fixed joint
    }
    else
    {
      /// \TODO: fix for multi-axis joints
      gzerr << "force torque for joint type [" << this->Type()
            << "] not implemented, returns false results!!\n";
    }

    // convert wrench from child cg location to child link frame
    if (this->jointDPtr->childLink)
    {
      ignition::math::Pose3d childPose =
        this->jointDPtr->childLink->WorldPose();

      // convert torque from about child CG to joint anchor location
      // cg position specified in child link frame
      ignition::math::Pose3d cgPose;
      const physics::Inertial inertial = this->jointDPtr->childLink->Inertia();
      cgPose = inertial.Pose();

      // anchorPose location of joint in child frame
      // childMomentArm: from child CG to joint location in child link frame
      // moment arm rotated into world frame (given feedback is in world frame)
      ignition::math::Vector3d childMomentArm = childPose.Rot().RotateVector(
        (this->jointDPtr->anchorPose -
         ignition::math::Pose3d(cgPose.Pos(),
           ignition::math::Quaterniond())).Pos());

      // gzerr << "anchor [" << anchorPose
      //       << "] iarm[" <<
      //       this->jointDPtr->childLink->Inertial()->GetPose().pos
      //       << "] childMomentArm[" << childMomentArm
      //       << "] f1[" << this->jointDPtr->wrench.body2Force
      //       << "] t1[" << this->jointDPtr->wrench.body2Torque
      //       << "] fxp[" << this->jointDPtr->wrench.body2Force.Cross(
      //       childMomentArm)
      //       << "]\n";

      this->jointDPtr->wrench.body2Torque +=
        this->jointDPtr->wrench.body2Force.Cross(childMomentArm);

      // rotate resulting body2Force in world frame into link frame
      this->jointDPtr->wrench.body2Force = childPose.Rot().RotateVectorReverse(
        -this->jointDPtr->wrench.body2Force);

      // rotate resulting body2Torque in world frame into link frame
      this->jointDPtr->wrench.body2Torque = childPose.Rot().RotateVectorReverse(
        -this->jointDPtr->wrench.body2Torque);
    }

    // convert torque from about parent CG to joint anchor location
    if (this->jointDPtr->parentLink)
    {
      // get child pose, or it's the inertial world if childLink is NULL
      ignition::math::Pose3d childPose;
      if (this->jointDPtr->childLink)
        childPose = this->jointDPtr->childLink->WorldPose();
      else
        gzerr << "missing child link, double check model.";

      ignition::math::Pose3d parentPose =
        this->jointDPtr->parentLink->WorldPose();

      // if parent link exists, convert torque from about parent
      // CG to joint anchor location

      // parent cg specified in parent link frame
      ignition::math::Pose3d cgPose;
      const Inertial inertial = this->jointDPtr->parentLink->Inertia();
      cgPose = inertial.Pose();

      // get parent CG pose in child link frame
      ignition::math::Pose3d parentCGInChildLink =
        ignition::math::Pose3d(cgPose.Pos(),
            ignition::math::Quaterniond()) - (childPose - parentPose);

      // anchor location in parent CG frame
      // this is the moment arm, but it's in parent CG frame, we need
      // to convert it into world frame
      ignition::math::Pose3d anchorInParendCGFrame =
        this->jointDPtr->anchorPose - parentCGInChildLink;

      // paretnCGFrame in world frame
      ignition::math::Pose3d parentCGInWorld = cgPose + parentPose;

      // rotate momeent arms into world frame
      ignition::math::Vector3d parentMomentArm =
        parentCGInWorld.Rot().RotateVector(
        (this->jointDPtr->anchorPose - parentCGInChildLink).Pos());

      // gzerr << "anchor [" << this->jointDPtr->anchorPose
      //       << "] pcginc[" << parentCGInChildLink
      //       << "] iarm[" << cgPose
      //       << "] anc2pcg[" << this->jointDPtr->anchorPose -
      //       parentCGInChildLink
      //       << "] parentMomentArm[" << parentMomentArm
      //       << "] f1[" << this->jointDPtr->wrench.body1Force
      //       << "] t1[" << this->jointDPtr->wrench.body1Torque
      //       << "] fxp[" << this->jointDPtr->wrench.body1Force.Cross(
      //       parentMomentArm)
      //       << "]\n";

      this->jointDPtr->wrench.body1Torque +=
        this->jointDPtr->wrench.body1Force.Cross(parentMomentArm);

      // rotate resulting body1Force in world frame into link frame
      this->jointDPtr->wrench.body1Force = parentPose.Rot().RotateVectorReverse(
        -this->jointDPtr->wrench.body1Force);

      // rotate resulting body1Torque in world frame into link frame
      this->jointDPtr->wrench.body1Torque =
        parentPose.Rot().RotateVectorReverse(
        -this->jointDPtr->wrench.body1Torque);

      if (!this->jointDPtr->childLink)
      {
        gzlog << "Joint [" << this->ScopedName()
              << "] with parent Link ["
              << this->jointDPtr->parentLink->ScopedName()
              << "] but no child Link.  Child Link must be world.\n";

        // if child link does not exist, use equal and opposite
        this->jointDPtr->wrench.body2Force =
          -this->jointDPtr->wrench.body1Force;
        this->jointDPtr->wrench.body2Torque =
          -this->jointDPtr->wrench.body1Torque;

        // force/torque are in parent link frame, transform them into
        // child link(world) frame.
        ignition::math::Pose3d parentToWorldTransform =
          this->jointDPtr->parentLink->WorldPose();
        this->jointDPtr->wrench.body1Force =
          parentToWorldTransform.Rot().RotateVector(
          this->jointDPtr->wrench.body1Force);
        this->jointDPtr->wrench.body1Torque =
          parentToWorldTransform.Rot().RotateVector(
          this->jointDPtr->wrench.body1Torque);
      }
    }
    else
    {
      if (!this->jointDPtr->childLink)
      {
        gzerr << "Both parent and child links are invalid, abort.\n";
        return JointWrench();
      }
      else
      {
        gzlog << "Joint [" << this->ScopedName()
              << "] with child Link ["
              << this->jointDPtr->childLink->ScopedName()
              << "] but no parent Link.  Parent Link must be world.\n";
        // if parentLink does not exist, use equal opposite body1 wrench
        this->jointDPtr->wrench.body1Force =
          -this->jointDPtr->wrench.body2Force;
        this->jointDPtr->wrench.body1Torque =
          -this->jointDPtr->wrench.body2Torque;

        // force/torque are in child link frame, transform them into
        // parent link frame.  Here, parent link is world, so zero transform.
        ignition::math::Pose3d childToWorldTransform =
          this->jointDPtr->childLink->WorldPose();
        this->jointDPtr->wrench.body1Force =
          childToWorldTransform.Rot().RotateVector(
          this->jointDPtr->wrench.body1Force);
        this->jointDPtr->wrench.body1Torque =
          childToWorldTransform.Rot().RotateVector(
          this->jointDPtr->wrench.body1Torque);
      }
    }
    this->jointDPtr->wrench = this->jointDPtr->wrench - wrenchAppliedWorld;
  }
  else
  {
    // forgot to set provide_feedback?
    gzwarn << "GetForceTorque: forgot to set <provide_feedback>?\n";
  }

  return this->jointDPtr->wrench;
}

//////////////////////////////////////////////////
bool ODEJoint::UsesImplicitSpringDamper() const
{
  return this->odeJointDPtr->useImplicitSpringDamper;
}

//////////////////////////////////////////////////
void ODEJoint::UseImplicitSpringDamper(const bool _implicit)
{
  this->odeJointDPtr->useImplicitSpringDamper = _implicit;
}

//////////////////////////////////////////////////
void ODEJoint::ApplyImplicitStiffnessDamping()
{
  // check if we are violating joint limits
  if (this->AngleCount() > 2)
  {
     gzerr << "Incompatible joint type, AngleCount() = "
           << this->AngleCount() << " > 2\n";
     return;
  }

  double dt = this->World()->Physics()->MaxStepSize();
  for (unsigned int i = 0; i < this->AngleCount(); ++i)
  {
    double angle = this->Angle(i).Radian();
    double dAngle = 2.0 * this->Velocity(i) * dt;
    angle += dAngle;

    if ((ignition::math::equal(
            this->jointDPtr->dissipationCoefficient[i], 0.0) &&
         ignition::math::equal(
           this->jointDPtr->stiffnessCoefficient[i], 0.0)) ||
        angle >= this->jointDPtr->upperLimit[i].Radian() ||
        angle <= this->jointDPtr->lowerLimit[i].Radian())
    {
      if (this->odeJointDPtr->implicitDampingState[i] != ODEJoint::JOINT_LIMIT)
      {
        // We have hit the actual joint limit!
        // turn off simulated damping by recovering cfm and erp,
        // and recover joint limits
        this->SetParam("stop_erp", i, this->odeJointDPtr->stopERP);
        this->SetParam("stop_cfm", i, this->odeJointDPtr->stopCFM);
        this->SetParam("hi_stop", i, this->jointDPtr->upperLimit[i].Radian());
        this->SetParam("lo_stop", i, this->jointDPtr->lowerLimit[i].Radian());
        this->SetParam("hi_stop", i, this->jointDPtr->upperLimit[i].Radian());
        this->odeJointDPtr->implicitDampingState[i] = ODEJoint::JOINT_LIMIT;
      }
      /* test to see if we can reduce jitter at joint limits
      // apply spring damper explicitly if in joint limit
      // this limits oscillations if spring is pushing joint
      // into the limit.
      {
        double dampingForce = -fabs(this->jointDPtr->dissipationCoefficient[i])
          * this->Velocity(i);
        double springForce = this->jointDPtr->stiffnessCoefficient[i]
          * (this->jointDPtr->springReferencePosition[i] - this->Angle(i).Radian());
        this->SetForceImpl(i, dampingForce + springForce);
      }
      */
    }
    else if (!ignition::math::equal(
          this->jointDPtr->dissipationCoefficient[i], 0.0) ||
        !ignition::math::equal(this->jointDPtr->stiffnessCoefficient[i], 0.0))
    {
      double kd = fabs(this->jointDPtr->dissipationCoefficient[i]);
      double kp = this->jointDPtr->stiffnessCoefficient[i];

      /// \TODO: This bit of code involving adaptive damping
      /// might be too complicated, add some more comments or simplify it.
      if (this->jointDPtr->dissipationCoefficient[i] < 0)
        kd = this->ApplyAdaptiveDamping(i, kd);

      // update if going into DAMPING_ACTIVE mode, or
      // if current applied damping value is not the same as predicted.
      if (this->odeJointDPtr->implicitDampingState[i] !=
          ODEJoint::DAMPING_ACTIVE ||
          !ignition::math::equal(kd, this->odeJointDPtr->currentKd[i]) ||
          !ignition::math::equal(kp, this->odeJointDPtr->currentKp[i]))
      {
        // save kp, kd applied for efficiency
        this->odeJointDPtr->currentKd[i] = kd;
        this->odeJointDPtr->currentKp[i] = kp;

        // convert kp, kd to cfm, erp
        double erp, cfm;
        this->KpKdToCFMERP(dt, kp, kd, cfm, erp);

        // add additional constraint row by fake hitting joint limit
        // then, set erp and cfm to simulate viscous joint damping
        this->SetParam("stop_erp", i, erp);
        this->SetParam("stop_cfm", i, cfm);
        this->SetParam("hi_stop", i,
            this->jointDPtr->springReferencePosition[i]);
        this->SetParam("lo_stop", i,
            this->jointDPtr->springReferencePosition[i]);
        this->SetParam("hi_stop", i,
            this->jointDPtr->springReferencePosition[i]);
        this->odeJointDPtr->implicitDampingState[i] = ODEJoint::DAMPING_ACTIVE;
      }
    }
  }
}

//////////////////////////////////////////////////
double ODEJoint::ApplyAdaptiveDamping(const unsigned int _index,
    const double _damping)
{
  /// \TODO: hardcoded thresholds for now, make them params.
  static double vThreshold = 0.01;
  static double fThreshold = 1.0;

  double f = this->Force(_index);
  double v = this->Velocity(_index);

  if (fabs(v) < vThreshold && fabs(f) > fThreshold)
  {
    // guess what the stable damping value might be based on v.
    double tmpDStable = f / (v/fabs(v)*std::max(fabs(v), vThreshold));

    // debug
    // gzerr << "joint [" << this->Name()
    //       << "] damping[" << _damping
    //       << "] f [" << f
    //       << "] v [" << v
    //       << "] f*v [" << f*v
    //       << "] f/v [" << tmpDStable
    //       << "] cur currentKd[" << _index
    //       << "] = [" << currentKd[_index] << "]\n";

    // limit v(n+1)/v(n) to 2.0 by multiplying tmpDStable by 0.5
    return std::max(_damping, 0.5*tmpDStable);
  }
  else
    return _damping;
}

//////////////////////////////////////////////////
void ODEJoint::KpKdToCFMERP(const double _dt,
                            const double _kp, const double _kd,
                            double &_cfm, double &_erp) const
{
  /// \TODO: check for NaN cases
  _erp = _dt * _kp / (_dt * _kp + _kd);
  _cfm = 1.0 / (_dt * _kp + _kd);
}

//////////////////////////////////////////////////
void ODEJoint::CFMERPToKpKd(const double _dt,
                            const double _cfm, const double _erp,
                            double &_kp, double &_kd) const
{
  /// \TODO: check for NaN cases
  _kp = _erp / (_dt * _cfm);
  _kd = (1.0 - _erp) / _cfm;
}

//////////////////////////////////////////////////
void ODEJoint::SetDamping(const unsigned int _index, const double _damping)
{
  if (_index < this->AngleCount())
  {
    this->SetStiffnessDamping(_index,
        this->jointDPtr->stiffnessCoefficient[_index], _damping);
  }
  else
  {
     gzerr << "ODEJoint::SetDamping: index[" << _index
           << "] is out of bounds (AngleCount() = "
           << this->AngleCount() << ").\n";
     return;
  }
}

//////////////////////////////////////////////////
void ODEJoint::SetStiffness(const unsigned int _index, const double _stiffness)
{
  if (_index < this->AngleCount())
  {
    this->SetStiffnessDamping(_index, _stiffness,
      this->jointDPtr->dissipationCoefficient[_index]);
  }
  else
  {
     gzerr << "ODEJoint::SetStiffness: index[" << _index
           << "] is out of bounds (AngleCount() = "
           << this->AngleCount() << ").\n";
     return;
  }
}

//////////////////////////////////////////////////
void ODEJoint::SetStiffnessDamping(const unsigned int _index,
  const double _stiffness, const double _damping, const double _reference)
{
  if (_index < this->AngleCount())
  {
    this->jointDPtr->stiffnessCoefficient[_index] = _stiffness;
    this->jointDPtr->dissipationCoefficient[_index] = _damping;
    this->jointDPtr->springReferencePosition[_index] = _reference;

    /// reset state of implicit damping state machine.
    if (this->odeJointDPtr->useImplicitSpringDamper)
    {
      if (static_cast<unsigned int>(_index) < this->AngleCount())
      {
        this->odeJointDPtr->implicitDampingState[_index] = ODEJoint::NONE;
      }
      else
      {
         gzerr << "Incompatible joint type, index[" << _index
               << "] is out of bounds (AngleCount() = "
               << this->AngleCount() << ").\n";
         return;
      }
    }

    /// \TODO:  The check for static parent or child below might not be needed,
    /// but we need to test first.  In theory, attaching an object to a static
    /// body should not affect spring/damper application.
    bool parentStatic =
      this->Parent() ? this->Parent()->IsStatic() : false;
    bool childStatic =
      this->Child() ? this->Child()->IsStatic() : false;

    if (!this->odeJointDPtr->stiffnessDampingInitialized)
    {
      if (!parentStatic && !childStatic)
      {
        this->odeJointDPtr->applyDamping = physics::Joint::ConnectJointUpdate(
          std::bind(&ODEJoint::ApplyStiffnessDamping, this));
        this->odeJointDPtr->stiffnessDampingInitialized = true;
      }
      else
      {
        gzwarn << "Spring Damper for Joint[" << this->Name()
               << "] is not initialized because either parent[" << parentStatic
               << "] or child[" << childStatic << "] is static.\n";
      }
    }
  }
  else
    gzerr << "SetStiffnessDamping _index too large.\n";
}

//////////////////////////////////////////////////
void ODEJoint::SetProvideFeedback(bool _enable)
{
  Joint::SetProvideFeedback(_enable);

  if (this->jointDPtr->provideFeedback)
  {
    if (this->odeJointDPtr->feedback == NULL)
    {
      this->odeJointDPtr->feedback = new dJointFeedback;
      this->odeJointDPtr->feedback->f1[0] = 0;
      this->odeJointDPtr->feedback->f1[1] = 0;
      this->odeJointDPtr->feedback->f1[2] = 0;
      this->odeJointDPtr->feedback->t1[0] = 0;
      this->odeJointDPtr->feedback->t1[1] = 0;
      this->odeJointDPtr->feedback->t1[2] = 0;
      this->odeJointDPtr->feedback->f2[0] = 0;
      this->odeJointDPtr->feedback->f2[1] = 0;
      this->odeJointDPtr->feedback->f2[2] = 0;
      this->odeJointDPtr->feedback->t2[0] = 0;
      this->odeJointDPtr->feedback->t2[1] = 0;
      this->odeJointDPtr->feedback->t2[2] = 0;
    }

    if (this->odeJointDPtr->jointId)
    {
      dJointSetFeedback(this->odeJointDPtr->jointId,
          this->odeJointDPtr->feedback);
    }
    else
    {
      gzerr << "ODE Joint ID is invalid\n";
    }
  }
}

//////////////////////////////////////////////////
void ODEJoint::SetForce(unsigned int _index, double _force)
{
  double force = Joint::CheckAndTruncateForce(_index, _force);
  this->SaveForce(_index, force);
  this->SetForceImpl(_index, force);

  // for engines that supports auto-disable of links
  if (this->jointDPtr->childLink)
    this->jointDPtr->childLink->SetEnabled(true);
  if (this->jointDPtr->parentLink)
    this->jointDPtr->parentLink->SetEnabled(true);
}

//////////////////////////////////////////////////
void ODEJoint::SaveForce(unsigned int _index, double _force)
{
  // this bit of code actually doesn't do anything physical,
  // it simply records the forces commanded inside forceApplied.
  if (_index < this->AngleCount())
  {
    if (this->odeJointDPtr->forceAppliedTime < this->World()->SimTime())
    {
      // reset forces if time step is new
      this->odeJointDPtr->forceAppliedTime = this->World()->SimTime();
      this->odeJointDPtr->forceApplied[0] =
        this->odeJointDPtr->forceApplied[1] = 0;
    }

    this->odeJointDPtr->forceApplied[_index] += _force;
  }
  else
    gzerr << "Something's wrong, joint [" << this->ScopedName()
          << "] index [" << _index
          << "] out of range.\n";
}

//////////////////////////////////////////////////
double ODEJoint::Force(const unsigned int _index) const
{
  if (_index < this->AngleCount())
  {
    return this->odeJointDPtr->forceApplied[_index];
  }
  else
  {
    gzerr << "Invalid joint index [" << _index
          << "] when trying to get force\n";
    return 0;
  }
}

//////////////////////////////////////////////////
void ODEJoint::ApplyStiffnessDamping()
{
  if (this->odeJointDPtr->useImplicitSpringDamper)
    this->ApplyImplicitStiffnessDamping();
  else
    this->ApplyExplicitStiffnessDamping();
}

//////////////////////////////////////////////////
void ODEJoint::ApplyExplicitStiffnessDamping()
{
  for (unsigned int i = 0; i < this->AngleCount(); ++i)
  {
    // Take absolute value of dissipationCoefficient, since negative values of
    // dissipationCoefficient are used for adaptive damping to
    // enforce stability.
    double dampingForce = -fabs(this->jointDPtr->dissipationCoefficient[i])
      * this->Velocity(i);

    double springForce = this->jointDPtr->stiffnessCoefficient[i]
      * (this->jointDPtr->springReferencePosition[i] - this->Angle(i).Radian());

    // do not change forceApplied if setting internal damping forces
    this->SetForceImpl(i, dampingForce + springForce);

    // gzerr << this->Velocity(0) << " : " << dampingForce << "\n";
  }
}

//////////////////////////////////////////////////
bool ODEJoint::SetPosition(const unsigned int _index, const double _position)
{
  return Joint::SetPositionMaximal(_index, _position);
}

//////////////////////////////////////////////////
double ODEJoint::GetStopERP()
{
  return this->StopERP();
}

//////////////////////////////////////////////////
double ODEJoint::StopERP() const
{
  return this->odeJointDPtr->stopERP;
}

//////////////////////////////////////////////////
double ODEJoint::GetStopCFM()
{
  return this->StopCFM();
}

//////////////////////////////////////////////////
double ODEJoint::StopCFM() const
{
  return this->odeJointDPtr->stopCFM;
}
