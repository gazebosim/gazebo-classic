/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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
#include <boost/bind/bind.hpp>
#include "gazebo/common/Exception.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Assert.hh"

#include "gazebo/physics/World.hh"
#include "gazebo/physics/Link.hh"
#include "gazebo/physics/PhysicsEngine.hh"
#include "gazebo/physics/ode/ODELink.hh"
#include "gazebo/physics/ode/ODEJoint.hh"
#include "gazebo/physics/GearboxJoint.hh"
#include "gazebo/physics/ScrewJoint.hh"
#include "gazebo/physics/JointWrench.hh"

// Needed for fixing issue 2430
#include "gazebo/physics/ode/ODEHingeJoint.hh"

using namespace gazebo;
using namespace physics;


//////////////////////////////////////////////////
ODEJoint::ODEJoint(BasePtr _parent)
  : Joint(_parent)
{
  this->jointId = nullptr;
  this->implicitDampingState[0] = ODEJoint::NONE;
  this->implicitDampingState[1] = ODEJoint::NONE;
  this->stiffnessDampingInitialized = false;
  this->feedback = nullptr;
  this->currentKd[0] = 0;
  this->currentKd[1] = 0;
  this->currentKp[0] = 0;
  this->currentKp[1] = 0;
  this->forceApplied[0] = 0;
  this->forceApplied[1] = 0;
  this->useImplicitSpringDamper = false;
  this->stopERP = 0.0;
  this->stopCFM = 0.0;
  this->angleOffset[0] = 0.0;
  this->angleOffset[1] = 0.0;
}

//////////////////////////////////////////////////
ODEJoint::~ODEJoint()
{
  this->Fini();
}

//////////////////////////////////////////////////
void ODEJoint::Fini()
{
  this->applyDamping.reset();

  if (this->feedback)
    delete this->feedback;
  this->feedback = nullptr;

  this->Detach();

  if (this->jointId)
    dJointDestroy(this->jointId);
  this->jointId = nullptr;

  Joint::Fini();
}

//////////////////////////////////////////////////
void ODEJoint::Load(sdf::ElementPtr _sdf)
{
  Joint::Load(_sdf);

  if (this->sdf->HasElement("physics") &&
      this->sdf->GetElement("physics")->HasElement("ode"))
  {
    sdf::ElementPtr elem = this->sdf->GetElement("physics")->GetElement("ode");

    if (elem->HasElement("implicit_spring_damper"))
    {
      this->useImplicitSpringDamper = elem->Get<bool>("implicit_spring_damper");
    }

    // initializa both axis, \todo: make cfm, erp per axis
    this->stopERP = elem->GetElement("limit")->Get<double>("erp");
    for (unsigned int i = 0; i < this->DOF(); ++i)
      this->SetParam("stop_erp", i, this->stopERP);

    // initializa both axis, \todo: make cfm, erp per axis
    this->stopCFM = elem->GetElement("limit")->Get<double>("cfm");
    for (unsigned int i = 0; i < this->DOF(); ++i)
      this->SetParam("stop_cfm", i, this->stopCFM);

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
LinkPtr ODEJoint::GetJointLink(unsigned int _index) const
{
  LinkPtr result;
  if (!this->jointId)
  {
    gzerr << "ODE Joint ID is invalid\n";
    return result;
  }

  if (_index == 0 || _index == 1)
  {
    ODELinkPtr odeLink1 = boost::static_pointer_cast<ODELink>(this->childLink);
    ODELinkPtr odeLink2 = boost::static_pointer_cast<ODELink>(this->parentLink);
    if (odeLink1 != nullptr &&
        dJointGetBody(this->jointId, _index) == odeLink1->GetODEId())
      result = this->childLink;
    else if (odeLink2)
      result = this->parentLink;
  }

  return result;
}

//////////////////////////////////////////////////
bool ODEJoint::AreConnected(LinkPtr _one, LinkPtr _two) const
{
  ODELinkPtr odeLink1 = boost::dynamic_pointer_cast<ODELink>(_one);
  ODELinkPtr odeLink2 = boost::dynamic_pointer_cast<ODELink>(_two);

  if (odeLink1 == nullptr || odeLink2 == nullptr)
    gzthrow("ODEJoint requires ODE bodies\n");

  return dAreConnected(odeLink1->GetODEId(), odeLink2->GetODEId());
}

//////////////////////////////////////////////////
// child classes where appropriate
double ODEJoint::GetParam(unsigned int /*parameter*/) const
{
  return 0;
}

//////////////////////////////////////////////////
void ODEJoint::Attach(LinkPtr _parent, LinkPtr _child)
{
  Joint::Attach(_parent, _child);

  ODELinkPtr odechild = boost::dynamic_pointer_cast<ODELink>(this->childLink);
  ODELinkPtr odeparent = boost::dynamic_pointer_cast<ODELink>(this->parentLink);

  if (odechild == nullptr && odeparent == nullptr)
    gzthrow("ODEJoint requires at least one ODE link\n");

  if (!this->jointId)
    gzerr << "ODE Joint ID is invalid\n";

  if (this->HasType(Base::HINGE2_JOINT) &&
      (odechild == nullptr || odeparent == nullptr))
    gzthrow("ODEHinge2Joint cannot be connected to the world");

  if (!odechild && odeparent)
  {
    dJointAttach(this->jointId, 0, odeparent->GetODEId());
  }
  else if (odechild && !odeparent)
  {
    dJointAttach(this->jointId, odechild->GetODEId(), 0);
  }
  else if (odechild && odeparent)
  {
    if (this->HasType(Base::HINGE2_JOINT))
      dJointAttach(this->jointId, odeparent->GetODEId(), odechild->GetODEId());
    else
      dJointAttach(this->jointId, odechild->GetODEId(), odeparent->GetODEId());
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
  if ((odeParent == nullptr || odeParent->GetODEId() == nullptr) ||
      (odeChild == nullptr || odeChild->GetODEId() == nullptr))
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
void ODEJoint::SetParam(unsigned int /*parameter*/, double /*value*/)
{
  if (this->childLink)
    this->childLink->SetEnabled(true);
  if (this->parentLink)
    this->parentLink->SetEnabled(true);
}

//////////////////////////////////////////////////
void ODEJoint::SetERP(double _newERP)
{
  this->SetParam(dParamSuspensionERP, _newERP);
}

//////////////////////////////////////////////////
double ODEJoint::GetERP()
{
  return this->GetParam(dParamSuspensionERP);
}

//////////////////////////////////////////////////
void ODEJoint::SetCFM(double _newCFM)
{
  this->SetParam(dParamSuspensionCFM, _newCFM);
}

//////////////////////////////////////////////////
double ODEJoint::GetCFM()
{
  return this->GetParam(dParamSuspensionCFM);
}

//////////////////////////////////////////////////
dJointFeedback *ODEJoint::GetFeedback()
{
  if (this->jointId)
    return dJointGetFeedback(this->jointId);
  else
    gzerr << "ODE Joint ID is invalid\n";
  return nullptr;
}

//////////////////////////////////////////////////
void ODEJoint::SetUpperLimit(const unsigned int _index, const double _limit)
{
  Joint::SetUpperLimit(_index, _limit);
  switch (_index)
  {
    case 0:
      this->SetParam(dParamHiStop, _limit);
      break;
    case 1:
      this->SetParam(dParamHiStop2, _limit);
      break;
    case 2:
      this->SetParam(dParamHiStop3, _limit);
      break;
    default:
      gzerr << "Invalid index[" << _index << "]\n";
  };
}

//////////////////////////////////////////////////
void ODEJoint::SetLowerLimit(const unsigned int _index, const double _limit)
{
  Joint::SetLowerLimit(_index, _limit);
  switch (_index)
  {
    case 0:
      this->SetParam(dParamLoStop, _limit);
      break;
    case 1:
      this->SetParam(dParamLoStop2, _limit);
      break;
    case 2:
      this->SetParam(dParamLoStop3, _limit);
      break;
    default:
      gzerr << "Invalid index[" << _index << "]\n";
  };
}

//////////////////////////////////////////////////
ignition::math::Vector3d ODEJoint::LinkForce(
          const unsigned int _index) const
{
  ignition::math::Vector3d result;

  if (!this->jointId)
  {
    gzerr << "ODE Joint ID is invalid\n";
    return result;
  }

  dJointFeedback *jointFeedback = dJointGetFeedback(this->jointId);

  if (_index == 0)
    result.Set(jointFeedback->f1[0], jointFeedback->f1[1],
               jointFeedback->f1[2]);
  else
    result.Set(jointFeedback->f2[0], jointFeedback->f2[1],
               jointFeedback->f2[2]);

  return result;
}

//////////////////////////////////////////////////
ignition::math::Vector3d ODEJoint::LinkTorque(
          const unsigned int _index) const
{
  ignition::math::Vector3d result;

  if (!this->jointId)
  {
    gzerr << "ODE Joint ID is invalid\n";
    return result;
  }

  dJointFeedback *jointFeedback = dJointGetFeedback(this->jointId);
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
void ODEJoint::SetAxis(const unsigned int _index,
    const ignition::math::Vector3d &_axis)
{
}

//////////////////////////////////////////////////
bool ODEJoint::SetParam(const std::string &_key, unsigned int _index,
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
      if (screwJoint != nullptr)
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
      if (gearboxJoint != nullptr)
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
double ODEJoint::GetParam(const std::string &_key, unsigned int _index)
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
      return this->GetParam(dParamFudgeFactor);
    }
    else if (_key == "suspension_erp")
    {
      return this->GetParam(dParamSuspensionERP);
    }
    else if (_key == "suspension_cfm")
    {
      return this->GetParam(dParamSuspensionCFM);
    }
    else if (_key == "stop_erp")
    {
      return this->GetParam(dParamStopERP | group);
    }
    else if (_key == "stop_cfm")
    {
      return this->GetParam(dParamStopCFM | group);
    }
    else if (_key == "erp")
    {
      return this->GetParam(dParamERP);
    }
    else if (_key == "cfm")
    {
      return this->GetParam(dParamCFM);
    }
    else if (_key == "fmax")
    {
      return this->GetParam(dParamFMax | group);
    }
    else if (_key == "friction")
    {
      return this->GetParam(dParamFMax | group);
    }
    else if (_key == "vel")
    {
      return this->GetParam(dParamVel | group);
    }
    else if (_key == "hi_stop")
    {
          return this->GetParam(dParamHiStop | group);
    }
    else if (_key == "lo_stop")
    {
          return this->GetParam(dParamLoStop | group);
    }
    else if (_key == "thread_pitch")
    {
      ScrewJoint<ODEJoint>* screwJoint =
        dynamic_cast<ScrewJoint<ODEJoint>* >(this);
      if (screwJoint != nullptr)
      {
          return screwJoint->GetThreadPitch();
      }
      else
      {
        gzerr << "Trying to get " << _key << " for non-screw joints."
              << std::endl;
        return 0;
      }
    }
    else if (_key == "gearbox_ratio")
    {
      GearboxJoint<ODEJoint>* gearboxJoint =
        dynamic_cast<GearboxJoint<ODEJoint>* >(this);
      if (gearboxJoint != nullptr)
      {
        return gearboxJoint->GetGearboxRatio();
      }
      else
      {
        gzerr << "Trying to get " << _key << " for non-gearbox joints."
              << std::endl;
        return 0;
      }
    }
  }
  catch(common::Exception &e)
  {
    gzerr << "GetParam(" << _key << ") error:"
          << e.GetErrorStr()
          << std::endl;
    return 0;
  }

  gzerr << "Unable to get joint attribute[" << _key << "]"
        << std::endl;
  return 0;
}

//////////////////////////////////////////////////
void ODEJoint::Reset()
{
  if (this->jointId)
    dJointReset(this->jointId);
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
JointWrench ODEJoint::GetForceTorque(unsigned int /*_index*/)
{
  // Note that:
  // f2, t2 are the force torque measured on parent body's cg
  // f1, t1 are the force torque measured on child body's cg
  dJointFeedback *fb = this->GetFeedback();
  if (fb)
  {
    // kind of backwards here, body1 (parent) corresponds go f2, t2
    // and body2 (child) corresponds go f1, t1
    this->wrench.body2Force.Set(fb->f1[0], fb->f1[1], fb->f1[2]);
    this->wrench.body2Torque.Set(fb->t1[0], fb->t1[1], fb->t1[2]);
    this->wrench.body1Force.Set(fb->f2[0], fb->f2[1], fb->f2[2]);
    this->wrench.body1Torque.Set(fb->t2[0], fb->t2[1], fb->t2[2]);

    // get force applied through SetForce
    physics::JointWrench wrenchAppliedWorld;
    if (this->HasType(physics::Base::HINGE_JOINT))
    {
      // rotate force into child link frame
      // LocalAxis is the axis specified in parent link frame!!!
      wrenchAppliedWorld.body2Torque =
        this->GetForce(0u) * this->LocalAxis(0u);

      // gzerr << "body2Torque [" << wrenchAppliedWorld.body2Torque
      //       << "] axis [" << this->LocalAxis(0u)
      //       << "]\n";

      wrenchAppliedWorld.body1Torque = -wrenchAppliedWorld.body2Torque;
    }
    else if (this->HasType(physics::Base::SLIDER_JOINT))
    {
      // rotate force into child link frame
      wrenchAppliedWorld.body2Force =
        this->GetForce(0u) * this->LocalAxis(0u);
      wrenchAppliedWorld.body1Force = -wrenchAppliedWorld.body2Force;
    }
    else if (this->HasType(physics::Base::FIXED_JOINT))
    {
      // no correction are necessary for fixed joint
    }
    else
    {
      /// \TODO: fix for multi-axis joints
      gzerr << "force torque for joint type [" << this->GetType()
            << "] not implemented, returns false results!!\n";
    }

    // convert wrench from child cg location to child link frame
    if (this->childLink)
    {
      ignition::math::Pose3d childPose = this->childLink->WorldPose();

      // convert torque from about child CG to joint anchor location
      // cg position specified in child link frame
      ignition::math::Pose3d cgPose;
      auto inertial = this->childLink->GetInertial();
      if (inertial)
        cgPose = inertial->Pose();

      // anchorPose location of joint in child frame
      // childMomentArm: from child CG to joint location in child link frame
      // moment arm rotated into world frame (given feedback is in world frame)
      ignition::math::Vector3d childMomentArm = childPose.Rot().RotateVector(
        (this->anchorPose -
         ignition::math::Pose3d(cgPose.Pos(),
           ignition::math::Quaterniond::Identity)).Pos());

      // gzerr << "anchor [" << anchorPose
      //       << "] iarm[" << this->childLink->GetInertial()->GetPose().Pos()
      //       << "] childMomentArm[" << childMomentArm
      //       << "] f1[" << this->wrench.body2Force
      //       << "] t1[" << this->wrench.body2Torque
      //       << "] fxp[" << this->wrench.body2Force.Cross(childMomentArm)
      //       << "]\n";

      this->wrench.body2Torque += this->wrench.body2Force.Cross(childMomentArm);

      // rotate resulting body2Force in world frame into link frame
      this->wrench.body2Force = childPose.Rot().RotateVectorReverse(
        -this->wrench.body2Force);

      // rotate resulting body2Torque in world frame into link frame
      this->wrench.body2Torque = childPose.Rot().RotateVectorReverse(
        -this->wrench.body2Torque);
    }

    // convert torque from about parent CG to joint anchor location
    if (this->parentLink)
    {
      // get child pose, or it's the inertial world if childLink is nullptr
      ignition::math::Pose3d childPose;
      if (this->childLink)
        childPose = this->childLink->WorldPose();
      else
        gzerr << "missing child link, double check model.";

      ignition::math::Pose3d parentPose = this->parentLink->WorldPose();

      // if parent link exists, convert torque from about parent
      // CG to joint anchor location

      // parent cg specified in parent link frame
      ignition::math::Pose3d cgPose;
      auto inertial = this->parentLink->GetInertial();
      if (inertial)
        cgPose = inertial->Pose();

      // get parent CG pose in child link frame
      ignition::math::Pose3d parentCGInChildLink =
        ignition::math::Pose3d(cgPose.Pos(),
            ignition::math::Quaterniond::Identity) - (childPose - parentPose);

      // paretnCGFrame in world frame
      ignition::math::Pose3d parentCGInWorld = cgPose + parentPose;

      // rotate momeent arms into world frame
      ignition::math::Vector3d parentMomentArm =
        parentCGInWorld.Rot().RotateVector(
            (this->anchorPose - parentCGInChildLink).Pos());

      // gzerr << "anchor [" << this->anchorPose
      //       << "] pcginc[" << parentCGInChildLink
      //       << "] iarm[" << cgPose
      //       << "] anc2pcg[" << this->anchorPose - parentCGInChildLink
      //       << "] parentMomentArm[" << parentMomentArm
      //       << "] f1[" << this->wrench.body1Force
      //       << "] t1[" << this->wrench.body1Torque
      //       << "] fxp[" << this->wrench.body1Force.Cross(parentMomentArm)
      //       << "]\n";

      this->wrench.body1Torque +=
        this->wrench.body1Force.Cross(parentMomentArm);

      // rotate resulting body1Force in world frame into link frame
      this->wrench.body1Force = parentPose.Rot().RotateVectorReverse(
        -this->wrench.body1Force);

      // rotate resulting body1Torque in world frame into link frame
      this->wrench.body1Torque = parentPose.Rot().RotateVectorReverse(
        -this->wrench.body1Torque);

      if (!this->childLink)
      {
        gzlog << "Joint [" << this->GetScopedName()
              << "] with parent Link [" << this->parentLink->GetScopedName()
              << "] but no child Link.  Child Link must be world.\n";
        // if child link does not exist, use equal and opposite
        this->wrench.body2Force = -this->wrench.body1Force;
        this->wrench.body2Torque = -this->wrench.body1Torque;

        // force/torque are in parent link frame, transform them into
        // child link(world) frame.
        auto parentToWorldTransform = this->parentLink->WorldPose();
        this->wrench.body1Force =
          parentToWorldTransform.Rot().RotateVector(
          this->wrench.body1Force);
        this->wrench.body1Torque =
          parentToWorldTransform.Rot().RotateVector(
          this->wrench.body1Torque);
      }
    }
    else
    {
      if (!this->childLink)
      {
        gzerr << "Both parent and child links are invalid, abort.\n";
        return JointWrench();
      }
      else
      {
        gzlog << "Joint [" << this->GetScopedName()
              << "] with child Link [" << this->childLink->GetScopedName()
              << "] but no parent Link.  Parent Link must be world.\n";
        // if parentLink does not exist, use equal opposite body1 wrench
        this->wrench.body1Force = -this->wrench.body2Force;
        this->wrench.body1Torque = -this->wrench.body2Torque;

        // force/torque are in child link frame, transform them into
        // parent link frame.  Here, parent link is world, so zero transform.
        auto childToWorldTransform = this->childLink->WorldPose();
        this->wrench.body1Force =
          childToWorldTransform.Rot().RotateVector(
          this->wrench.body1Force);
        this->wrench.body1Torque =
          childToWorldTransform.Rot().RotateVector(
          this->wrench.body1Torque);
      }
    }
    this->wrench = this->wrench - wrenchAppliedWorld;
  }
  else
  {
    // forgot to set provide_feedback?
    gzwarn << "GetForceTorque: forgot to set <provide_feedback>?\n";
  }

  return this->wrench;
}

//////////////////////////////////////////////////
bool ODEJoint::UsesImplicitSpringDamper()
{
  return this->useImplicitSpringDamper;
}

//////////////////////////////////////////////////
void ODEJoint::UseImplicitSpringDamper(const bool _implicit)
{
  this->useImplicitSpringDamper = _implicit;
}

//////////////////////////////////////////////////
void ODEJoint::ApplyImplicitStiffnessDamping()
{
  // check if we are violating joint limits
  if (this->DOF() > 2)
  {
     gzerr << "Incompatible joint type, DOF() = "
           << this->DOF() << " > 2\n";
     return;
  }

  double dt = this->GetWorld()->Physics()->GetMaxStepSize();
  for (unsigned int i = 0; i < this->DOF(); ++i)
  {
    double angle = this->Position(i);
    double dAngle = 2.0 * this->GetVelocity(i) * dt;
    angle += dAngle;

    if ((ignition::math::equal(this->dissipationCoefficient[i], 0.0) &&
         ignition::math::equal(this->stiffnessCoefficient[i], 0.0)) ||
        angle >= this->upperLimit[i] ||
        angle <= this->lowerLimit[i])
    {
      if (this->implicitDampingState[i] != ODEJoint::JOINT_LIMIT)
      {
        // We have hit the actual joint limit!
        // turn off simulated damping by recovering cfm and erp,
        // and recover joint limits
        this->SetParam("stop_erp", i, this->stopERP);
        this->SetParam("stop_cfm", i, this->stopCFM);
        this->SetParam("hi_stop", i, this->upperLimit[i]);
        this->SetParam("lo_stop", i, this->lowerLimit[i]);
        this->SetParam("hi_stop", i, this->upperLimit[i]);
        this->implicitDampingState[i] = ODEJoint::JOINT_LIMIT;
      }
      /* test to see if we can reduce jitter at joint limits
      // apply spring damper explicitly if in joint limit
      // this limits oscillations if spring is pushing joint
      // into the limit.
      {
        double dampingForce = -fabs(this->dissipationCoefficient[i])
          * this->GetVelocity(i);
        double springForce = this->stiffnessCoefficient[i]
          * (this->springReferencePosition[i] - this->Position(i));
        this->SetForceImpl(i, dampingForce + springForce);
      }
      */
    }
    else if (!ignition::math::equal(this->dissipationCoefficient[i], 0.0) ||
             !ignition::math::equal(this->stiffnessCoefficient[i], 0.0))
    {
      double kd = fabs(this->dissipationCoefficient[i]);
      double kp = this->stiffnessCoefficient[i];

      /// \TODO: This bit of code involving adaptive damping
      /// might be too complicated, add some more comments or simplify it.
      if (this->dissipationCoefficient[i] < 0)
        kd = this->ApplyAdaptiveDamping(i, kd);

      // update if going into DAMPING_ACTIVE mode, or
      // if current applied damping value is not the same as predicted.
      if (this->implicitDampingState[i] != ODEJoint::DAMPING_ACTIVE ||
          !ignition::math::equal(kd, this->currentKd[i]) ||
          !ignition::math::equal(kp, this->currentKp[i]))
      {
        // save kp, kd applied for efficiency
        this->currentKd[i] = kd;
        this->currentKp[i] = kp;

        // convert kp, kd to cfm, erp
        double erp, cfm;
        this->KpKdToCFMERP(dt, kp, kd, cfm, erp);

        // add additional constraint row by fake hitting joint limit
        // then, set erp and cfm to simulate viscous joint damping
        this->SetParam("stop_erp", i, erp);
        this->SetParam("stop_cfm", i, cfm);
        this->SetParam("hi_stop", i, this->springReferencePosition[i]);
        this->SetParam("lo_stop", i, this->springReferencePosition[i]);
        this->SetParam("hi_stop", i, this->springReferencePosition[i]);
        this->implicitDampingState[i] = ODEJoint::DAMPING_ACTIVE;
      }
    }
  }
}

//////////////////////////////////////////////////
double ODEJoint::ApplyAdaptiveDamping(unsigned int _index,
    const double _damping)
{
  /// \TODO: hardcoded thresholds for now, make them params.
  static double vThreshold = 0.01;
  static double fThreshold = 1.0;

  double f = this->GetForce(_index);
  double v = this->GetVelocity(_index);

  if (fabs(v) < vThreshold && fabs(f) > fThreshold)
  {
    // guess what the stable damping value might be based on v.
    double tmpDStable = f / (v/fabs(v)*std::max(fabs(v), vThreshold));

    // debug
    // gzerr << "joint [" << this->GetName()
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
                           double &_cfm, double &_erp)
{
  /// \TODO: check for NaN cases
  _erp = _dt * _kp / (_dt * _kp + _kd);
  _cfm = 1.0 / (_dt * _kp + _kd);
}

//////////////////////////////////////////////////
void ODEJoint::CFMERPToKpKd(const double _dt,
                           const double _cfm, const double _erp,
                           double &_kp, double &_kd)
{
  /// \TODO: check for NaN cases
  _kp = _erp / (_dt * _cfm);
  _kd = (1.0 - _erp) / _cfm;
}

//////////////////////////////////////////////////
void ODEJoint::SetDamping(unsigned int _index, double _damping)
{
  if (_index < this->DOF())
  {
    this->SetStiffnessDamping(_index, this->stiffnessCoefficient[_index],
      _damping);
  }
  else
  {
     gzerr << "ODEJoint::SetDamping: index[" << _index
           << "] is out of bounds (DOF() = "
           << this->DOF() << ").\n";
     return;
  }
}

//////////////////////////////////////////////////
void ODEJoint::SetStiffness(unsigned int _index, const double _stiffness)
{
  if (_index < this->DOF())
  {
    this->SetStiffnessDamping(_index, _stiffness,
      this->dissipationCoefficient[_index]);
  }
  else
  {
     gzerr << "ODEJoint::SetStiffness: index[" << _index
           << "] is out of bounds (DOF() = "
           << this->DOF() << ").\n";
     return;
  }
}

//////////////////////////////////////////////////
void ODEJoint::SetStiffnessDamping(unsigned int _index,
  double _stiffness, double _damping, double _reference)
{
  if (_index < this->DOF())
  {
    this->stiffnessCoefficient[_index] = _stiffness;
    this->dissipationCoefficient[_index] = _damping;
    this->springReferencePosition[_index] = _reference;

    /// reset state of implicit damping state machine.
    if (this->useImplicitSpringDamper)
    {
      if (static_cast<unsigned int>(_index) < this->DOF())
      {
        this->implicitDampingState[_index] = ODEJoint::NONE;
      }
      else
      {
         gzerr << "Incompatible joint type, index[" << _index
               << "] is out of bounds (DOF() = "
               << this->DOF() << ").\n";
         return;
      }
    }

    /// \TODO:  The check for static parent or child below might not be needed,
    /// but we need to test first.  In theory, attaching an object to a static
    /// body should not affect spring/damper application.
    bool parentStatic =
      this->GetParent() ? this->GetParent()->IsStatic() : false;
    bool childStatic =
      this->GetChild() ? this->GetChild()->IsStatic() : false;

    if (!this->stiffnessDampingInitialized)
    {
      if (!parentStatic && !childStatic)
      {
        this->applyDamping = physics::Joint::ConnectJointUpdate(
          boost::bind(&ODEJoint::ApplyStiffnessDamping, this));
        this->stiffnessDampingInitialized = true;
      }
      else
      {
        gzwarn << "Spring Damper for Joint[" << this->GetName()
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

  if (this->provideFeedback)
  {
    if (this->feedback == nullptr)
    {
      this->feedback = new dJointFeedback;
      this->feedback->f1[0] = 0;
      this->feedback->f1[1] = 0;
      this->feedback->f1[2] = 0;
      this->feedback->t1[0] = 0;
      this->feedback->t1[1] = 0;
      this->feedback->t1[2] = 0;
      this->feedback->f2[0] = 0;
      this->feedback->f2[1] = 0;
      this->feedback->f2[2] = 0;
      this->feedback->t2[0] = 0;
      this->feedback->t2[1] = 0;
      this->feedback->t2[2] = 0;
    }

    if (this->jointId)
      dJointSetFeedback(this->jointId, this->feedback);
    else
      gzerr << "ODE Joint ID is invalid\n";
  }
}

//////////////////////////////////////////////////
void ODEJoint::SetForce(unsigned int _index, double _force)
{
  double force = Joint::CheckAndTruncateForce(_index, _force);
  this->SaveForce(_index, force);
  this->SetForceImpl(_index, force);

  // for engines that supports auto-disable of links
  if (this->childLink)
    this->childLink->SetEnabled(true);
  if (this->parentLink)
    this->parentLink->SetEnabled(true);
}

//////////////////////////////////////////////////
void ODEJoint::SaveForce(unsigned int _index, double _force)
{
  // this bit of code actually doesn't do anything physical,
  // it simply records the forces commanded inside forceApplied.
  if (_index < this->DOF())
  {
    if (this->forceAppliedTime < this->GetWorld()->SimTime())
    {
      // reset forces if time step is new
      this->forceAppliedTime = this->GetWorld()->SimTime();
      this->forceApplied[0] = this->forceApplied[1] = 0;
    }

    this->forceApplied[_index] += _force;
  }
  else
    gzerr << "Something's wrong, joint [" << this->GetScopedName()
          << "] index [" << _index
          << "] out of range.\n";
}

//////////////////////////////////////////////////
double ODEJoint::GetForce(unsigned int _index)
{
  if (_index < this->DOF())
  {
    return this->forceApplied[_index];
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
  IGN_PROFILE("ODEJoint::ApplyStiffnessDamping");
  if (this->useImplicitSpringDamper)
  {
    IGN_PROFILE_BEGIN("implicit");
    this->ApplyImplicitStiffnessDamping();
    IGN_PROFILE_END();
  }
  else
  {
    IGN_PROFILE_BEGIN("explicit");
    this->ApplyExplicitStiffnessDamping();
    IGN_PROFILE_END();
  }
}

//////////////////////////////////////////////////
void ODEJoint::ApplyExplicitStiffnessDamping()
{
  for (unsigned int i = 0; i < this->DOF(); ++i)
  {
    // Take absolute value of dissipationCoefficient, since negative values of
    // dissipationCoefficient are used for adaptive damping to
    // enforce stability.
    double dampingForce = -fabs(this->dissipationCoefficient[i])
      * this->GetVelocity(i);

    double springForce = this->stiffnessCoefficient[i]
      * (this->springReferencePosition[i] - this->Position(i));

    // do not change forceApplied if setting internal damping forces
    this->SetForceImpl(i, dampingForce + springForce);

    // gzerr << this->GetVelocity(0) << " : " << dampingForce << "\n";
  }
}

//////////////////////////////////////////////////
bool ODEJoint::SetPosition(const unsigned int _index, const double _position,
                           const bool _preserveWorldVelocity)
{
  const bool result = Joint::SetPositionMaximal(_index, _position,
                                                _preserveWorldVelocity);

  // The following code fixes issue 2430 without breaking ABI
  // We only need to worry about this for angles outside the range [-pi, pi]
  if (std::abs(_position) >= M_PI)
  {
    // It's only relevant for hinge joints
    if (ODEHingeJoint *hinge = dynamic_cast<ODEHingeJoint*>(this))
    {
      hinge->SetCumulativeAngle(_position);
    }
  }

  return result;
}
