/*
 * Copyright (C) 2014-2016 Open Source Robotics Foundation
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

#include "gazebo/common/Assert.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Exception.hh"

#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/physics/Link.hh"
#include "gazebo/physics/PhysicsEngine.hh"
#include "gazebo/physics/dart/DARTLink.hh"
#include "gazebo/physics/dart/DARTModel.hh"
#include "gazebo/physics/dart/DARTJoint.hh"

#include "gazebo/physics/dart/DARTJointPrivate.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
DARTJoint::DARTJoint(BasePtr _parent)
  : Joint(_parent),
    dataPtr(new DARTJointPrivate(boost::dynamic_pointer_cast<DARTPhysics>(
      this->GetWorld()->Physics())))
{
}

//////////////////////////////////////////////////
DARTJoint::~DARTJoint()
{
  this->Detach();

  delete this->dataPtr;
  this->dataPtr = nullptr;
}

//////////////////////////////////////////////////
void DARTJoint::Load(sdf::ElementPtr _sdf)
{
  // In Joint::Load(sdf::ElementPtr), this joint stored the information of the
  // parent link and child link.
  Joint::Load(_sdf);

  this->dataPtr->dtProperties.reset(new dart::dynamics::Joint::Properties());
  this->dataPtr->dtProperties->mName = this->GetName();
  this->dataPtr->dtProperties->mIsPositionLimitEnforced = true;
}

//////////////////////////////////////////////////
void DARTJoint::Init()
{
  Joint::Init();

  this->dataPtr->Initialize();

  GZ_ASSERT(this->dataPtr->dtJoint, "DARTJoint is not initialized.");

  // Parent and child link information
  DARTLinkPtr dartParentLink =
    boost::static_pointer_cast<DARTLink>(this->parentLink);
  DARTLinkPtr dartChildLink =
    boost::static_pointer_cast<DARTLink>(this->childLink);

  Eigen::Isometry3d dtTransformParentLinkToJoint =
      Eigen::Isometry3d::Identity();
  Eigen::Isometry3d dtTransformChildLinkToJoint = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d dtTransformParentBodyNode = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d dtTransformChildBodyNode = Eigen::Isometry3d::Identity();

  GZ_ASSERT(dartChildLink, "dartChildLink pointer is null");
  {
    dtTransformChildBodyNode =
        DARTTypes::ConvPose(dartChildLink->WorldPose());
    this->dataPtr->dtChildBodyNode = dartChildLink->DARTBodyNode();
  }
  dtTransformChildLinkToJoint = DARTTypes::ConvPose(this->anchorPose);

  if (dartParentLink)
  {
    dtTransformParentBodyNode =
        DARTTypes::ConvPose(dartParentLink->WorldPose());
  }

  dtTransformParentLinkToJoint = dtTransformParentBodyNode.inverse() *
                                 dtTransformChildBodyNode *
                                 dtTransformChildLinkToJoint;

  // We assume that the joint angles are all zero.
  GZ_ASSERT(this->dataPtr->dtJoint, "dtJoint is null pointer.\n");
  this->dataPtr->dtJoint->setTransformFromParentBodyNode(
        dtTransformParentLinkToJoint);
  this->dataPtr->dtJoint->setTransformFromChildBodyNode(
        dtTransformChildLinkToJoint);
}

//////////////////////////////////////////////////
void DARTJoint::Reset()
{
  Joint::Reset();
}

//////////////////////////////////////////////////
LinkPtr DARTJoint::GetJointLink(unsigned int _index) const
{
  LinkPtr result;

  if (_index == 0)
  {
    DARTLinkPtr dartLink1
        = boost::static_pointer_cast<DARTLink>(this->parentLink);

    if (dartLink1)
      return this->parentLink;
  }

  if (_index == 1)
  {
    DARTLinkPtr dartLink2
        = boost::static_pointer_cast<DARTLink>(this->childLink);

    if (dartLink2)
      return this->childLink;
  }

  return result;
}

//////////////////////////////////////////////////
bool DARTJoint::AreConnected(LinkPtr _one, LinkPtr _two) const
{
  if (_one == nullptr && _two == nullptr)
    return false;

  if ((this->childLink.get() == _one.get() &&
       this->parentLink.get() == _two.get()) ||
      (this->childLink.get() == _two.get() &&
       this->parentLink.get() == _one.get()))
    return true;

  return false;
}

//////////////////////////////////////////////////
void DARTJoint::Attach(LinkPtr _parent, LinkPtr _child)
{
  Joint::Attach(_parent, _child);

  if (this->AreConnected(_parent, _child))
    return;

  gzerr << "DART does not support joint attaching.\n";
}

//////////////////////////////////////////////////
void DARTJoint::Detach()
{
  if (!this->AreConnected(this->parentLink, this->childLink))
    return;

  this->childLink.reset();
  this->parentLink.reset();

  gzerr << "DART does not support joint dettaching.\n";

  Joint::Detach();
}

//////////////////////////////////////////////////
void DARTJoint::SetAnchor(const unsigned int /*_index*/,
    const ignition::math::Vector3d &/*_anchor*/)
{
  gzerr << "DARTJoint: SetAnchor is not implemented.\n";
}

//////////////////////////////////////////////////
ignition::math::Vector3d DARTJoint::Anchor(
    const unsigned int _index) const
{
  if (!this->dataPtr->IsInitialized())
  {
    return this->dataPtr->GetCached<ignition::math::Vector3d>(
          "Anchor" + std::to_string(_index));
  }

  GZ_ASSERT(this->dataPtr->dtJoint, "DART joint is nullptr.");

  Eigen::Isometry3d T = this->dataPtr->dtChildBodyNode->getTransform() *
                        this->dataPtr->dtJoint->getTransformFromChildBodyNode();
  Eigen::Vector3d worldOrigin = T.translation();

  return DARTTypes::ConvVec3Ign(worldOrigin);
}

//////////////////////////////////////////////////
double DARTJoint::GetVelocity(unsigned int _index) const
{
  if (!this->dataPtr->IsInitialized())
  {
    return this->dataPtr->GetCached<double>(
          "Velocity" + std::to_string(_index));
  }

  double result = 0.0;

  GZ_ASSERT(this->dataPtr->dtJoint, "DART joint is nullptr.");

  const unsigned int dofs = static_cast<unsigned int>(
        this->dataPtr->dtJoint->getNumDofs());

  if (_index < dofs)
    result = this->dataPtr->dtJoint->getVelocity(
          static_cast<std::size_t>(_index));
  else
    gzerr << "Invalid index [" << _index << "] (max: " << dofs << ")\n";

  return result;
}

//////////////////////////////////////////////////
void DARTJoint::SetVelocity(unsigned int _index, double _vel)
{
  if (!this->dataPtr->IsInitialized())
  {
    this->dataPtr->Cache(
          "Velocity" + std::to_string(_index),
          boost::bind(&DARTJoint::SetVelocity, this, _index, _vel),
          _vel);
    return;
  }

  GZ_ASSERT(this->dataPtr->dtJoint, "DART joint is nullptr.");

  const unsigned int dofs = static_cast<unsigned int>(
        this->dataPtr->dtJoint->getNumDofs());

  if (_index < dofs)
    this->dataPtr->dtJoint->setVelocity(_index, _vel);
  else
    gzerr << "Invalid index [" << _index << "] (max: " << dofs << ")\n";
}

//////////////////////////////////////////////////
void DARTJoint::SetDamping(unsigned int _index, double _damping)
{
  if (_index < this->DOF())
  {
    this->SetStiffnessDamping(_index, this->stiffnessCoefficient[_index],
                              _damping);
  }
  else
  {
    gzerr << "Index[" << _index << "] is out of bounds (DOF() = "
          << this->DOF() << ").\n";
    return;
  }
}

//////////////////////////////////////////////////
void DARTJoint::SetStiffness(unsigned int _index, const double _stiffness)
{
  if (_index < this->DOF())
  {
    this->SetStiffnessDamping(_index, _stiffness,
      this->dissipationCoefficient[_index]);
  }
  else
  {
     gzerr << "DARTJoint::SetStiffness: index[" << _index
           << "] is out of bounds (DOF() = "
           << this->DOF() << ").\n";
     return;
  }
}

//////////////////////////////////////////////////
void DARTJoint::SetStiffnessDamping(unsigned int _index,
  double _stiffness, double _damping, double _reference)
{
  if (_index < this->DOF())
  {
    this->stiffnessCoefficient[_index] = _stiffness;
    this->dissipationCoefficient[_index] = _damping;
    this->springReferencePosition[_index] = _reference;

    /// \TODO: this check might not be needed?  attaching an object to a static
    /// body should not affect damping application.
    bool parentStatic =
        this->GetParent() ? this->GetParent()->IsStatic() : false;
    bool childStatic =
        this->GetChild() ? this->GetChild()->IsStatic() : false;

    if (!this->applyDamping)
    {
      if (!parentStatic && !childStatic)
      {
        if (!this->dataPtr->IsInitialized())
        {
          this->dataPtr->Cache(
                "StiffnessDamping",
                boost::bind(&DARTJoint::SetStiffnessDamping, this,
                            _index, _stiffness, _damping, _reference));
          return;
        }

        GZ_ASSERT(this->dataPtr->dtJoint, "dtJoint is null pointer.\n");

        this->dataPtr->dtJoint->setSpringStiffness(
              static_cast<int>(_index), _stiffness);
        this->dataPtr->dtJoint->setRestPosition(
              static_cast<int>(_index), _reference);
        this->dataPtr->dtJoint->setDampingCoefficient(
              static_cast<int>(_index), _damping);
        this->applyDamping = physics::Joint::ConnectJointUpdate(
          boost::bind(&DARTJoint::ApplyDamping, this));
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
  {
    gzerr << "SetStiffnessDamping _index " << _index << " is too large.\n";
  }
}

//////////////////////////////////////////////////
void DARTJoint::SetUpperLimit(const unsigned int _index, const double _limit)
{
  if (!this->dataPtr->IsInitialized())
  {
    this->dataPtr->Cache(
          "UpperLimit"+std::to_string(_index),
          boost::bind(&DARTJoint::SetUpperLimit, this, _index, _limit), _limit);
    return;
  }

  const unsigned int dofs = static_cast<unsigned int>(
        this->dataPtr->dtJoint->getNumDofs());

  if (_index >= dofs)
  {
    gzerr << "Invalid index [" << _index << "] (max: " << dofs << ")\n";
    return;
  }

  GZ_ASSERT(this->dataPtr->dtJoint, "dtJoint is null pointer.\n");
  this->dataPtr->dtJoint->setPositionUpperLimit(_index, _limit);
}

//////////////////////////////////////////////////
void DARTJoint::SetLowerLimit(const unsigned int _index, const double _limit)
{
  if (!this->dataPtr->IsInitialized())
  {
    this->dataPtr->Cache(
          "LowerLimit"+std::to_string(_index),
          boost::bind(&DARTJoint::SetLowerLimit, this, _index, _limit), _limit);
    return;
  }

  const unsigned int dofs = static_cast<unsigned int>(
        this->dataPtr->dtJoint->getNumDofs());

  if (_index >= dofs)
  {
    gzerr << "Invalid index [" << _index << "] (max: " << dofs << ")\n";
    return;
  }

  GZ_ASSERT(this->dataPtr->dtJoint, "dtJoint is null pointer.\n");
  this->dataPtr->dtJoint->setPositionLowerLimit(_index, _limit);
}

//////////////////////////////////////////////////
double DARTJoint::UpperLimit(const unsigned int _index) const
{
  if (!this->dataPtr->IsInitialized())
  {
    return this->dataPtr->GetCached<double>(
          "UpperLimit"+std::to_string(_index));
  }

  const unsigned int dofs = static_cast<unsigned int>(
        this->dataPtr->dtJoint->getNumDofs());

  if (_index >= dofs)
  {
    gzerr << "Invalid index [" << _index << "] (max: " << dofs << ")\n";
    return ignition::math::NAN_D;
  }

  GZ_ASSERT(this->dataPtr->dtJoint, "dtJoint is null pointer.\n");
  return this->dataPtr->dtJoint->getPositionUpperLimit(_index);
}

//////////////////////////////////////////////////
double DARTJoint::LowerLimit(const unsigned int _index) const
{
  if (!this->dataPtr->IsInitialized())
  {
    return this->dataPtr->GetCached<double>(
          "LowerLimit"+std::to_string(_index));
  }

  const unsigned int dofs = static_cast<unsigned int>(
        this->dataPtr->dtJoint->getNumDofs());

  if (_index >= dofs)
  {
    gzerr << "Invalid index [" << _index << "] (max: " << dofs << ")\n";
    return ignition::math::NAN_D;
  }

  GZ_ASSERT(this->dataPtr->dtJoint, "dtJoint is null pointer.\n");
  return this->dataPtr->dtJoint->getPositionLowerLimit(_index);
}

//////////////////////////////////////////////////
ignition::math::Vector3d DARTJoint::LinkForce(
          const unsigned int _index) const
{
  if (!this->dataPtr->IsInitialized())
    return ignition::math::Vector3d::Zero;

  ignition::math::Vector3d result;

  if (!this->dataPtr->dtJoint)
  {
    gzerr << "DART joint is invalid\n";
    return result;
  }

  //---------------------------------------------
  // Parent and child link information
  //---------------------------------------------
  DARTLinkPtr theChildLink =
    boost::static_pointer_cast<DARTLink>(this->childLink);

  Eigen::Vector6d F1 = Eigen::Vector6d::Zero();
  Eigen::Vector6d F2 = Eigen::Vector6d::Zero();
  Eigen::Isometry3d T12 = this->dataPtr->dtJoint->getRelativeTransform();

  // JointWrench.body1Force contains the
  // force applied by the parent Link on the Joint specified in
  // the parent Link frame.
  if (theChildLink)
  {
    dart::dynamics::BodyNode *dartChildBody = theChildLink->DARTBodyNode();
    GZ_ASSERT(dartChildBody, "dartChildBody pointer is null");
    GZ_ASSERT(this->dataPtr->dtJoint, "dtJoint is null pointer.\n");
    F2 = -dart::math::dAdT(
          this->dataPtr->dtJoint->getTransformFromChildBodyNode(),
          dartChildBody->getBodyForce());
  }

  // JointWrench.body2Force contains
  // the force applied by the child Link on the Joint specified
  // in the child Link frame.
  F1 = -dart::math::dAdInvR(T12, F2);

  if (_index == 0)
    result.Set(F1(3), F1(4), F1(5));
  else
    result.Set(F2(3), F2(4), F2(5));

  return result;
}

//////////////////////////////////////////////////
ignition::math::Vector3d DARTJoint::LinkTorque(
          const unsigned int _index) const
{
  if (!this->dataPtr->IsInitialized())
    return ignition::math::Vector3d::Zero;

  ignition::math::Vector3d result;

  if (!this->dataPtr->dtJoint)
  {
    gzerr << "DART joint is invalid\n";
    return result;
  }

  // Parent and child link information
  DARTLinkPtr theChildLink =
    boost::static_pointer_cast<DARTLink>(this->childLink);

  Eigen::Vector6d F1 = Eigen::Vector6d::Zero();
  Eigen::Vector6d F2 = Eigen::Vector6d::Zero();
  Eigen::Isometry3d T12 = this->dataPtr->dtJoint->getRelativeTransform();

  // JointWrench.body1Force contains the
  // force applied by the parent Link on the Joint specified in
  // the parent Link frame.
  if (theChildLink)
  {
    dart::dynamics::BodyNode *dartChildBody = theChildLink->DARTBodyNode();
    GZ_ASSERT(dartChildBody, "dartChildBody pointer is null");
    F2 = -dart::math::dAdT(
      this->dataPtr->dtJoint->getTransformFromChildBodyNode(),
      dartChildBody->getBodyForce());
  }

  // JointWrench.body2Force contains
  // the force applied by the child Link on the Joint specified
  // in the child Link frame.
  F1 = -dart::math::dAdInvR(T12, F2);

  if (_index == 0)
    result.Set(F1(0), F1(1), F1(2));
  else
    result.Set(F2(0), F2(1), F2(2));

  return result;
}

//////////////////////////////////////////////////
bool DARTJoint::SetParam(const std::string &_key, unsigned int _index,
                         const boost::any &_value)
{
  if (!this->dataPtr->IsInitialized())
  {
    this->dataPtr->Cache(
          "Param_"+_key+std::to_string(_index),
          boost::bind(&DARTJoint::SetParam, this, _key, _index, _value),
          _value);
    return false;
  }

  const unsigned int dofs = static_cast<unsigned int>(
        this->dataPtr->dtJoint->getNumDofs());

  if (_index >= dofs)
    gzerr << "Invalid index [" << _index << "] (max: " << dofs << ")\n";

  // try because boost::any_cast can throw
  try
  {
    if (_key == "hi_stop")
    {
      this->SetUpperLimit(_index, boost::any_cast<double>(_value));
    }
    else if (_key == "lo_stop")
    {
      this->SetLowerLimit(_index, boost::any_cast<double>(_value));
    }
    else if (_key == "friction")
    {
      GZ_ASSERT(this->dataPtr->dtJoint, "dtJoint is null pointer.\n");
      this->dataPtr->dtJoint->setCoulombFriction(
            _index, boost::any_cast<double>(_value));
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
double DARTJoint::GetParam(const std::string &_key, unsigned int _index)
{
  if (!this->dataPtr->IsInitialized())
  {
    return this->dataPtr->GetCached<double>(
          "Param_"+_key+std::to_string(_index));
  }

  const unsigned int dofs = static_cast<unsigned int>(
        this->dataPtr->dtJoint->getNumDofs());

  if (_index >= dofs)
  {
    gzerr << "Invalid index [" << _index << "] (max: " << dofs << ")\n";
    return false;
  }

  try
  {
    if (_key == "friction")
    {
      GZ_ASSERT(this->dataPtr->dtJoint, "dtJoint is null pointer.\n");
      return this->dataPtr->dtJoint->getCoulombFriction(_index);
    }
  }
  catch(const common::Exception &e)
  {
    gzerr << "GetParam(" << _key << ") error:"
          << e.GetErrorStr()
          << std::endl;
    return 0;
  }
  return Joint::GetParam(_key, _index);
}

//////////////////////////////////////////////////
void DARTJoint::CacheForceTorque()
{
  // Does nothing for now, will add when recovering pull request #1721
}

//////////////////////////////////////////////////
JointWrench DARTJoint::GetForceTorque(unsigned int /*_index*/)
{
  if (!this->dataPtr->IsInitialized())
    return JointWrench();

  JointWrench jointWrench;

  GZ_ASSERT(this->dataPtr->dtJoint, "dtJoint is null pointer.\n");

  //---------------------------------------------
  // Parent and child link information
  //---------------------------------------------
  DARTLinkPtr theChildLink =
    boost::static_pointer_cast<DARTLink>(this->childLink);

  Eigen::Vector6d F1 = Eigen::Vector6d::Zero();
  Eigen::Vector6d F2 = Eigen::Vector6d::Zero();
  Eigen::Isometry3d T12 = this->dataPtr->dtJoint->getRelativeTransform();

  // JointWrench.body2Force (F2) contains
  // the force applied by the child Link on the parent link specified
  // in the child Link orientation frame and with respect to the joint origin
  if (theChildLink)
  {
    dart::dynamics::BodyNode *dartChildBody = theChildLink->DARTBodyNode();
    GZ_ASSERT(dartChildBody, "dartChildBody pointer is null");
    Eigen::Isometry3d TJ2 = Eigen::Isometry3d::Identity();
    TJ2.translation() =
        this->dataPtr->dtJoint->getTransformFromChildBodyNode().translation();
    F2 = -dart::math::dAdT(TJ2,
                           dartChildBody->getBodyForce());
  }

  // JointWrench.body1Force (F1) contains the
  // force applied by the parent Link on the child Link specified in
  // the parent Link orientation frame and with respect to the joint origin
  F1 = -dart::math::dAdInvR(T12, F2);

  // kind of backwards here, body1 (parent) corresponds go f2, t2
  // and body2 (child) corresponds go f1, t1
  jointWrench.body1Force.Set(F1(3), F1(4), F1(5));
  jointWrench.body1Torque.Set(F1(0), F1(1), F1(2));
  jointWrench.body2Force.Set(F2(3), F2(4), F2(5));
  jointWrench.body2Torque.Set(F2(0), F2(1), F2(2));

  return jointWrench;
}

/////////////////////////////////////////////////
bool DARTJoint::SetPosition(const unsigned int _index, const double _position,
                            const bool _preserveWorldVelocity)
{
  if (!this->dataPtr->IsInitialized())
  {
    this->dataPtr->Cache(
          "Position" + std::to_string(_index),
          boost::bind(&DARTJoint::SetPosition, this, _index, _position,
                      _preserveWorldVelocity),
          _position);
    return false;
  }

  GZ_ASSERT(this->dataPtr->dtJoint, "DART joint is a nullptr.");

  const unsigned int dofs = static_cast<unsigned int>(
        this->dataPtr->dtJoint->getNumDofs());

  if (_preserveWorldVelocity)
  {
    gzwarn << "[SetPosition] You requested _preserveWorldVelocity "
           << "to be true, but this is not supported in DART. The world "
           << "velocity of the child link will not be preserved\n";
  }

  if (_index < dofs)
  {
    this->dataPtr->dtJoint->setPosition(_index, _position);
    return true;
  }

  gzerr << "Invalid index [" << _index << "] (max: " << dofs << ")\n";
  return false;
}

/////////////////////////////////////////////////
void DARTJoint::SetForce(unsigned int _index, double _force)
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

/////////////////////////////////////////////////
double DARTJoint::GetForce(unsigned int _index)
{
  if (_index < this->DOF())
  {
    return this->dataPtr->forceApplied[_index];
  }
  else
  {
    gzerr << "Invalid joint index [" << _index
          << "] when trying to get force\n";
    return 0;
  }
}

/////////////////////////////////////////////////
void DARTJoint::ApplyDamping()
{
  // rename ApplyDamping to ApplyStiffnessDamping (below) in gazebo 4.0.
  // public: virtual void ApplyStiffnessDamping();

  // DART applies stiffness and damping force implicitly itself by setting
  // the stiffness coefficient and the damping coefficient using
  // dart::dynamics::Joint::setSpringStiffness(index, stiffnessCoeff) and
  // dart::dynamics::Joint::setDampingCoefficient(index, dampingCoeff).
  // Therefore, we do nothing here.
}

/////////////////////////////////////////////////
DARTModelPtr DARTJoint::GetDARTModel() const
{
  return boost::dynamic_pointer_cast<DARTModel>(this->model);
}

/////////////////////////////////////////////////
DARTJointPropPtr DARTJoint::DARTProperties() const
{
  return this->dataPtr->dtProperties;
}

/////////////////////////////////////////////////
void DARTJoint::SetDARTJoint(dart::dynamics::Joint *_dtJoint)
{
  GZ_ASSERT(_dtJoint, "Can only set joints which are not nullptr");
  this->dataPtr->dtJoint = _dtJoint;
}

/////////////////////////////////////////////////
dart::dynamics::Joint *DARTJoint::GetDARTJoint()
{
  return this->dataPtr->dtJoint;
}

//////////////////////////////////////////////////
double DARTJoint::PositionImpl(const unsigned int _index) const
{
  if (!this->dataPtr->IsInitialized())
  {
    return this->dataPtr->GetCached<double>(
          "Position" + std::to_string(_index));
  }

  double result = ignition::math::NAN_D;

  GZ_ASSERT(this->dataPtr->dtJoint, "DART joint is nullptr.");

  const unsigned int dofs = static_cast<unsigned int>(
        this->dataPtr->dtJoint->getNumDofs());

  if (_index < dofs)
  {
    result = this->dataPtr->dtJoint->getPosition(
          static_cast<std::size_t>(_index));
  }
  else
  {
    gzerr << "Invalid index [" << _index << "] (max: " << dofs << ")\n";
  }

  return result;
}

//////////////////////////////////////////////////
void DARTJoint::SetForceImpl(unsigned int _index, double _force)
{
  if (!this->dataPtr->IsInitialized())
  {
    this->dataPtr->Cache(
        "Force" + std::to_string(_index),
        boost::bind(&DARTJoint::SetForceImpl, this, _index, _force));
    return;
  }

  GZ_ASSERT(this->dataPtr->dtJoint, "DART joint is nullptr.");

  const unsigned int dofs = static_cast<unsigned int>(
        this->dataPtr->dtJoint->getNumDofs());

  if (_index < dofs)
    this->dataPtr->dtJoint->setForce(static_cast<std::size_t>(_index), _force);
  else
    gzerr << "Invalid index [" << _index << "] (max: " << dofs << ")\n";
}

/////////////////////////////////////////////////
void DARTJoint::SaveForce(unsigned int _index, double _force)
{
  if (!this->dataPtr->IsInitialized())
  {
    this->dataPtr->Cache(
          "DARTJoint::SaveForce",
          boost::bind(&DARTJoint::SaveForce, this, _index, _force));
    return;
  }

  // this bit of code actually doesn't do anything physical,
  // it simply records the forces commanded inside forceApplied.
  if (_index < this->DOF())
  {
    if (this->dataPtr->forceAppliedTime < this->GetWorld()->SimTime())
    {
      // reset forces if time step is new
      this->dataPtr->forceAppliedTime = this->GetWorld()->SimTime();
      this->dataPtr->forceApplied[0] = this->dataPtr->forceApplied[1] = 0.0;
    }

    this->dataPtr->forceApplied[_index] += _force;
  }
  else
    gzerr << "Something's wrong, joint [" << this->GetName()
          << "] index [" << _index
          << "] out of range.\n";
}
