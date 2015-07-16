/*
 * Copyright (C) 2014-2015 Open Source Robotics Foundation
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
      this->GetWorld()->GetPhysicsEngine())))
{
}

//////////////////////////////////////////////////
DARTJoint::~DARTJoint()
{
  this->Detach();

  delete this->dataPtr;
}

//////////////////////////////////////////////////
void DARTJoint::Load(sdf::ElementPtr _sdf)
{
  // In Joint::Load(sdf::ElementPtr), this joint stored the information of the
  // parent link and child link.
  Joint::Load(_sdf);
}

//////////////////////////////////////////////////
void DARTJoint::Init()
{
  Joint::Init();

  // Name
  std::string jointName = this->GetName();
  this->dataPtr->dtJoint->setName(jointName.c_str());

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

  // if (theChildLink != NULL)
  GZ_ASSERT(dartChildLink.get() != NULL, "dartChildLink pointer is NULL");
  {
    dtTransformChildBodyNode =
        DARTTypes::ConvPose(dartChildLink->GetWorldPose());
    this->dataPtr->dtChildBodyNode = dartChildLink->GetDARTBodyNode();
    this->dataPtr->dtChildBodyNode->setParentJoint(this->dataPtr->dtJoint);
  }
  dtTransformChildLinkToJoint = DARTTypes::ConvPose(this->anchorPose);

  if (dartParentLink.get() != NULL)
  {
    dtTransformParentBodyNode =
        DARTTypes::ConvPose(dartParentLink->GetWorldPose());
    dart::dynamics::BodyNode *dtParentBodyNode =
      dartParentLink->GetDARTBodyNode();
    dtParentBodyNode->addChildBodyNode(this->dataPtr->dtChildBodyNode);
  }

  dtTransformParentLinkToJoint = dtTransformParentBodyNode.inverse() *
                                 dtTransformChildBodyNode *
                                 dtTransformChildLinkToJoint;

  // We assume that the joint angles are all zero.
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

    if (dartLink1 != NULL)
      return this->parentLink;
  }

  if (_index == 1)
  {
    DARTLinkPtr dartLink2
        = boost::static_pointer_cast<DARTLink>(this->childLink);

    if (dartLink2 != NULL)
      return this->childLink;
  }

  return result;
}

//////////////////////////////////////////////////
bool DARTJoint::AreConnected(LinkPtr _one, LinkPtr _two) const
{
  if (_one.get() == NULL && _two.get() == NULL)
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
void DARTJoint::SetAnchor(unsigned int /*_index*/,
    const gazebo::math::Vector3 &/*_anchor*/)
{
  // nothing to do here for DART.
}

//////////////////////////////////////////////////
void DARTJoint::SetDamping(unsigned int _index, double _damping)
{
  if (_index < this->GetAngleCount())
  {
    this->SetStiffnessDamping(_index, this->stiffnessCoefficient[_index],
                              _damping);
  }
  else
  {
    gzerr << "Index[" << _index << "] is out of bounds (GetAngleCount() = "
          << this->GetAngleCount() << ").\n";
    return;
  }
}

//////////////////////////////////////////////////
void DARTJoint::SetStiffness(unsigned int _index, const double _stiffness)
{
  if (_index < this->GetAngleCount())
  {
    this->SetStiffnessDamping(_index, _stiffness,
      this->dissipationCoefficient[_index]);
  }
  else
  {
     gzerr << "DARTJoint::SetStiffness: index[" << _index
           << "] is out of bounds (GetAngleCount() = "
           << this->GetAngleCount() << ").\n";
     return;
  }
}

//////////////////////////////////////////////////
void DARTJoint::SetStiffnessDamping(unsigned int _index,
  double _stiffness, double _damping, double _reference)
{
  if (_index < this->GetAngleCount())
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
bool DARTJoint::SetHighStop(unsigned int _index, const math::Angle &_angle)
{
  switch (_index)
  {
    case 0:
    case 1:
    case 2:
      this->dataPtr->dtJoint->setPositionUpperLimit(_index, _angle.Radian());
      return true;
    default:
      gzerr << "Invalid index[" << _index << "]\n";
      return false;
  };
}

//////////////////////////////////////////////////
bool DARTJoint::SetLowStop(unsigned int _index, const math::Angle &_angle)
{
  switch (_index)
  {
  case 0:
  case 1:
  case 2:
    this->dataPtr->dtJoint->setPositionLowerLimit(_index, _angle.Radian());
    return true;
  default:
    gzerr << "Invalid index[" << _index << "]\n";
    return false;
  };
}

//////////////////////////////////////////////////
math::Angle DARTJoint::GetHighStop(unsigned int _index)
{
  switch (_index)
  {
  case 0:
  case 1:
  case 2:
    return this->dataPtr->dtJoint->getPositionUpperLimit(_index);
  default:
    gzerr << "Invalid index[" << _index << "]\n";
  };

  return 0;
}

//////////////////////////////////////////////////
math::Angle DARTJoint::GetLowStop(unsigned int _index)
{
  switch (_index)
  {
  case 0:
  case 1:
  case 2:
    return this->dataPtr->dtJoint->getPositionLowerLimit(_index);
  default:
    gzerr << "Invalid index[" << _index << "]\n";
  };

  return 0;
}

//////////////////////////////////////////////////
math::Vector3 DARTJoint::GetLinkForce(unsigned int _index) const
{
  math::Vector3 result;

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
  Eigen::Isometry3d T12 = this->dataPtr->dtJoint->getLocalTransform();

  // JointWrench.body1Force contains the
  // force applied by the parent Link on the Joint specified in
  // the parent Link frame.
  if (theChildLink != NULL)
  {
    dart::dynamics::BodyNode *dartChildBody = theChildLink->GetDARTBodyNode();
    GZ_ASSERT(dartChildBody, "dartChildBody pointer is NULL");
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
math::Vector3 DARTJoint::GetLinkTorque(unsigned int _index) const
{
  math::Vector3 result;

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
  Eigen::Isometry3d T12 = this->dataPtr->dtJoint->getLocalTransform();

  // JointWrench.body1Force contains the
  // force applied by the parent Link on the Joint specified in
  // the parent Link frame.
  if (theChildLink != NULL)
  {
    dart::dynamics::BodyNode *dartChildBody = theChildLink->GetDARTBodyNode();
    GZ_ASSERT(dartChildBody, "dartChildBody pointer is NULL");
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
  // try because boost::any_cast can throw
  try
  {
    if (_key == "hi_stop")
    {
      this->SetHighStop(_index, boost::any_cast<double>(_value));
    }
    else if (_key == "lo_stop")
    {
      this->SetLowStop(_index, boost::any_cast<double>(_value));
    }
    else if (_key == "friction")
    {
      this->dataPtr->dtJoint->setCoulombFriction(_index,
                                        boost::any_cast<double>(_value));
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
  try
  {
    if (_key == "friction")
    {
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
JointWrench DARTJoint::GetForceTorque(unsigned int /*_index*/)
{
  JointWrench jointWrench;

  //---------------------------------------------
  // Parent and child link information
  //---------------------------------------------
  DARTLinkPtr theChildLink =
    boost::static_pointer_cast<DARTLink>(this->childLink);

  Eigen::Vector6d F1 = Eigen::Vector6d::Zero();
  Eigen::Vector6d F2 = Eigen::Vector6d::Zero();
  Eigen::Isometry3d T12 = this->dataPtr->dtJoint->getLocalTransform();

  // JointWrench.body2Force (F2) contains
  // the force applied by the child Link on the parent link specified
  // in the child Link orientation frame and with respect to the joint origin
  if (theChildLink != NULL)
  {
    dart::dynamics::BodyNode *dartChildBody = theChildLink->GetDARTBodyNode();
    GZ_ASSERT(dartChildBody, "dartChildBody pointer is NULL");
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
  if (_index < this->GetAngleCount())
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
unsigned int DARTJoint::GetAngleCount() const
{
  unsigned int angleCount = 0;

  angleCount = this->dataPtr->dtJoint->getNumDofs();

  return angleCount;
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
dart::dynamics::Joint *DARTJoint::GetDARTJoint()
{
  return this->dataPtr->dtJoint;
}

/////////////////////////////////////////////////
void DARTJoint::SaveForce(unsigned int _index, double _force)
{
  // this bit of code actually doesn't do anything physical,
  // it simply records the forces commanded inside forceApplied.
  if (_index < this->GetAngleCount())
  {
    if (this->dataPtr->forceAppliedTime < this->GetWorld()->GetSimTime())
    {
      // reset forces if time step is new
      this->dataPtr->forceAppliedTime = this->GetWorld()->GetSimTime();
      this->dataPtr->forceApplied[0] = this->dataPtr->forceApplied[1] = 0.0;
    }

    this->dataPtr->forceApplied[_index] += _force;
  }
  else
    gzerr << "Something's wrong, joint [" << this->GetName()
          << "] index [" << _index
          << "] out of range.\n";
}
