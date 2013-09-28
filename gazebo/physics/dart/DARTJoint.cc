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
#include "gazebo/physics/dart/DARTLink.hh"
#include "gazebo/physics/dart/DARTModel.hh"
#include "gazebo/physics/dart/DARTJoint.hh"
#include "gazebo/physics/dart/DARTUtils.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
DARTJoint::DARTJoint(BasePtr _parent)
  : Joint(_parent),
    dartJoint(NULL),
    dartChildBodyNode(NULL)
{
  this->forceApplied[0] = 0.0;
  this->forceApplied[1] = 0.0;
}

//////////////////////////////////////////////////
DARTJoint::~DARTJoint()
{
  this->Detach();

  if (dartJoint)
    delete dartJoint;
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

  //----------------------------------------------------------------------------
  // Name
  //----------------------------------------------------------------------------
  std::string jointName = this->GetName();
  this->dartJoint->setName(jointName.c_str());

  //----------------------------------------------------------------------------
  // Parent and child link information
  //----------------------------------------------------------------------------
  DARTLinkPtr theParentLink =
    boost::static_pointer_cast<DARTLink>(this->parentLink);
  DARTLinkPtr theChildLink =
    boost::static_pointer_cast<DARTLink>(this->childLink);

  dart::dynamics::BodyNode* dartParentBody = NULL;
  dart::dynamics::BodyNode* dartChildBody = NULL;
  Eigen::Isometry3d dartTransfParentLinkToJointLeft = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d dartTransfChildLinkToJointRight = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d dartTransfParentLink = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d dartTransfChildLink = Eigen::Isometry3d::Identity();

  //if (theChildLink != NULL)
  assert(theChildLink);
  {
    dartChildBody = theChildLink->getDARTBodyNode();
    assert(dartChildBody);

    math::Pose poseChildLink = theChildLink->GetWorldPose();
    dartTransfChildLink = DARTTypes::ConvPose(poseChildLink);

    dartChildBody->setParentJoint(this->dartJoint);
  }
  dartTransfChildLinkToJointRight = DARTTypes::ConvPose(this->anchorPose);

  if (theParentLink != NULL)
  {
    dartParentBody = theParentLink->getDARTBodyNode();
    assert(dartParentBody);

    math::Pose poseParentLink = theParentLink->GetWorldPose();
    dartTransfParentLink = DARTTypes::ConvPose(poseParentLink);

    dartParentBody->addChildBodyNode(dartChildBody);
  }
  // TODO: If the joint is not home position, then we need to care about it
  //       by multiply jointLocalTransformation.inverse() to the end of below
  //       line.
  dartTransfParentLinkToJointLeft = dartTransfParentLink.inverse()
                                    * dartTransfChildLink
                                    * dartTransfChildLinkToJointRight;

  this->dartJoint->setTransformFromParentBodyNode(dartTransfParentLinkToJointLeft);
  this->dartJoint->setTransformFromChildBodyNode(dartTransfChildLinkToJointRight);
  this->dartChildBodyNode = dartChildBody;

  //----------------------------------------------------------------------------
  // TODO: Currently, dampingCoefficient seems not to be initialized when
  //       this joint is loaded. Therefore, we need below code...
  //----------------------------------------------------------------------------
  if (this->sdf->HasElement("axis"))
  {
    sdf::ElementPtr axisElem = this->sdf->GetElement("axis");
    if (axisElem->HasElement("dynamics"))
    {
      sdf::ElementPtr dynamicsElem = axisElem->GetElement("dynamics");

      if (dynamicsElem->HasElement("damping"))
      {
        this->SetDamping(0, dynamicsElem->Get<double>("damping"));
      }
      if (dynamicsElem->HasElement("friction"))
      {
        sdf::ElementPtr frictionElem = dynamicsElem->GetElement("friction");
        gzlog << "joint friction not implemented\n";
      }
    }
  }

  if (this->sdf->HasElement("axis2"))
  {
    sdf::ElementPtr axisElem = this->sdf->GetElement("axis");
    if (axisElem->HasElement("dynamics"))
    {
      sdf::ElementPtr dynamicsElem = axisElem->GetElement("dynamics");

      if (dynamicsElem->HasElement("damping"))
      {
        this->SetDamping(1, dynamicsElem->Get<double>("damping"));
      }
      if (dynamicsElem->HasElement("friction"))
      {
        sdf::ElementPtr frictionElem = dynamicsElem->GetElement("friction");
        gzlog << "joint friction not implemented\n";
      }
    }
  }
}

//////////////////////////////////////////////////
void DARTJoint::Reset()
{
  Joint::Reset();
}

//////////////////////////////////////////////////
LinkPtr DARTJoint::GetJointLink(int _index) const
{
  LinkPtr result;

  if (_index == 0)
  {
    DARTLinkPtr dartLink1
        = boost::shared_static_cast<DARTLink>(this->parentLink);

    if (dartLink1 != NULL)
      return this->parentLink;
  }

  if (_index == 1)
  {
    DARTLinkPtr dartLink2
        = boost::shared_static_cast<DARTLink>(this->childLink);

    if (dartLink2 != NULL)
      return this->childLink;
  }

  return result;
}

//////////////////////////////////////////////////
bool DARTJoint::AreConnected(LinkPtr _one, LinkPtr _two) const
{
  bool res = false;

  DARTLinkPtr dartLink1 = boost::shared_dynamic_cast<DARTLink>(_one);
  DARTLinkPtr dartLink2 = boost::shared_dynamic_cast<DARTLink>(_two);

  dart::dynamics::BodyNode* dartBodyNode1 = dartLink1->getDARTBodyNode();
  dart::dynamics::BodyNode* dartBodyNode2 = dartLink2->getDARTBodyNode();

  if (dartLink1 == NULL || dartLink2 == NULL)
    gzthrow("DARTJoint requires DART bodies\n");

  if (dartBodyNode1->getParentBodyNode() == dartBodyNode2 ||
      dartBodyNode2->getParentBodyNode() == dartBodyNode1)
    res = true;

  return res;
}

//////////////////////////////////////////////////
void DARTJoint::SetDamping(int _index, double _damping)
{
  this->dampingCoefficient = _damping;

  if (this->GetAngleCount() > 2)
  {
     gzerr << "Incompatible joint type, GetAngleCount() = "
           << this->GetAngleCount() << " > 2\n";
     return;
  }

  // \TODO: implement on a per axis basis (requires additional sdf parameters)

  /// \TODO:  this check might not be needed?  attaching an object to a static
  /// body should not affect damping application.
  bool parentStatic = this->GetParent() ? this->GetParent()->IsStatic() : false;
  bool childStatic = this->GetChild() ? this->GetChild()->IsStatic() : false;

  if (!parentStatic && !childStatic)
  {
    this->dartJoint->setDampingCoefficient(_index, _damping);
  }
}

//////////////////////////////////////////////////
void DARTJoint::SetHighStop(int _index, const math::Angle& _angle)
{
  switch (_index)
  {
    case 0:
    case 1:
    case 2:
      this->dartJoint->getGenCoord(_index)->set_qMax(_angle.Radian());
      break;
    default:
      gzerr << "Invalid index[" << _index << "]\n";
      break;
  };
}

//////////////////////////////////////////////////
void DARTJoint::SetLowStop(int _index, const math::Angle& _angle)
{
  switch (_index)
  {
  case 0:
  case 1:
  case 2:
    this->dartJoint->getGenCoord(_index)->set_qMin(_angle.Radian());
    break;
  default:
    gzerr << "Invalid index[" << _index << "]\n";
  };
}

//////////////////////////////////////////////////
math::Angle DARTJoint::GetHighStop(int _index)
{
  switch (_index)
  {
  case 0:
  case 1:
  case 2:
    return this->dartJoint->getGenCoord(_index)->get_qMax();
  default:
    gzerr << "Invalid index[" << _index << "]\n";
  };

  return 0;
}

//////////////////////////////////////////////////
math::Angle DARTJoint::GetLowStop(int _index)
{
  switch (_index)
  {
  case 0:
  case 1:
  case 2:
    return this->dartJoint->getGenCoord(_index)->get_qMin();
  default:
    gzerr << "Invalid index[" << _index << "]\n";
  };

  return 0;
}

//////////////////////////////////////////////////
math::Vector3 DARTJoint::GetLinkForce(unsigned int _index) const
{
  math::Vector3 result;

  if (!this->dartJoint)
  {
    gzerr << "DART joint is invalid\n";
    return result;
  }

  //----------------------------------------------------------------------------
  // Parent and child link information
  //----------------------------------------------------------------------------
  DARTLinkPtr theChildLink =
    boost::static_pointer_cast<DARTLink>(this->childLink);

  Eigen::Vector6d F1 = Eigen::Vector6d::Zero();
  Eigen::Vector6d F2 = Eigen::Vector6d::Zero();
  Eigen::Isometry3d T12 = dartJoint->getLocalTransform();

  // JointWrench.body1Force contains the
  // force applied by the parent Link on the Joint specified in
  // the parent Link frame.
  if (theChildLink != NULL)
  {
    dart::dynamics::BodyNode* dartChildBody = theChildLink->getDARTBodyNode();
    assert(dartChildBody);
    F2 = -dart::math::dAdT(dartJoint->getTransformFromChildBodyNode(),dartChildBody->getBodyForce());
  }

  // JointWrench.body2Force contains
  // the force applied by the child Link on the Joint specified
  // in the child Link frame.
  F1 = -dart::math::dAdInvR(T12 ,F2);

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

  if (!this->dartJoint)
  {
    gzerr << "DART joint is invalid\n";
    return result;
  }

  //----------------------------------------------------------------------------
  // Parent and child link information
  //----------------------------------------------------------------------------
  DARTLinkPtr theChildLink =
    boost::static_pointer_cast<DARTLink>(this->childLink);

  Eigen::Vector6d F1 = Eigen::Vector6d::Zero();
  Eigen::Vector6d F2 = Eigen::Vector6d::Zero();
  Eigen::Isometry3d T12 = dartJoint->getLocalTransform();

  // JointWrench.body1Force contains the
  // force applied by the parent Link on the Joint specified in
  // the parent Link frame.
  if (theChildLink != NULL)
  {
    dart::dynamics::BodyNode* dartChildBody = theChildLink->getDARTBodyNode();
    assert(dartChildBody);
    F2 = -dart::math::dAdT(dartJoint->getTransformFromChildBodyNode(),dartChildBody->getBodyForce());
  }

  // JointWrench.body2Force contains
  // the force applied by the child Link on the Joint specified
  // in the child Link frame.
  F1 = -dart::math::dAdInvR(T12 ,F2);

  if (_index == 0)
    result.Set(F1(0), F1(1), F1(2));
  else
    result.Set(F2(0), F2(1), F2(2));

  return result;
}

//////////////////////////////////////////////////
void DARTJoint::SetAttribute(const std::string &_key, int _index,
                             const boost::any &_value)
{
  if (_key == "hi_stop")
  {
    try
    {
      this->SetHighStop(_index, boost::any_cast<double>(_value));
    }
    catch(boost::bad_any_cast &e)
    {
      gzerr << "boost any_cast error:" << e.what() << "\n";
    }
  }
  else if (_key == "lo_stop")
  {
    try
    {
      this->SetLowStop(_index, boost::any_cast<double>(_value));
    }
    catch(boost::bad_any_cast &e)
    {
      gzerr << "boost any_cast error:" << e.what() << "\n";
    }
  }
  else
  {
    try
    {
      gzerr << "Unable to handle joint attribute[" << _key << "]\n";
    }
    catch(boost::bad_any_cast &e)
    {
      gzerr << "boost any_cast error:" << e.what() << "\n";
    }
  }
}

//////////////////////////////////////////////////
double DARTJoint::GetAttribute(const std::string& _key,
                               unsigned int _index)
{
  if (_key == "hi_stop")
  {
    try
    {
      return this->GetHighStop(_index).Radian();
    }
    catch(common::Exception &e)
    {
      gzerr << "GetParam error:" << e.GetErrorStr() << "\n";
      return 0;
    }
  }
  else if (_key == "lo_stop")
  {
    try
    {
      return this->GetLowStop(_index).Radian();
    }
    catch(common::Exception &e)
    {
      gzerr << "GetParam error:" << e.GetErrorStr() << "\n";
      return 0;
    }
  }
  else
  {
    gzerr << "Unable to get joint attribute[" << _key << "]\n";
    return 0;
  }
}

//////////////////////////////////////////////////
JointWrench DARTJoint::GetForceTorque(int _index)
{
  return this->GetForceTorque(static_cast<unsigned int>(_index));
}

//////////////////////////////////////////////////
JointWrench DARTJoint::GetForceTorque(unsigned int /*_index*/)
{
  JointWrench jointWrench;

  //----------------------------------------------------------------------------
  // Parent and child link information
  //----------------------------------------------------------------------------
  DARTLinkPtr theChildLink =
    boost::static_pointer_cast<DARTLink>(this->childLink);

  Eigen::Vector6d F1 = Eigen::Vector6d::Zero();
  Eigen::Vector6d F2 = Eigen::Vector6d::Zero();
  Eigen::Isometry3d T12 = dartJoint->getLocalTransform();

  // JointWrench.body1Force contains the
  // force applied by the parent Link on the Joint specified in
  // the parent Link frame.
  if (theChildLink != NULL)
  {
    dart::dynamics::BodyNode* dartChildBody = theChildLink->getDARTBodyNode();
    assert(dartChildBody);
    F2 = -dart::math::dAdT(dartJoint->getTransformFromChildBodyNode(),dartChildBody->getBodyForce());
  }

  // JointWrench.body2Force contains
  // the force applied by the child Link on the Joint specified
  // in the child Link frame.
  F1 = -dart::math::dAdInvR(T12 ,F2);

  // kind of backwards here, body1 (parent) corresponds go f2, t2
  // and body2 (child) corresponds go f1, t1
  jointWrench.body1Force.Set(F1(3), F1(4), F1(5));
  jointWrench.body1Torque.Set(F1(0), F1(1), F1(2));
  jointWrench.body2Force.Set(F2(3), F2(4), F2(5));
  jointWrench.body2Torque.Set(F2(0), F2(1), F2(2));

  return jointWrench;
}

void DARTJoint::SetForce(int _index, double _force)
{
  double force = Joint::CheckAndTruncateForce(_index, _force);
  this->SaveForce(_index, force);
  this->SetForceImpl(_index, force);

  // for engines that supports auto-disable of links
  if (this->childLink) this->childLink->SetEnabled(true);
  if (this->parentLink) this->parentLink->SetEnabled(true);
}

double DARTJoint::GetForce(unsigned int _index)
{
  if (_index < this->GetAngleCount())
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

unsigned int DARTJoint::GetAngleCount() const
{
  unsigned int angleCount = 0;

  angleCount = this->dartJoint->getNumGenCoords();

  return angleCount;
}

void DARTJoint::ApplyDamping()
{
  // DART applying damping force inside of DART.
}

DARTModelPtr DARTJoint::GetDARTModel() const
{
  return boost::shared_dynamic_cast<DARTModel>(this->model);
}

void DARTJoint::SaveForce(int _index, double _force)
{
  // this bit of code actually doesn't do anything physical,
  // it simply records the forces commanded inside forceApplied.
  if (_index >= 0 && static_cast<unsigned int>(_index) < this->GetAngleCount())
  {
    if (this->forceAppliedTime < this->GetWorld()->GetSimTime())
    {
      // reset forces if time step is new
      this->forceAppliedTime = this->GetWorld()->GetSimTime();
      this->forceApplied[0] = this->forceApplied[1] = 0.0;
    }

    this->forceApplied[_index] += _force;
  }
  else
    gzerr << "Something's wrong, joint [" << this->GetName()
          << "] index [" << _index
          << "] out of range.\n";
}
