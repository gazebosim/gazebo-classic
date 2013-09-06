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
//#include "physics/ScrewJoint.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
DARTJoint::DARTJoint(BasePtr _parent)
  : Joint(_parent),
    dartJoint(NULL)
{
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

  // In case this joint is already loaded, we delete dart joint if it is
  // created.
//  if (dartJoint != NULL)
//  {
//    delete dartJoint;
//    dartJoint = NULL;
//  }
}

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

  if (theChildLink != NULL)
  {
    dartChildBody = theChildLink->getDARTBodyNode();
    assert(dartChildBody);

    math::Pose poseChildLink = theChildLink->GetWorldPose();
    dartTransfChildLink = DARTUtils::ConvertPose(poseChildLink);
  }
  dartTransfChildLinkToJointRight = DARTUtils::ConvertPose(this->anchorPose);

  if (theParentLink != NULL)
  {
    dartParentBody = theParentLink->getDARTBodyNode();
    assert(dartParentBody);

    math::Pose poseParentLink = theParentLink->GetWorldPose();
    dartTransfParentLink = DARTUtils::ConvertPose(poseParentLink);
  }
  // TODO: If the joint is not home position, then we need to care about it
  //       by multiply jointLocalTransformation.inverse() to the end of below
  //       line.
  dartTransfParentLinkToJointLeft = dart::math::Inv(dartTransfParentLink)
                                    * dartTransfChildLink
                                    * dartTransfChildLinkToJointRight;
  //* Inv(this->dartJoint->getLocalTransformation());

  this->dartJoint->setParentBody(dartParentBody);
  this->dartJoint->setChildBody(dartChildBody);
  this->dartJoint->setTransformFromParentBody(dartTransfParentLinkToJointLeft);
  this->dartJoint->setTransformFromChildBody(dartTransfChildLinkToJointRight);

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
        //this->SetDamping(0, dynamicsElem->GetValueDouble("damping"));
        this->dampingCoefficient = dynamicsElem->Get<double>("damping");
      }
      if (dynamicsElem->HasElement("friction"))
      {
        sdf::ElementPtr frictionElem = dynamicsElem->GetElement("friction");
        gzlog << "joint friction not implemented\n";
      }
    }
  }

  this->GetDARTModel()->GetSkeleton()->addJoint(dartJoint);
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
  //  DARTLinkPtr dartLink1 = boost::shared_dynamic_cast<DARTLink>(_one);
  //  DARTLinkPtr dartLink2 = boost::shared_dynamic_cast<DARTLink>(_two);

  //  if (dartLink1 == NULL || dartLink2 == NULL)
  //    gzthrow("DARTJoint requires DART bodies\n");

  gzerr << "Not implemented...\n";

  //  return dAreConnected(odeLink1->GetODEId(), odeLink2->GetODEId());
  return (this->childLink.get() == _one.get() && this->parentLink.get() == _two.get())
      || (this->childLink.get() == _two.get() && this->parentLink.get() == _one.get());
}

//////////////////////////////////////////////////
void DARTJoint::Attach(LinkPtr _parent, LinkPtr _child)
{
  Joint::Attach(_parent, _child);

  DARTLinkPtr dartchild = boost::shared_dynamic_cast<DARTLink>(this->childLink);
  DARTLinkPtr dartparent = boost::shared_dynamic_cast<DARTLink>(this->parentLink);

  if (dartchild == NULL && dartparent == NULL)
    gzthrow("DARTJoint requires at least one DART link\n");

  // TODO: DART's joint can't change their links connected.
  // For now, recreating the joint is the only way.

  // TODO: We need to add the functionality, attach/detaach, into dart's joint
  // class.
  //   if (this->dartJoint)
  //     delete this->dartJoint;

  //   kinematics::BodyNode* parentBodyNode = boost::shared_dynamic_cast<DARTLink>(
  //         this->parentLink)->GetBodyNode();
  //   kinematics::BodyNode* childBodyNode = boost::shared_dynamic_cast<DARTLink>(
  //         this->childLink)->GetBodyNode();

  //   this->dartJoint = new kinematics::Joint(parentBodyNode, childBodyNode);

  gzerr << "Not implemented...\n";
}

//////////////////////////////////////////////////
void DARTJoint::Detach()
{
//  this->childLink.reset();
//  this->parentLink.reset();

//  // TODO: DART's joint can't change their links connected.
//  // For now, recreating the joint is the only way.
//  if (this->dartJoint)
//    delete this->dartJoint;

//  kinematics::BodyNode* parentBodyNode = boost::shared_dynamic_cast<DARTLink>(
//        this->parentLink)->GetBodyNode();
//  kinematics::BodyNode* childBodyNode = boost::shared_dynamic_cast<DARTLink>(
//        this->childLink)->GetBodyNode();

//  this->dartJoint = new kinematics::Joint(NULL, NULL);
  gzerr << "Not implemented...\n";
}

//////////////////////////////////////////////////
void DARTJoint::SetHighStop(int _index, const math::Angle& _angle)
{
  switch (_index)
  {
    case 0:
      this->dartJoint->getDof(_index)->set_qMax(_angle.Radian());
      break;
    case 1:
      this->dartJoint->getDof(_index)->set_qMax(_angle.Radian());
      break;
    case 2:
      this->dartJoint->getDof(_index)->set_qMax(_angle.Radian());
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
    this->dartJoint->getDof(_index)->set_qMin(_angle.Radian());
    break;
  case 1:
    this->dartJoint->getDof(_index)->set_qMin(_angle.Radian());
    break;
  case 2:
    this->dartJoint->getDof(_index)->set_qMin(_angle.Radian());
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
    return this->dartJoint->getDof(_index)->get_qMax();
  case 1:
    return this->dartJoint->getDof(_index)->get_qMax();
  case 2:
    return this->dartJoint->getDof(_index)->get_qMax();
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
    return this->dartJoint->getDof(_index)->get_qMin();
  case 1:
    return this->dartJoint->getDof(_index)->get_qMin();
  case 2:
    return this->dartJoint->getDof(_index)->get_qMin();
  default:
    gzerr << "Invalid index[" << _index << "]\n";
  };

  return 0;
}

//////////////////////////////////////////////////
math::Vector3 DARTJoint::GetLinkForce(unsigned int /*_index*/) const
{
  math::Vector3 result;

  gzwarn << "Not implemented!\n";

  return result;
}

//////////////////////////////////////////////////
math::Vector3 DARTJoint::GetLinkTorque(unsigned int /*_index*/) const
{
  math::Vector3 result;

  gzwarn << "Not implemented!\n";

  return result;
}

//////////////////////////////////////////////////
void DARTJoint::SetAttribute(const std::string& /*_key*/, int /*_index*/,
                              const boost::any& /*_value*/)
{
  gzwarn << "Not implemented!\n";
}

//////////////////////////////////////////////////
double DARTJoint::GetAttribute(const std::string& /*_key*/,
                              unsigned int /*_index*/)
{
  gzwarn << "Not implemented!\n";

  return 0;
}

//////////////////////////////////////////////////
JointWrench DARTJoint::GetForceTorque(int _index)
{
  return this->GetForceTorque(static_cast<unsigned int>(_index));
}

JointWrench DARTJoint::GetForceTorque(unsigned int /*_index*/)
{
  JointWrench wrench;

  //----------------------------------------------------------------------------
  // Parent and child link information
  //----------------------------------------------------------------------------
  DARTLinkPtr theChildLink =
    boost::static_pointer_cast<DARTLink>(this->childLink);

  Eigen::Vector6d F1 = Eigen::Vector6d::Zero();
  Eigen::Vector6d F2 = Eigen::Vector6d::Zero();
  Eigen::Isometry3d T12 = dartJoint->getLocalTransformation();

  // JointWrench.body1Force contains the
  // force applied by the parent Link on the Joint specified in
  // the parent Link frame.
  if (theChildLink != NULL)
  {
    dart::dynamics::BodyNode* dartChildBody = theChildLink->getDARTBodyNode();
    assert(dartChildBody);
    F2 = -dart::math::dAdT(dartJoint->getLocalTransformationFromChildBody(),dartChildBody->getBodyForce());
  }

  // JointWrench.body2Force contains
  // the force applied by the child Link on the Joint specified
  // in the child Link frame.
  F1 = -dart::math::dAdInvR(T12 ,F2);

  // kind of backwards here, body1 (parent) corresponds go f2, t2
  // and body2 (child) corresponds go f1, t1
  wrench.body1Force.Set(F1(3), F1(4), F1(5));
  wrench.body1Torque.Set(F1(0), F1(1), F1(2));
  wrench.body2Force.Set(F2(3), F2(4), F2(5));
  wrench.body2Torque.Set(F2(0), F2(1), F2(2));

  return wrench;
}

unsigned int DARTJoint::GetAngleCount() const
{
  unsigned int angleCount = 0;

  angleCount = this->dartJoint->getDOF();

  return angleCount;
}

DARTModelPtr DARTJoint::GetDARTModel() const
{
  return boost::shared_dynamic_cast<DARTModel>(this->model);
}
