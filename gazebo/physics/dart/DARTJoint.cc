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
//#include "physics/ScrewJoint.hh"

#include "dart/kinematics/Dof.h"
#include "dart/kinematics/Joint.h"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
DARTJoint::DARTJoint(BasePtr _parent)
  : Joint(_parent), dartJoint(NULL)
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
  if (dartJoint)
  {
    delete dartJoint;
    dartJoint = NULL;
  }

  // TODO: need test
  BasePtr myBase = shared_from_this();
  if (this->parentLink)
    boost::shared_dynamic_cast<DARTLink>(this->parentLink)->AddDARTChildJoint(boost::shared_static_cast<DARTJoint>(myBase));
  boost::shared_dynamic_cast<DARTLink>(this->childLink)->SetDARTParentJoint(boost::shared_static_cast<DARTJoint>(myBase));

  // In order to create dart joint, we need to know this joint's parent
  // and child link so we create dart joint after the joint is loaded with sdf
  // .
  kinematics::BodyNode* childBodyNode
      = boost::shared_dynamic_cast<DARTLink>(this->childLink)->GetBodyNode();
  DARTModelPtr dartModel
      = boost::shared_dynamic_cast<DARTModel>(this->model);

  if (this->parentLink && this->childLink)
  {
    // a) create a dart joint.
    kinematics::BodyNode* parentBodyNode
        = boost::shared_dynamic_cast<DARTLink>(this->parentLink)->GetBodyNode();
    dartJoint = new kinematics::Joint(parentBodyNode, childBodyNode);
  }
  else if (this->childLink)
  {
    // a) create a dart joint whose parent link is null pointer.
    dartJoint = new kinematics::Joint(NULL, childBodyNode);

    // b) set the canonical joint of the model as this joint
    dartModel->SetCanonicalJoint(dartJoint);
  }
  else
  {
    gzthrow("joint without links\n");
  }

  if (this->sdf->HasElement("axis"))
  {
    sdf::ElementPtr axisElem = this->sdf->GetElement("axis");
    if (axisElem->HasElement("dynamics"))
    {
      sdf::ElementPtr dynamicsElem = axisElem->GetElement("dynamics");

      if (dynamicsElem->HasElement("damping"))
      {
        //this->SetDamping(0, dynamicsElem->GetValueDouble("damping"));
        dampingCoefficient = dynamicsElem->GetValueDouble("damping");
      }
      if (dynamicsElem->HasElement("friction"))
      {
        sdf::ElementPtr frictionElem = dynamicsElem->GetElement("friction");
        gzlog << "joint friction not implemented\n";
      }
    }
  }

  dartModel->GetSkeletonDynamics()->addJoint(dartJoint);
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
      this->dartJoint->getDof(_index)->setMax(_angle.Radian());
      break;
    case 1:
      this->dartJoint->getDof(_index)->setMax(_angle.Radian());
      break;
    case 2:
      this->dartJoint->getDof(_index)->setMax(_angle.Radian());
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
    this->dartJoint->getDof(_index)->setMin(_angle.Radian());
    break;
  case 1:
    this->dartJoint->getDof(_index)->setMin(_angle.Radian());
    break;
  case 2:
    this->dartJoint->getDof(_index)->setMin(_angle.Radian());
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
    return this->dartJoint->getDof(_index)->getMax();
  case 1:
    return this->dartJoint->getDof(_index)->getMax();
  case 2:
    return this->dartJoint->getDof(_index)->getMax();
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
    return this->dartJoint->getDof(_index)->getMin();
  case 1:
    return this->dartJoint->getDof(_index)->getMin();
  case 2:
    return this->dartJoint->getDof(_index)->getMin();
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
JointWrench DARTJoint::GetForceTorque(int /*_index*/)
{
  JointWrench wrench;

  gzwarn << "Not implemented!\n";

  return wrench;
}

JointWrench DARTJoint::GetForceTorque(unsigned int /*_index*/)
{
  JointWrench wrench;

  gzerr << "Not implemented...\n";

  return wrench;
}

unsigned int DARTJoint::GetAngleCount() const
{
  unsigned int angleCount = 0;

  angleCount = this->dartJoint->getNumDofs();

  return angleCount;
}
