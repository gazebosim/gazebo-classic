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

#include "gazebo/common/Console.hh"
#include "gazebo/common/Exception.hh"

#include "gazebo/physics/World.hh"

#include "gazebo/physics/dart/dart_inc.h"
#include "gazebo/physics/dart/DARTPhysics.hh"
#include "gazebo/physics/dart/DARTModel.hh"
#include "gazebo/physics/dart/DARTLink.hh"
#include "gazebo/physics/dart/DARTJoint.hh"
#include "gazebo/physics/dart/DARTUtils.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
DARTLink::DARTLink(EntityPtr _parent)
  : Link(_parent),
    dartBodyNode(new dart::dynamics::BodyNode)
{
}

//////////////////////////////////////////////////
DARTLink::~DARTLink()
{
  // We don't need to delete dartBodyNode because skeletone will delete
  // dartBodyNode if this is registered to the skeletone.
}

//////////////////////////////////////////////////
void DARTLink::Load(sdf::ElementPtr _sdf)
{
  this->dartPhysics = boost::dynamic_pointer_cast<DARTPhysics>(
      this->GetWorld()->GetPhysicsEngine());

  if (this->dartPhysics == NULL)
    gzthrow("Not using the dart physics engine");

  Link::Load(_sdf);
}

//////////////////////////////////////////////////
void DARTLink::Init()
{
  Link::Init();

  // Name
  std::string bodyName = this->GetName();
  this->dartBodyNode->setName(bodyName);

  // Mass
  double mass = this->inertial->GetMass();
  this->dartBodyNode->setMass(mass);

  // Inertia
  double Ixx = this->inertial->GetIXX();
  double Iyy = this->inertial->GetIYY();
  double Izz = this->inertial->GetIZZ();
  double Ixy = this->inertial->GetIXY();
  double Ixz = this->inertial->GetIXZ();
  double Iyz = this->inertial->GetIYZ();
  this->dartBodyNode->setInertia(Ixx, Iyy, Izz, Ixy, Ixz, Iyz);

  // Visual
  this->visuals;

  // COG offset
  math::Vector3 cog = this->inertial->GetCoG();
  this->dartBodyNode->setLocalCOM(DARTTypes::ConvVec3(cog));

  // Gravity mode
  this->SetGravityMode(this->sdf->Get<bool>("gravity"));

  // We don't add dart body node to the skeleton here because dart body node
  // should be set its parent joint before being added. This body node will be
  // added to the skeleton in DARTModel::Init().
}

//////////////////////////////////////////////////
void DARTLink::Fini()
{
  Link::Fini();
}

//////////////////////////////////////////////////
void DARTLink::OnPoseChange()
{
  Link::OnPoseChange();

  // DART body node always have its parent joint.
  dart::dynamics::Joint* joint = this->dartBodyNode->getParentJoint();

  // This is for the case this function called before DARTModel::Init() is
  // called.
  if (joint == NULL)
    return;

  if (joint->getJointType() == dart::dynamics::Joint::FREE)
  {
    dart::dynamics::FreeJoint* freeJoint =
        dynamic_cast<dart::dynamics::FreeJoint*>(joint);

    const Eigen::Isometry3d& W = DARTTypes::ConvPose(this->GetWorldPose());
    const Eigen::Isometry3d& T1 = joint->getTransformFromParentBodyNode();
    const Eigen::Isometry3d& InvT2 = joint->getTransformFromChildBodyNode();
    Eigen::Isometry3d P = Eigen::Isometry3d::Identity();
    if (this->dartBodyNode->getParentBodyNode())
      P = this->dartBodyNode->getParentBodyNode()->getWorldTransform();
    Eigen::Vector6d t =
        dart::math::logMap(T1.inverse() * P.inverse() * W * InvT2);

    freeJoint->set_q(t);
  }
  else
  {
    gzdbg << "DARTLink::OnPoseChange() doesn't make sense unless the link has "
          << "free joint.\n";
  }
}

//////////////////////////////////////////////////
void DARTLink::SetEnabled(bool /*_enable*/) const
{
  // TODO: DART does not support this functionality.
}

//////////////////////////////////////////////////
bool DARTLink::GetEnabled() const
{
  // TODO: DART does not support this functionality.
  return true;
}

//////////////////////////////////////////////////
void DARTLink::SetLinearVel(const math::Vector3& /*_vel*/)
{
  gzdbg << "DARTLink::SetLinearVel() doesn't make sense in dart.\n";
}

//////////////////////////////////////////////////
void DARTLink::SetAngularVel(const math::Vector3& /*_vel*/)
{
  gzdbg << "DARTLink::SetAngularVel() doesn't make sense in dart.\n";
}

//////////////////////////////////////////////////
void DARTLink::SetForce(const math::Vector3& _force)
{
  // DART assume that _force is external force.
  this->dartBodyNode->setExtForce(Eigen::Vector3d::Zero(),
                                  DARTTypes::ConvVec3(_force));
}

//////////////////////////////////////////////////
void DARTLink::SetTorque(const math::Vector3& _torque)
{
  // DART assume that _torque is external torque.
  this->dartBodyNode->setExtTorque(DARTTypes::ConvVec3(_torque));
}

//////////////////////////////////////////////////
void DARTLink::AddForce(const math::Vector3& _force)
{
  this->dartBodyNode->addExtForce(Eigen::Vector3d::Zero(),
                                  DARTTypes::ConvVec3(_force));
}

/////////////////////////////////////////////////
void DARTLink::AddRelativeForce(const math::Vector3& _force)
{
  this->dartBodyNode->addExtForce(Eigen::Vector3d::Zero(),
                                  DARTTypes::ConvVec3(_force),
                                  true, true);
}

/////////////////////////////////////////////////
void DARTLink::AddForceAtWorldPosition(const math::Vector3& _force,
                                        const math::Vector3& _pos)
{
  this->dartBodyNode->addExtForce(DARTTypes::ConvVec3(_pos),
                                  DARTTypes::ConvVec3(_force),
                                  false, false);
}

/////////////////////////////////////////////////
void DARTLink::AddForceAtRelativePosition(const math::Vector3& _force,
                                          const math::Vector3& _relpos)
{
  this->dartBodyNode->addExtForce(DARTTypes::ConvVec3(_relpos),
                                  DARTTypes::ConvVec3(_force),
                                  true, true);
}

/////////////////////////////////////////////////
void DARTLink::AddTorque(const math::Vector3& _torque)
{
  this->dartBodyNode->addExtTorque(DARTTypes::ConvVec3(_torque));
}

/////////////////////////////////////////////////
void DARTLink::AddRelativeTorque(const math::Vector3& _torque)
{
  this->dartBodyNode->addExtTorque(DARTTypes::ConvVec3(_torque), true);
}

//////////////////////////////////////////////////
gazebo::math::Vector3 DARTLink::GetWorldLinearVel(
    const math::Vector3& _offset) const
{
  const Eigen::Vector3d& linVel =
      this->dartBodyNode->getWorldVelocity(
        DARTTypes::ConvVec3(_offset)).tail<3>();

  return DARTTypes::ConvVec3(linVel);
}

//////////////////////////////////////////////////
math::Vector3 DARTLink::GetWorldLinearVel(
    const gazebo::math::Vector3& _offset,
    const gazebo::math::Quaternion& _q) const
{
  Eigen::Matrix3d R1          = Eigen::Matrix3d(DARTTypes::ConvQuat(_q));
  Eigen::Vector3d worldOffset = R1 * DARTTypes::ConvVec3(_offset);
  Eigen::Vector3d linVel =
      this->dartBodyNode->getWorldVelocity(worldOffset).tail<3>();

  //std::cout << R1 << std::endl;

  return DARTTypes::ConvVec3(linVel);
}

//////////////////////////////////////////////////
math::Vector3 DARTLink::GetWorldCoGLinearVel() const
{
  Eigen::Vector3d worldCOM = this->dartBodyNode->getWorldCOM();
  Eigen::Vector3d linVel
      = this->dartBodyNode->getWorldVelocity(worldCOM).tail<3>();

  return DARTTypes::ConvVec3(linVel);
}

//////////////////////////////////////////////////
math::Vector3 DARTLink::GetWorldAngularVel() const
{
  const Eigen::Vector3d& angVel
      = this->dartBodyNode->getWorldVelocity().head<3>();

  return DARTTypes::ConvVec3(angVel);
}

/////////////////////////////////////////////////
math::Vector3 DARTLink::GetWorldForce() const
{
  // TODO: Need verification
  math::Vector3 force;

  Eigen::Isometry3d W = this->dartBodyNode->getWorldTransform();
  Eigen::Matrix6d G   = this->dartBodyNode->getInertia();
  Eigen::VectorXd V   = this->dartBodyNode->getBodyVelocity();
  Eigen::VectorXd dV  = this->dartBodyNode->getBodyAcceleration();
  Eigen::Vector6d F   = G * dV - dart::math::dad(V, G * V);

  force = DARTTypes::ConvVec3(W.linear() * F.tail<3>());

  return force;
}

//////////////////////////////////////////////////
math::Vector3 DARTLink::GetWorldTorque() const
{
  // TODO: Need verification
  math::Vector3 torque;

  Eigen::Isometry3d W = this->dartBodyNode->getWorldTransform();
  Eigen::Matrix6d G   = this->dartBodyNode->getInertia();
  Eigen::VectorXd V   = this->dartBodyNode->getBodyVelocity();
  Eigen::VectorXd dV  = this->dartBodyNode->getBodyAcceleration();
  Eigen::Vector6d F   = G * dV - dart::math::dad(V, G * V);

  torque = DARTTypes::ConvVec3(W.linear() * F.head<3>());

  return torque;
}

//////////////////////////////////////////////////
void DARTLink::SetGravityMode(bool _mode)
{
  this->sdf->GetElement("gravity")->Set(_mode);
  this->dartBodyNode->setGravityMode(_mode);
}

//////////////////////////////////////////////////
bool DARTLink::GetGravityMode() const
{
  return this->dartBodyNode->getGravityMode();
}

//////////////////////////////////////////////////
void DARTLink::SetSelfCollide(bool _collide)
{
  this->sdf->GetElement("self_collide")->Set(_collide);

  // see: https://github.com/dartsim/dart/issues/84
  gzdbg << "DART does not support DARTLink::SetSelfCollide() yet.\n";
}

//////////////////////////////////////////////////
void DARTLink::SetLinearDamping(double /*_damping*/)
{
  // see: https://github.com/dartsim/dart/issues/85
  gzdbg << "DART does not support DARTLink::SetLinearDamping() yet.\n";
}

//////////////////////////////////////////////////
void DARTLink::SetAngularDamping(double /*_damping*/)
{
  // see: https://github.com/dartsim/dart/issues/85
  gzdbg << "DART does not support DARTLink::SetAngularDamping() yet.\n";
}

//////////////////////////////////////////////////
void DARTLink::SetKinematic(const bool& _state)
{
  this->sdf->GetElement("kinematic")->Set(_state);

  gzdbg << "DART does not support DARTLink::SetKinematic() yet.\n";
}

//////////////////////////////////////////////////
bool DARTLink::GetKinematic() const
{
  // DART does not support kinematic mode for link.";
  return false;
}

//////////////////////////////////////////////////
void DARTLink::SetAutoDisable(bool /*_disable*/)
{
  gzdbg << "DART does not support DARTLink::SetAutoDisable() yet.\n";
}

//////////////////////////////////////////////////
void DARTLink::SetLinkStatic(bool /*_static*/)
{
  gzdbg << "DART does not support DARTLink::SetLinkStatic() yet.\n";
}

//////////////////////////////////////////////////
void DARTLink::updateDirtyPoseFromDARTTransformation()
{
  //-- Step 1: get dart body's transformation
  //-- Step 2: set gazebo link's pose using the transformation
  math::Pose newPose = DARTTypes::ConvPose(
                         this->dartBodyNode->getWorldTransform());

  // Set the new pose to this link
  this->dirtyPose = newPose;

  // Set the new pose to the world
  // (Below method can be changed in gazebo code)
  this->world->dirtyPoses.push_back(this);
}

//////////////////////////////////////////////////
dart::dynamics::BodyNode*DARTLink::GetBodyNode() const
{
  return dartBodyNode;
}

//////////////////////////////////////////////////
DARTPhysicsPtr DARTLink::GetDARTPhysics(void) const
{
  return boost::shared_dynamic_cast<DARTPhysics>(
        this->GetWorld()->GetPhysicsEngine());
}

//////////////////////////////////////////////////
dart::simulation::World* DARTLink::GetDARTWorld(void) const
{
  return GetDARTPhysics()->GetDARTWorld();
}

//////////////////////////////////////////////////
DARTModelPtr DARTLink::GetDARTModel() const
{
  return boost::shared_dynamic_cast<DARTModel>(this->GetModel());
}

//////////////////////////////////////////////////
dart::dynamics::BodyNode*DARTLink::getDARTBodyNode() const
{
  return dartBodyNode;
}

//////////////////////////////////////////////////
void DARTLink::SetDARTParentJoint(DARTJointPtr _dartParentJoint)
{
  dartParentJoint = _dartParentJoint;
}

//////////////////////////////////////////////////
void DARTLink::AddDARTChildJoint(DARTJointPtr _dartChildJoint)
{
  dartChildJoints.push_back(_dartChildJoint);
}
