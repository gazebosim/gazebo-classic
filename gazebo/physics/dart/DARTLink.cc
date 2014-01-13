/*
 * Copyright 2014 Open Source Robotics Foundation
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

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
DARTLink::DARTLink(EntityPtr _parent)
  : Link(_parent),
    dtBodyNode(new dart::dynamics::BodyNode)
{
  staticLink = false;
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
  this->dtBodyNode->setName(bodyName);

  // Mass
  double mass = this->inertial->GetMass();
  this->dtBodyNode->setMass(mass);

  // Inertia
  double Ixx = this->inertial->GetIXX();
  double Iyy = this->inertial->GetIYY();
  double Izz = this->inertial->GetIZZ();
  double Ixy = this->inertial->GetIXY();
  double Ixz = this->inertial->GetIXZ();
  double Iyz = this->inertial->GetIYZ();
  this->dtBodyNode->setInertia(Ixx, Iyy, Izz, Ixy, Ixz, Iyz);

  // Visual
  this->visuals;

  // COG offset
  math::Vector3 cog = this->inertial->GetCoG();
  this->dtBodyNode->setLocalCOM(DARTTypes::ConvVec3(cog));

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
  dart::dynamics::Joint* joint = this->dtBodyNode->getParentJoint();

  // This is for the case this function called before DARTModel::Init() is
  // called.
  if (joint == NULL)
    return;

  if (joint->getJointType() == dart::dynamics::Joint::FREE)
  {
    dart::dynamics::FreeJoint* freeJoint =
        dynamic_cast<dart::dynamics::FreeJoint*>(joint);

    const Eigen::Isometry3d &W = DARTTypes::ConvPose(this->GetWorldPose());
    const Eigen::Isometry3d &T1 = joint->getTransformFromParentBodyNode();
    const Eigen::Isometry3d &InvT2 = joint->getTransformFromChildBodyNode();
    Eigen::Isometry3d P = Eigen::Isometry3d::Identity();

    if (this->dtBodyNode->getParentBodyNode())
      P = this->dtBodyNode->getParentBodyNode()->getWorldTransform();

    Eigen::Isometry3d Q = T1.inverse() * P.inverse() * W * InvT2;
    Eigen::Vector6d t = Eigen::Vector6d::Zero();
    t.tail<3>() = Q.translation();
    t.head<3>() = dart::math::logMap(Q.linear());
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
void DARTLink::SetLinearVel(const math::Vector3 &/*_vel*/)
{
  gzdbg << "DARTLink::SetLinearVel() doesn't make sense in dart.\n";
}

//////////////////////////////////////////////////
void DARTLink::SetAngularVel(const math::Vector3 &/*_vel*/)
{
  gzdbg << "DARTLink::SetAngularVel() doesn't make sense in dart.\n";
}

//////////////////////////////////////////////////
void DARTLink::SetForce(const math::Vector3 &_force)
{
  // DART assume that _force is external force.
  this->dtBodyNode->setExtForce(Eigen::Vector3d::Zero(),
                                DARTTypes::ConvVec3(_force));
}

//////////////////////////////////////////////////
void DARTLink::SetTorque(const math::Vector3 &_torque)
{
  // DART assume that _torque is external torque.
  this->dtBodyNode->setExtTorque(DARTTypes::ConvVec3(_torque));
}

//////////////////////////////////////////////////
void DARTLink::AddForce(const math::Vector3 &_force)
{
  this->dtBodyNode->addExtForce(Eigen::Vector3d::Zero(),
                                DARTTypes::ConvVec3(_force));
}

/////////////////////////////////////////////////
void DARTLink::AddRelativeForce(const math::Vector3 &_force)
{
  this->dtBodyNode->addExtForce(Eigen::Vector3d::Zero(),
                                DARTTypes::ConvVec3(_force),
                                true, true);
}

/////////////////////////////////////////////////
void DARTLink::AddForceAtWorldPosition(const math::Vector3 &_force,
                                        const math::Vector3 &_pos)
{
  this->dtBodyNode->addExtForce(DARTTypes::ConvVec3(_pos),
                                DARTTypes::ConvVec3(_force),
                                false, false);
}

/////////////////////////////////////////////////
void DARTLink::AddForceAtRelativePosition(const math::Vector3 &_force,
                                          const math::Vector3 &_relpos)
{
  this->dtBodyNode->addExtForce(DARTTypes::ConvVec3(_relpos),
                                DARTTypes::ConvVec3(_force),
                                true, true);
}

/////////////////////////////////////////////////
void DARTLink::AddTorque(const math::Vector3 &_torque)
{
  this->dtBodyNode->addExtTorque(DARTTypes::ConvVec3(_torque));
}

/////////////////////////////////////////////////
void DARTLink::AddRelativeTorque(const math::Vector3 &_torque)
{
  this->dtBodyNode->addExtTorque(DARTTypes::ConvVec3(_torque), true);
}

//////////////////////////////////////////////////
gazebo::math::Vector3 DARTLink::GetWorldLinearVel(
    const math::Vector3 &_offset) const
{
  const Eigen::Vector3d &linVel =
      this->dtBodyNode->getWorldVelocity(
        DARTTypes::ConvVec3(_offset)).tail<3>();

  return DARTTypes::ConvVec3(linVel);
}

//////////////////////////////////////////////////
math::Vector3 DARTLink::GetWorldLinearVel(
    const gazebo::math::Vector3 &_offset,
    const gazebo::math::Quaternion &_q) const
{
  Eigen::Matrix3d R1 = Eigen::Matrix3d(DARTTypes::ConvQuat(_q));
  Eigen::Vector3d worldOffset = R1 * DARTTypes::ConvVec3(_offset);
  Eigen::Vector3d linVel =
    this->dtBodyNode->getWorldVelocity(worldOffset).tail<3>();

  return DARTTypes::ConvVec3(linVel);
}

//////////////////////////////////////////////////
math::Vector3 DARTLink::GetWorldCoGLinearVel() const
{
  Eigen::Vector3d worldCOM = this->dtBodyNode->getWorldCOM();
  Eigen::Vector3d linVel
    = this->dtBodyNode->getWorldVelocity(worldCOM).tail<3>();

  return DARTTypes::ConvVec3(linVel);
}

//////////////////////////////////////////////////
math::Vector3 DARTLink::GetWorldAngularVel() const
{
  const Eigen::Vector3d &angVel
    = this->dtBodyNode->getWorldVelocity().head<3>();

  return DARTTypes::ConvVec3(angVel);
}

/////////////////////////////////////////////////
math::Vector3 DARTLink::GetWorldForce() const
{
  Eigen::Vector6d F = this->dtBodyNode->getExternalForceGlobal();
  return DARTTypes::ConvVec3(F.tail<3>());
}

//////////////////////////////////////////////////
math::Vector3 DARTLink::GetWorldTorque() const
{
  // TODO: Need verification
  math::Vector3 torque;

  Eigen::Isometry3d W = this->dtBodyNode->getWorldTransform();
  Eigen::Matrix6d G   = this->dtBodyNode->getInertia();
  Eigen::VectorXd V   = this->dtBodyNode->getBodyVelocity();
  Eigen::VectorXd dV  = this->dtBodyNode->getBodyAcceleration();
  Eigen::Vector6d F   = G * dV - dart::math::dad(V, G * V);

  torque = DARTTypes::ConvVec3(W.linear() * F.head<3>());

  return torque;
}

//////////////////////////////////////////////////
void DARTLink::SetGravityMode(bool _mode)
{
  this->sdf->GetElement("gravity")->Set(_mode);
  this->dtBodyNode->setGravityMode(_mode);
}

//////////////////////////////////////////////////
bool DARTLink::GetGravityMode() const
{
  return this->dtBodyNode->getGravityMode();
}

//////////////////////////////////////////////////
void DARTLink::SetSelfCollide(bool _collide)
{
  this->sdf->GetElement("self_collide")->Set(_collide);

  // If this function is called before the body node is not added to a skeleton,
  // the body node does not have parent skeleton. So we just return here. Self
  // collision setting will be done later in DARTModel::Init().
  if (dtBodyNode->getSkeleton() == NULL)
    return;

  dart::simulation::World *dtWorld = this->dartPhysics->GetDARTWorld();
  dart::dynamics::Skeleton *dtSkeleton = this->dtBodyNode->getSkeleton();
  dart::collision::CollisionDetector *dtCollDet =
      dtWorld->getConstraintHandler()->getCollisionDetector();

  Link_V links = this->GetModel()->GetLinks();

  bool isSkeletonSelfCollidable =
      this->dtBodyNode->getSkeleton()->isSelfCollidable();

  if (_collide)
  {
    // If the skeleton is already self collidable, then we enable self
    // collidable pairs those are associated with this link. The links in the
    // pairs should be all and not itself each other.
    if (isSkeletonSelfCollidable)
    {
      for (size_t i = 0; i < links.size(); ++i)
      {
        if (links[i].get() != this && links[i]->GetSelfCollide())
        {
          dart::dynamics::BodyNode *itdtBodyNode =
            boost::dynamic_pointer_cast<DARTLink>(links[i])->GetDARTBodyNode();

          // If this->dtBodyNode and itdtBodyNode are connected then don't
          // enable the pair.
          // Please see: https://bitbucket.org/osrf/gazebo/issue/899
          if ((this->dtBodyNode->getParentBodyNode() == itdtBodyNode) ||
              itdtBodyNode->getParentBodyNode() == this->dtBodyNode)
            continue;

          dtCollDet->enablePair(this->dtBodyNode, itdtBodyNode);
        }
      }
    }
    // If the skeleton is not self collidable, we first set the skeleton as
    // self collidable. If the skeleton is self collidable, then DART regards
    // that all the links in the skeleton is self collidable. So, we disable all
    // the pairs of which both of the links in the pair is not self collidable.
    else
    {
      dtSkeleton->setSelfCollidable(true);

      for (size_t i = 0; i < links.size() - 1; ++i)
      {
        for (size_t j = i + 1; j < links.size(); ++j)
        {
          dart::dynamics::BodyNode *itdtBodyNode1 =
            boost::dynamic_pointer_cast<DARTLink>(links[i])->GetDARTBodyNode();
          dart::dynamics::BodyNode *itdtBodyNode2 =
            boost::dynamic_pointer_cast<DARTLink>(links[j])->GetDARTBodyNode();

          // If this->dtBodyNode and itdtBodyNode are connected then don't
          // enable the pair.
          // Please see: https://bitbucket.org/osrf/gazebo/issue/899
          if ((itdtBodyNode1->getParentBodyNode() == itdtBodyNode2) ||
              itdtBodyNode2->getParentBodyNode() == itdtBodyNode1)
            dtCollDet->disablePair(itdtBodyNode1, itdtBodyNode2);

          if (!links[i]->GetSelfCollide() || !links[j]->GetSelfCollide())
            dtCollDet->disablePair(itdtBodyNode1, itdtBodyNode2);
        }
      }
    }
  }
  else
  {
    // If the skeleton is self collidable, then we disable all the pairs
    // associated with this link.
    if (isSkeletonSelfCollidable)
    {
      for (size_t i = 0; i < links.size(); ++i)
      {
        if (links[i].get() != this)
        {
          dart::dynamics::BodyNode *itdtBodyNode =
            boost::dynamic_pointer_cast<DARTLink>(links[i])->GetDARTBodyNode();
          dtCollDet->disablePair(this->dtBodyNode, itdtBodyNode);
        }
      }
    }

    // If now all the links are not self collidable, then we set the skeleton
    // as not self collidable.
    bool isAllLinksNotCollidable = true;
    for (size_t i = 0; i < links.size(); ++i)
    {
      if (links[i]->GetSelfCollide())
      {
        isAllLinksNotCollidable = false;
        break;
      }
    }
    if (isAllLinksNotCollidable)
      dtSkeleton->setSelfCollidable(false);
  }
}

//////////////////////////////////////////////////
void DARTLink::SetLinearDamping(double /*_damping*/)
{
  // see: https://github.com/dartsim/dart/issues/85
  gzwarn << "DART does not support DARTLink::SetLinearDamping() yet.\n";
}

//////////////////////////////////////////////////
void DARTLink::SetAngularDamping(double /*_damping*/)
{
  // see: https://github.com/dartsim/dart/issues/85
  gzwarn << "DART does not support DARTLink::SetAngularDamping() yet.\n";
}

//////////////////////////////////////////////////
void DARTLink::SetKinematic(const bool& _state)
{
  this->sdf->GetElement("kinematic")->Set(_state);

  gzwarn << "DART does not support DARTLink::SetKinematic() yet.\n";
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
  gzwarn << "DART does not support DARTLink::SetAutoDisable() yet.\n";
}

//////////////////////////////////////////////////
void DARTLink::SetLinkStatic(bool /*_static*/)
{
//  if (_static == staticLink)
//    return;

//  if (_static)
//  {
//    // Store the original joint
//    this->dtDynamicJoint = this->dtBodyNode->getParentJoint();

//    this->dtBodyNode->setParentJoint(this->dtStaticJoint);
//  }
//  else
//  {

//  }

  gzwarn << "DART does not support DARTLink::SetLinkStatic() yet.\n";
}

//////////////////////////////////////////////////
void DARTLink::updateDirtyPoseFromDARTTransformation()
{
  // Step 1: get dart body's transformation
  // Step 2: set gazebo link's pose using the transformation
  math::Pose newPose = DARTTypes::ConvPose(
                         this->dtBodyNode->getWorldTransform());

  // Set the new pose to this link
  this->dirtyPose = newPose;

  // Set the new pose to the world
  // (Below method can be changed in gazebo code)
  this->world->dirtyPoses.push_back(this);
}

//////////////////////////////////////////////////
DARTPhysicsPtr DARTLink::GetDARTPhysics(void) const
{
  return boost::dynamic_pointer_cast<DARTPhysics>(
        this->GetWorld()->GetPhysicsEngine());
}

//////////////////////////////////////////////////
dart::simulation::World *DARTLink::GetDARTWorld(void) const
{
  return GetDARTPhysics()->GetDARTWorld();
}

//////////////////////////////////////////////////
DARTModelPtr DARTLink::GetDARTModel() const
{
  return boost::dynamic_pointer_cast<DARTModel>(this->GetModel());
}

//////////////////////////////////////////////////
dart::dynamics::BodyNode *DARTLink::GetDARTBodyNode() const
{
  return dtBodyNode;
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
