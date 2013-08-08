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
    dartBodyNode(NULL)
{

}

//////////////////////////////////////////////////
DARTLink::~DARTLink()
{
  // We don't need to delete dartBodyNode because skeletone will delete
  // dartBodyNode if this is registered to the skeletone.
}

void DARTLink::Load(sdf::ElementPtr _sdf)
{
  this->dartPhysics = boost::dynamic_pointer_cast<DARTPhysics>(
      this->GetWorld()->GetPhysicsEngine());

  if (this->dartPhysics == NULL)
    gzthrow("Not using the dart physics engine");

  if (this->dartBodyNode != NULL)
  {
    delete this->dartBodyNode;
    this->dartBodyNode = NULL;
  }

  // Create dart's body node associated with gazebo's link.
  this->dartBodyNode = new dart::dynamics::BodyNode;

  Link::Load(_sdf);
}

//////////////////////////////////////////////////
void DARTLink::Init()
{
  //----------------------------------------------------------------------------
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
  this->dartBodyNode->setMomentOfInertia(Ixx, Iyy, Izz, Ixy, Ixz, Iyz);

  // Visual
  this->visuals;

  // COG offset
  math::Vector3 cog = this->inertial->GetCoG();
  this->dartBodyNode->setCOM(DARTUtils::ConvertVector3(cog));

  // Transformation
  math::Pose bodyWorldPose = this->GetWorldPose();
  this->dartBodyNode->setWorldTransform(DARTUtils::ConvertPose(bodyWorldPose));

  // Gravity mode
  this->SetGravityMode(this->sdf->Get<bool>("gravity"));

  // Add dart body to dart skeleton.
  this->GetDARTModel()->GetSkeleton()->addBodyNode(this->dartBodyNode, false);
}

//////////////////////////////////////////////////
void DARTLink::Fini()
{
  Link::Fini();
}

//////////////////////////////////////////////////
void DARTLink::Update()
{
  Link::Update();
}

//////////////////////////////////////////////////
void DARTLink::OnPoseChange()
{
  Link::OnPoseChange();

  const math::Pose& currentPose = this->GetWorldPose();
  this->dartBodyNode->setWorldTransform(DARTUtils::ConvertPose(currentPose));
}

//////////////////////////////////////////////////
void DARTLink::SetEnabled(bool /*_enable*/) const
{
  // TODO: DART does not support this function yet.
  //gzwarn << "Not implemented.\n";
}

//////////////////////////////////////////////////
bool DARTLink::GetEnabled() const
{
  // TODO: DART does not support this function yet.
  //gzwarn << "Not implemented.\n";

  return true;
}

//////////////////////////////////////////////////
void DARTLink::SetLinearVel(const math::Vector3& /*_vel*/)
{
  gzwarn << "Not implemented!\n";
}

//////////////////////////////////////////////////
void DARTLink::SetAngularVel(const math::Vector3& /*_vel*/)
{
  gzwarn << "Not implemented!\n";
}

//////////////////////////////////////////////////
void DARTLink::SetForce(const math::Vector3& /*_force*/)
{
  gzwarn << "Not implemented!\n";
}

//////////////////////////////////////////////////
void DARTLink::SetTorque(const math::Vector3& /*_torque*/)
{
  gzwarn << "Not implemented!\n";
}

//////////////////////////////////////////////////
void DARTLink::AddForce(const math::Vector3& /*_force*/)
{
  gzwarn << "Not implemented!\n";
}

/////////////////////////////////////////////////
void DARTLink::AddRelativeForce(const math::Vector3& /*_force*/)
{
  gzwarn << "Not implemented!\n";
}

/////////////////////////////////////////////////
void DARTLink::AddForceAtWorldPosition(const math::Vector3& /*_force*/,
                                        const math::Vector3& /*_pos*/)
{
  gzwarn << "Not implemented!\n";
}

/////////////////////////////////////////////////
void DARTLink::AddForceAtRelativePosition(const math::Vector3& /*_force*/,
                                           const math::Vector3& /*_relpos*/)
{
  gzwarn << "Not implemented!\n";
}

/////////////////////////////////////////////////
void DARTLink::AddTorque(const math::Vector3& /*_torque*/)
{
  gzwarn << "Not implemented!\n";
}

/////////////////////////////////////////////////
void DARTLink::AddRelativeTorque(const math::Vector3& /*_torque*/)
{
  gzwarn << "Not implemented!\n";
}

//////////////////////////////////////////////////
gazebo::math::Vector3 DARTLink::GetWorldLinearVel(
    const math::Vector3& _offset) const
{
  const dart::math::Vec3& linVel
          = this->dartBodyNode->getVelocityWorldAtPoint(
              DARTUtils::ConvertVector3(_offset)).tail<3>();

  return DARTUtils::ConvertVector3(linVel);
}

//////////////////////////////////////////////////
math::Vector3 DARTLink::GetWorldLinearVel(
    const gazebo::math::Vector3& _offset,
    const gazebo::math::Quaternion& _q) const
{
  math::Pose pose(_offset, _q);

  dart::math::Vec3 linVel
    = this->dartBodyNode->getVelocityWorldAtFrame(
      DARTUtils::ConvertPose(pose)).tail<3>();

  return DARTUtils::ConvertVector3(linVel);
}

math::Vector3 DARTLink::GetWorldCoGLinearVel() const
{
  const dart::math::Vec3& linVel
      = this->dartBodyNode->getVelocityWorldAtCOG().tail<3>();

  return DARTUtils::ConvertVector3(linVel);
}

//////////////////////////////////////////////////
math::Vector3 DARTLink::GetWorldAngularVel() const
{
  const dart::math::Vec3& angVel
      = this->dartBodyNode->getVelocityWorld().head<3>();

  return DARTUtils::ConvertVector3(angVel);
}

/////////////////////////////////////////////////
math::Vector3 DARTLink::GetWorldForce() const
{
  math::Vector3 force;

  gzwarn << "Not implemented!\n";

  return force;
}

//////////////////////////////////////////////////
math::Vector3 DARTLink::GetWorldTorque() const
{
  math::Vector3 torque;

  gzwarn << "Not implemented!\n";

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
  int mode = 0;

  mode = this->dartBodyNode->getGravityMode();

  return mode;
}

//////////////////////////////////////////////////
void DARTLink::SetSelfCollide(bool /*_collide*/)
{
  gzwarn << "Not implemented!\n";
}

//////////////////////////////////////////////////
void DARTLink::SetLinearDamping(double /*_damping*/)
{
  gzwarn << "Not implemented!\n";
}

//////////////////////////////////////////////////
void DARTLink::SetAngularDamping(double /*_damping*/)
{
  gzwarn << "Not implemented!\n";
}

//////////////////////////////////////////////////
void DARTLink::SetKinematic(const bool& _state)
{
  this->sdf->GetElement("kinematic")->Set(_state);
}

//////////////////////////////////////////////////
bool DARTLink::GetKinematic() const
{
  return false;
}

//////////////////////////////////////////////////
void DARTLink::SetAutoDisable(bool /*_disable*/)
{
  gzwarn << "Not implemented!\n";
}

void DARTLink::updateDirtyPoseFromDARTTransformation()
{
  //-- Step 1: get dart body's transformation
  //-- Step 2: set gazebo link's pose using the transformation
  math::Pose newPose = DARTUtils::ConvertPose(
                         this->dartBodyNode->getWorldTransform());

  // Set the new pose to this link
  this->dirtyPose = newPose;

  // Set the new pose to the world
  // (Below method can be changed in gazebo code)
  this->world->dirtyPoses.push_back(this);
}


//////////////////////////////////////////////////
DARTPhysicsPtr DARTLink::GetDARTPhysics(void) const {
  return boost::shared_dynamic_cast<DARTPhysics>(this->GetWorld()->GetPhysicsEngine());
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
void DARTLink::SetDARTParentJoint(DARTJointPtr _dartParentJoint)
{
  dartParentJoint = _dartParentJoint;
}

//////////////////////////////////////////////////
void DARTLink::AddDARTChildJoint(DARTJointPtr _dartChildJoint)
{
  dartChildJoints.push_back(_dartChildJoint);
}

