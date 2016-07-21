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
#include "gazebo/common/Assert.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Exception.hh"

#include "gazebo/physics/bullet/BulletTypes.hh"
#include "gazebo/physics/bullet/BulletLink.hh"
#include "gazebo/physics/bullet/BulletJointPrivate.hh"
#include "gazebo/physics/bullet/BulletBallJoint.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
BulletBallJoint::BulletBallJoint(btDynamicsWorld *_world, BasePtr _parent)
    : BallJoint<BulletJoint>(_parent)
{
  GZ_ASSERT(_world, "bullet world pointer is null");
  this->bulletJointDPtr->bulletWorld = _world;
  this->bulletBall = nullptr;
}

//////////////////////////////////////////////////
BulletBallJoint::~BulletBallJoint()
{
}

//////////////////////////////////////////////////
void BulletBallJoint::Load(sdf::ElementPtr _sdf)
{
  BallJoint<BulletJoint>::Load(_sdf);
}

//////////////////////////////////////////////////
void BulletBallJoint::Init()
{
  BallJoint<BulletJoint>::Init();

  // Cast to BulletLink
  BulletLinkPtr bulletChildLink =
    std::static_pointer_cast<BulletLink>(this->bulletJointDPtr->childLink);
  BulletLinkPtr bulletParentLink =
    std::static_pointer_cast<BulletLink>(this->bulletJointDPtr->parentLink);

  // Local variables used to compute pivots and axes in body-fixed frames
  // for the parent and child links.
  math::Vector3 pivotParent, pivotChild;
  math::Pose pose;

  // Initialize pivots to anchorPos, which is expressed in the
  // world coordinate frame.
  pivotParent = this->bulletJointDPtr->anchorPos;
  pivotChild = this->bulletJointDPtr->anchorPos;

  // Check if parentLink exists. If not, the parent will be the world.
  if (this->bulletJointDPtr->parentLink)
  {
    // Compute relative pose between joint anchor and CoG of parent link.
    pose = this->bulletJointDPtr->parentLink->WorldCoGPose();
    // Subtract CoG position from anchor position, both in world frame.
    pivotParent -= pose.pos;
    // Rotate pivot offset and axis into body-fixed frame of parent.
    pivotParent = pose.rot.RotateVectorReverse(pivotParent);
  }
  // Check if childLink exists. If not, the child will be the world.
  if (this->bulletJointDPtr->childLink)
  {
    // Compute relative pose between joint anchor and CoG of child link.
    pose = this->bulletJointDPtr->childLink->WorldCoGPose();

    // Subtract CoG position from anchor position, both in world frame.
    pivotChild -= pose.pos;

    // Rotate pivot offset and axis into body-fixed frame of child.
    pivotChild = pose.rot.RotateVectorReverse(pivotChild);
  }

  // If both links exist, then create a joint between the two links.
  if (bulletChildLink && bulletParentLink)
  {
    this->bulletBall = new btPoint2PointConstraint(
      *bulletChildLink->BtLink(),
      *bulletParentLink->BtLink(),
      BulletTypes::ConvertVector3(pivotChild),
      BulletTypes::ConvertVector3(pivotParent));
  }
  // If only the child exists, then create a joint between the child
  // and the world.
  else if (bulletChildLink)
  {
    this->bulletBall = new btPoint2PointConstraint(
      *bulletChildLink->BtLink(),
      BulletTypes::ConvertVector3(pivotChild));
  }
  // If only the parent exists, then create a joint between the parent
  // and the world.
  else if (bulletParentLink)
  {
    this->bulletBall = new btPoint2PointConstraint(
      *bulletParentLink->BtLink(),
      BulletTypes::ConvertVector3(pivotParent));
  }
  else
  {
    gzerr << "joint without links\n";
    return;
  }

  this->bulletJointDPtr->constraint = this->bulletBall;

  // Add the joint to the world
  GZ_ASSERT(this->bulletJointDPtr->bulletWorld, "bullet world pointer is null");
  this->bulletJointDPtr->bulletWorld->addConstraint(
      this->bulletJointDPtr->constraint);

  // Allows access to impulse
  this->bulletJointDPtr->constraint->enableFeedback(true);

  // Setup Joint force and torque feedback
  this->SetupJointFeedback();
}

//////////////////////////////////////////////////
ignition::math::Vector3d BulletBallJoint::Anchor(
    const unsigned int /*_index*/) const
{
  return this->bulletJointDPtr->anchorPos;
}

/////////////////////////////////////////////////
void BulletBallJoint::SetVelocity(const unsigned int /*_index*/,
    const double /*_angle*/)
{
  gzerr << "Not implemented\n";
}

/////////////////////////////////////////////////
double BulletBallJoint::Velocity(const unsigned int /*_index*/) const
{
  gzerr << "Not implemented\n";
  return 0;
}

/////////////////////////////////////////////////
ignition::math::Vector3d BulletBallJoint::GlobalAxis(
    const unsigned int /*_index*/) const
{
  gzerr << "Not implemented\n";
  return ignition::math::Vector3d::Zero;
}

/////////////////////////////////////////////////
ignition::math::Angle BulletBallJoint::AngleImpl(
    const unsigned int /*_index*/) const
{
  gzerr << "Not implemented\n";
  return ignition::math::Angle::Zero;
}

//////////////////////////////////////////////////
bool BulletBallJoint::SetHighStop(const unsigned int /*_index*/,
    const ignition::math::Angle &/*_angle*/)
{
  if (this->bulletBall)
  {
    // this function has additional parameters that we may one day
    // implement. Be warned that this function will reset them to default
    // settings
    // this->bulletBall->setLimit(this->btBall->getLowerLimit(),
    //                         _angle.Radian());
    gzerr << "BulletBallJoint limits not implemented" << std::endl;
    return false;
  }
  else
  {
    gzerr << "bulletBall does not yet exist" << std::endl;
    return false;
  }
}

//////////////////////////////////////////////////
void BulletBallJoint::SetForceImpl(
    const unsigned int /*_index*/, const double /*_torque*/)
{
  gzerr << "Not implemented";
}

//////////////////////////////////////////////////
bool BulletBallJoint::SetLowStop(unsigned int /*_index*/,
    const ignition::math::Angle &/*_angle*/)
{
  if (this->bulletBall)
  {
    // this function has additional parameters that we may one day
    // implement. Be warned that this function will reset them to default
    // settings
    // this->bulletBall->setLimit(-_angle.Radian(),
    //                         this->bulletBall->getUpperLimit());
    gzerr << "BulletBallJoint limits not implemented" << std::endl;
    return false;
  }
  else
  {
    gzerr << "bulletBall does not yet exist" << std::endl;
    return false;
  }
}

//////////////////////////////////////////////////
ignition::math::Vector3d BulletBallJoint::Axis(
    const unsigned int /*_index*/) const
{
  return ignition::math::Vector3d::Zero;
}

//////////////////////////////////////////////////
void BulletBallJoint::SetAxis(const unsigned int /*_index*/,
    const ignition::math::Vector3d &/*_axis*/)
{
  gzerr << "BulletBallJoint::SetAxis not implemented" << std::endl;
}

//////////////////////////////////////////////////
ignition::math::Angle BulletBallJoint::HighStop(const unsigned int /*_index*/)
{
  gzerr << "BulletBallJoint::GetHighStop not implemented" << std::endl;
  return ignition::math::Angle();
}

//////////////////////////////////////////////////
ignition::math::Angle BulletBallJoint::LowStop(const unsigned int /*_index*/)
{
  gzerr << "BulletBallJoint::GetLowStop not implemented" << std::endl;
  return ignition::math::Angle();
}
