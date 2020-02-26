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
/* Desc: A ball joint
 * Author: Nate Koenig, Andrew Howard
 * Date: 21 May 2003
 */

#include <ignition/math/Helpers.hh>

#include "gazebo/common/Assert.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Exception.hh"

#include "gazebo/physics/bullet/BulletTypes.hh"
#include "gazebo/physics/bullet/BulletLink.hh"
#include "gazebo/physics/bullet/BulletBallJoint.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
BulletBallJoint::BulletBallJoint(btDynamicsWorld *_world, BasePtr _parent)
    : BallJoint<BulletJoint>(_parent)
{
  GZ_ASSERT(_world, "bullet world pointer is null");
  this->bulletWorld = _world;
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
    boost::static_pointer_cast<BulletLink>(this->childLink);
  BulletLinkPtr bulletParentLink =
    boost::static_pointer_cast<BulletLink>(this->parentLink);

  // Local variables used to compute pivots and axes in body-fixed frames
  // for the parent and child links.
  ignition::math::Vector3d pivotParent, pivotChild;
  ignition::math::Pose3d pose;

  // Initialize pivots to anchorPos, which is expressed in the
  // world coordinate frame.
  pivotParent = this->anchorPos;
  pivotChild = this->anchorPos;

  // Check if parentLink exists. If not, the parent will be the world.
  if (this->parentLink)
  {
    // Compute relative pose between joint anchor and inertial frame of parent.
    pose = this->parentLink->WorldInertialPose();
    // Subtract CoG position from anchor position, both in world frame.
    pivotParent -= pose.Pos();
    // Rotate pivot offset and axis into body-fixed inertial frame of parent.
    pivotParent = pose.Rot().RotateVectorReverse(pivotParent);
  }
  // Check if childLink exists. If not, the child will be the world.
  if (this->childLink)
  {
    // Compute relative pose between joint anchor and inertial frame of child.
    pose = this->childLink->WorldInertialPose();
    // Subtract CoG position from anchor position, both in world frame.
    pivotChild -= pose.Pos();
    // Rotate pivot offset and axis into body-fixed inertial frame of child.
    pivotChild = pose.Rot().RotateVectorReverse(pivotChild);
  }

  // If both links exist, then create a joint between the two links.
  if (bulletChildLink && bulletParentLink)
  {
    this->bulletBall = new btPoint2PointConstraint(
      *bulletChildLink->GetBulletLink(),
      *bulletParentLink->GetBulletLink(),
      BulletTypes::ConvertVector3(pivotChild),
      BulletTypes::ConvertVector3(pivotParent));
  }
  // If only the child exists, then create a joint between the child
  // and the world.
  else if (bulletChildLink)
  {
    this->bulletBall = new btPoint2PointConstraint(
      *bulletChildLink->GetBulletLink(),
      BulletTypes::ConvertVector3(pivotChild));
  }
  // If only the parent exists, then create a joint between the parent
  // and the world.
  else if (bulletParentLink)
  {
    this->bulletBall = new btPoint2PointConstraint(
      *bulletParentLink->GetBulletLink(),
      BulletTypes::ConvertVector3(pivotParent));
  }
  // Throw an error if no links are given.
  else
  {
    gzthrow("joint without links\n");
  }

  this->constraint = this->bulletBall;

  // Add the joint to the world
  GZ_ASSERT(this->bulletWorld, "bullet world pointer is null");
  this->bulletWorld->addConstraint(this->constraint);

  // Allows access to impulse
  this->constraint->enableFeedback(true);

  // Setup Joint force and torque feedback
  this->SetupJointFeedback();
}

//////////////////////////////////////////////////
ignition::math::Vector3d BulletBallJoint::Anchor(
    const unsigned int /*_index*/) const
{
  return this->anchorPos;
}

/////////////////////////////////////////////////
void BulletBallJoint::SetVelocity(unsigned int /*_index*/, double /*_angle*/)
{
  gzerr << "Not implemented\n";
}

/////////////////////////////////////////////////
double BulletBallJoint::GetVelocity(unsigned int /*_index*/) const
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
double BulletBallJoint::PositionImpl(const unsigned int /*_index*/) const
{
  gzerr << "BulletBallJoint::PositionImpl not implemented" << std::endl;
  return ignition::math::NAN_D;
}

//////////////////////////////////////////////////
void BulletBallJoint::SetUpperLimit(const unsigned int /*_index*/,
                                    const double /*_angle*/)
{
  if (this->bulletBall)
  {
    // this function has additional parameters that we may one day
    // implement. Be warned that this function will reset them to default
    // settings
    // this->bulletBall->setLimit(this->btBall->getLowerLimit(),
    //                         _angle.Radian());
    gzerr << "BulletBallJoint limits not implemented" << std::endl;
  }
  else
  {
    gzerr << "bulletBall does not yet exist" << std::endl;
  }
}

//////////////////////////////////////////////////
void BulletBallJoint::SetForceImpl(unsigned int /*_index*/, double /*_torque*/)
{
  gzerr << "Not implemented";
}

//////////////////////////////////////////////////
void BulletBallJoint::SetLowerLimit(const unsigned int /*_index*/,
                                    const double /*_angle*/)
{
  if (this->bulletBall)
  {
    // this function has additional parameters that we may one day
    // implement. Be warned that this function will reset them to default
    // settings
    // this->bulletBall->setLimit(-_angle.Radian(),
    //                         this->bulletBall->getUpperLimit());
    gzerr << "BulletBallJoint limits not implemented" << std::endl;
  }
  else
  {
    gzerr << "bulletBall does not yet exist" << std::endl;
  }
}

//////////////////////////////////////////////////
void BulletBallJoint::SetAxis(const unsigned int /*_index*/,
                        const ignition::math::Vector3d &/*_axis*/)
{
  gzerr << "BulletBallJoint::SetAxis not implemented" << std::endl;
}

//////////////////////////////////////////////////
double BulletBallJoint::UpperLimit(const unsigned int /*_index*/) const
{
  gzerr << "BulletBallJoint::UpperLimit not implemented" << std::endl;
  return ignition::math::NAN_D;
}

//////////////////////////////////////////////////
double BulletBallJoint::LowerLimit(const unsigned int /*_index*/) const
{
  gzerr << "BulletBallJoint::LowerLimit not implemented" << std::endl;
  return ignition::math::NAN_D;
}
