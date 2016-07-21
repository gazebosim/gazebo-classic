/*
 * Copyright (C) 2015-2016 Open Source Robotics Foundation
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

#include "gazebo/physics/Model.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/physics/bullet/BulletLink.hh"
#include "gazebo/physics/bullet/BulletPhysics.hh"
#include "gazebo/physics/bullet/BulletJointPrivate.hh"
#include "gazebo/physics/bullet/BulletFixedJoint.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
BulletFixedJoint::BulletFixedJoint(btDynamicsWorld *_world, BasePtr _parent)
: FixedJoint<BulletJoint>(_parent)
{
  GZ_ASSERT(_world, "bullet world pointer is null");
  this->bulletJointDPtr->bulletWorld = _world;
  this->bulletFixed = nullptr;
}

//////////////////////////////////////////////////
BulletFixedJoint::~BulletFixedJoint()
{
}

//////////////////////////////////////////////////
void BulletFixedJoint::Load(sdf::ElementPtr _sdf)
{
  FixedJoint<BulletJoint>::Load(_sdf);
}

//////////////////////////////////////////////////
void BulletFixedJoint::Init()
{
  FixedJoint<BulletJoint>::Init();

  // Cast to BulletLink
  BulletLinkPtr bulletChildLink =
      std::static_pointer_cast<BulletLink>(this->bulletJointDPtr->childLink);
  BulletLinkPtr bulletParentLink =
      std::static_pointer_cast<BulletLink>(this->bulletJointDPtr->parentLink);

  // Get axis unit vector (expressed in world frame).
  ignition::math::Vector3d axis = ignition::math::Vector3d::UnitZ;

  // Local variables used to compute pivots and axes in body-fixed frames
  // for the parent and child links.
  ignition::math::Vector3d pivotParent, pivotChild, axisParent, axisChild;
  ignition::math::Pose3d pose;

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
    pivotParent -= pose.Pos();

    // Rotate pivot offset and axis into body-fixed frame of parent.
    pivotParent = pose.Rot().RotateVectorReverse(pivotParent);
    axisParent = pose.Rot().RotateVectorReverse(axis);
    axisParent = axisParent.Normalize();
  }
  // Check if childLink exists. If not, the child will be the world.
  if (this->bulletJointDPtr->childLink)
  {
    // Compute relative pose between joint anchor and CoG of child link.
    pose = this->bulletJointDPtr->childLink->WorldCoGPose();

    // Subtract CoG position from anchor position, both in world frame.
    pivotChild -= pose.Pos();

    // Rotate pivot offset and axis into body-fixed frame of child.
    pivotChild = pose.Rot().RotateVectorReverse(pivotChild);
    axisChild = pose.Rot().RotateVectorReverse(axis);
    axisChild = axisChild.Normalize();
  }

  // If both links exist, then create a joint between the two links.
  if (bulletChildLink && bulletParentLink)
  {
    this->bulletFixed = new btHingeConstraint(
        *(bulletChildLink->BtLink()),
        *(bulletParentLink->BtLink()),
        BulletTypes::ConvertVector3(pivotChild),
        BulletTypes::ConvertVector3(pivotParent),
        BulletTypes::ConvertVector3(axisChild),
        BulletTypes::ConvertVector3(axisParent));
  }
  // If only the child exists, then create a joint between the child
  // and the world.
  else if (bulletChildLink)
  {
    this->bulletFixed = new btHingeConstraint(
        *(bulletChildLink->BtLink()),
        BulletTypes::ConvertVector3(pivotChild),
        BulletTypes::ConvertVector3(axisChild));
  }
  // If only the parent exists, then create a joint between the parent
  // and the world.
  else if (bulletParentLink)
  {
    this->bulletFixed = new btHingeConstraint(
        *(bulletParentLink->BtLink()),
        BulletTypes::ConvertVector3(pivotParent),
        BulletTypes::ConvertVector3(axisParent));
  }
  // Throw an error if no links are given.
  else
  {
    gzerr << "unable to create bullet hinge without links.\n";
    return;
  }

  if (!this->bulletFixed)
  {
    gzerr << "unable to create bullet hinge constraint\n";
    return;
  }

  // Give parent class BulletJoint a pointer to this constraint.
  this->bulletJointDPtr->constraint = this->bulletFixed;

  this->bulletFixed->setLimit(0.0, 0.0);

  // Add the joint to the world
  GZ_ASSERT(this->bulletJointDPtr->bulletWorld, "bullet world pointer is null");
  this->bulletJointDPtr->bulletWorld->addConstraint(this->bulletFixed, true);

  // Allows access to impulse
  this->bulletFixed->enableFeedback(true);

  // Setup Joint force and torque feedback
  this->SetupJointFeedback();
}

//////////////////////////////////////////////////
void BulletFixedJoint::SetAxis(const unsigned int /*_index*/,
    const ignition::math::Vector3d &/*_axis*/)
{
  gzwarn << "BulletFixedJoint: called method "
         << "SetAxis that is not valid for joints of type fixed.\n";
}

//////////////////////////////////////////////////
ignition::math::Angle BulletFixedJoint::AngleImpl(
    const unsigned int /*_index*/) const
{
  gzwarn << "BulletFixedJoint: called method "
         << "AngleImpl that is not valid for joints of type fixed.\n";
  return ignition::math::Angle::Zero;
}

//////////////////////////////////////////////////
void BulletFixedJoint::SetVelocity(const unsigned int /*_index*/,
    const double /*_angle*/)
{
  gzwarn << "BulletFixedJoint: called method "
         << "SetVelocity that is not valid for joints of type fixed.\n";
}

//////////////////////////////////////////////////
double BulletFixedJoint::Velocity(const unsigned int /*_index*/) const
{
  gzwarn << "BulletFixedJoint: called method "
         << "GetVelocity that is not valid for joints of type fixed.\n";
  return 0.0;
}

//////////////////////////////////////////////////
void BulletFixedJoint::SetForceImpl(const unsigned int /*_index*/,
    const double /*_effort*/)
{
  gzwarn << "BulletFixedJoint: called method "
         << "SetForceImpl that is not valid for joints of type fixed.\n";
  return;
}

//////////////////////////////////////////////////
bool BulletFixedJoint::SetHighStop(const unsigned int /*_index*/,
                      const ignition::math::Angle &/*_angle*/)
{
  gzwarn << "BulletFixedJoint: called method "
         << "SetHighStop that is not valid for joints of type fixed.\n";
  return false;
}

//////////////////////////////////////////////////
bool BulletFixedJoint::SetLowStop(const unsigned int /*_index*/,
                     const ignition::math::Angle &/*_angle*/)
{
  gzwarn << "BulletFixedJoint: called method "
         << "SetLowStop that is not valid for joints of type fixed.\n";
  return false;
}

//////////////////////////////////////////////////
ignition::math::Vector3d BulletFixedJoint::GlobalAxis(
    const unsigned int /*_index*/) const
{
  gzwarn << "BulletFixedJoint: called method "
         << "GetGlobalAxis that is not valid for joints of type fixed.\n";
  return ignition::math::Vector3d::Zero;
}
