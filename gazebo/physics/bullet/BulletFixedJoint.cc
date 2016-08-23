/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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
#include "gazebo/physics/bullet/BulletFixedJoint.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
BulletFixedJoint::BulletFixedJoint(btDynamicsWorld *_world, BasePtr _parent)
    : FixedJoint<BulletJoint>(_parent)
{
  GZ_ASSERT(_world, "bullet world pointer is NULL");
  this->bulletWorld = _world;
  this->bulletFixed = NULL;
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
      boost::static_pointer_cast<BulletLink>(this->childLink);
  BulletLinkPtr bulletParentLink =
      boost::static_pointer_cast<BulletLink>(this->parentLink);

  // Get axis unit vector (expressed in world frame).
  math::Vector3 axis = math::Vector3::UnitZ;

  // Local variables used to compute pivots and axes in body-fixed frames
  // for the parent and child links.
  math::Vector3 pivotParent, pivotChild, axisParent, axisChild;
  math::Pose pose;

  // Initialize pivots to anchorPos, which is expressed in the
  // world coordinate frame.
  pivotParent = this->anchorPos;
  pivotChild = this->anchorPos;

  // Check if parentLink exists. If not, the parent will be the world.
  if (this->parentLink)
  {
    // Compute relative pose between joint anchor and CoG of parent link.
    pose = this->parentLink->GetWorldCoGPose();
    // Subtract CoG position from anchor position, both in world frame.
    pivotParent -= pose.pos;
    // Rotate pivot offset and axis into body-fixed frame of parent.
    pivotParent = pose.rot.RotateVectorReverse(pivotParent);
    axisParent = pose.rot.RotateVectorReverse(axis);
    axisParent = axisParent.Normalize();
  }
  // Check if childLink exists. If not, the child will be the world.
  if (this->childLink)
  {
    // Compute relative pose between joint anchor and CoG of child link.
    pose = this->childLink->GetWorldCoGPose();
    // Subtract CoG position from anchor position, both in world frame.
    pivotChild -= pose.pos;
    // Rotate pivot offset and axis into body-fixed frame of child.
    pivotChild = pose.rot.RotateVectorReverse(pivotChild);
    axisChild = pose.rot.RotateVectorReverse(axis);
    axisChild = axisChild.Normalize();
  }

  // If both links exist, then create a joint between the two links.
  if (bulletChildLink && bulletParentLink)
  {
    this->bulletFixed = new btHingeConstraint(
        *(bulletChildLink->GetBulletLink()),
        *(bulletParentLink->GetBulletLink()),
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
        *(bulletChildLink->GetBulletLink()),
        BulletTypes::ConvertVector3(pivotChild),
        BulletTypes::ConvertVector3(axisChild));
  }
  // If only the parent exists, then create a joint between the parent
  // and the world.
  else if (bulletParentLink)
  {
    this->bulletFixed = new btHingeConstraint(
        *(bulletParentLink->GetBulletLink()),
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
  this->constraint = this->bulletFixed;

  this->bulletFixed->setLimit(0.0, 0.0);

  // Add the joint to the world
  GZ_ASSERT(this->bulletWorld, "bullet world pointer is NULL");
  this->bulletWorld->addConstraint(this->bulletFixed, true);

  // Allows access to impulse
  this->bulletFixed->enableFeedback(true);

  // Setup Joint force and torque feedback
  this->SetupJointFeedback();
}

//////////////////////////////////////////////////
void BulletFixedJoint::SetAxis(unsigned int /*_index*/,
    const math::Vector3 &/*_axis*/)
{
  gzwarn << "BulletFixedJoint: called method "
         << "SetAxis that is not valid for joints of type fixed.\n";
}

//////////////////////////////////////////////////
math::Angle BulletFixedJoint::GetAngleImpl(unsigned int /*_index*/) const
{
  gzwarn << "BulletFixedJoint: called method "
         << "GetAngleImpl that is not valid for joints of type fixed.\n";
  return math::Angle();
}

//////////////////////////////////////////////////
void BulletFixedJoint::SetVelocity(unsigned int /*_index*/, double /*_angle*/)
{
  gzwarn << "BulletFixedJoint: called method "
         << "SetVelocity that is not valid for joints of type fixed.\n";
}

//////////////////////////////////////////////////
double BulletFixedJoint::GetVelocity(unsigned int /*_index*/) const
{
  gzwarn << "BulletFixedJoint: called method "
         << "GetVelocity that is not valid for joints of type fixed.\n";
  return 0.0;
}

//////////////////////////////////////////////////
void BulletFixedJoint::SetMaxForce(unsigned int /*_index*/, double /*_t*/)
{
  gzwarn << "BulletFixedJoint: called method "
         << "SetMaxForce that is not valid for joints of type fixed.\n";
}

//////////////////////////////////////////////////
double BulletFixedJoint::GetMaxForce(unsigned int /*_index*/)
{
  gzwarn << "BulletFixedJoint: called method "
         << "GetMaxForce that is not valid for joints of type fixed.\n";
  return 0.0;
}

//////////////////////////////////////////////////
void BulletFixedJoint::SetForceImpl(unsigned int /*_index*/, double /*_effort*/)
{
  gzwarn << "BulletFixedJoint: called method "
         << "SetForceImpl that is not valid for joints of type fixed.\n";
  return;
}

//////////////////////////////////////////////////
bool BulletFixedJoint::SetHighStop(unsigned int /*_index*/,
                      const math::Angle &/*_angle*/)
{
  gzwarn << "BulletFixedJoint: called method "
         << "SetHighStop that is not valid for joints of type fixed.\n";
  return false;
}

//////////////////////////////////////////////////
bool BulletFixedJoint::SetLowStop(unsigned int /*_index*/,
                     const math::Angle &/*_angle*/)
{
  gzwarn << "BulletFixedJoint: called method "
         << "SetLowStop that is not valid for joints of type fixed.\n";
  return false;
}

//////////////////////////////////////////////////
math::Vector3 BulletFixedJoint::GetGlobalAxis(unsigned int /*_index*/) const
{
  gzwarn << "BulletFixedJoint: called method "
         << "GetGlobalAxis that is not valid for joints of type fixed.\n";
  return math::Vector3();
}
