/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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

  BulletLinkPtr bulletChildLink =
    boost::static_pointer_cast<BulletLink>(this->childLink);
  BulletLinkPtr bulletParentLink =
    boost::static_pointer_cast<BulletLink>(this->parentLink);

  // The constructor for btGeneric6DofSpringConstraint requires
  // the frame of fixed joint expressed both in the bullet link frames
  // for both the parent and the child link. Please note that the bullet
  // link frames are different from the Gazebo link frames, because
  // the bullet link frames are centered in the center of mass
  // and are oriented with the principal inertia axes of the link.
  // Furthermore the current Bullet link implementation in Gazebo
  // is assuming that the Bullet link frame is coincident with the
  // SDF Inertial frame, disregarding any off diagonal inertia tensor element.
  btTransform jointPoseWrtParentLinkBullet, jointPoseWrtChildLinkBullet;

  // Check if parentLink exists. If not, the parent will be the world.
  if (this->parentLink)
  {
    math::Pose linkPoseBulletWrtWorld =
        BulletTypes::ConvertPose(
            bulletParentLink->GetBulletLink()->getCenterOfMassTransform());
    math::Pose jointPoseWrtWorld      = this->GetWorldPose();
    math::Pose jointPoseWrtLinkPoseBullet =
        jointPoseWrtWorld + (linkPoseBulletWrtWorld.GetInverse());
    jointPoseWrtParentLinkBullet =
        BulletTypes::ConvertPose(jointPoseWrtLinkPoseBullet);
  }

  // Check if childLink exists. If not, the child will be the world.
  if (this->childLink)
  {
    math::Pose linkPoseBulletWrtWorld =
        BulletTypes::ConvertPose(
            bulletChildLink->GetBulletLink()->getCenterOfMassTransform());
    math::Pose jointPoseWrtWorld      = this->GetWorldPose();
    math::Pose jointPoseWrtLinkPoseBullet =
        jointPoseWrtWorld + (linkPoseBulletWrtWorld.GetInverse());
    jointPoseWrtChildLinkBullet =
        BulletTypes::ConvertPose(jointPoseWrtLinkPoseBullet);
  }

  // If both links exist, then create a joint between the two links.
  if (bulletChildLink && bulletParentLink)
  {
    this->bulletFixed = new btGeneric6DofSpringConstraint(
        *bulletParentLink->GetBulletLink(),
        *bulletChildLink->GetBulletLink(),
        jointPoseWrtParentLinkBullet, jointPoseWrtChildLinkBullet, true);
  }

  // If only the child exists, then create a joint between the child
  // and the world.
  else if (bulletChildLink)
  {
    this->bulletFixed = new btGeneric6DofSpringConstraint(
        *bulletChildLink->GetBulletLink(), jointPoseWrtChildLinkBullet, true);
  }
  // If only the parent exists, then create a joint between the parent
  // and the world.
  else if (bulletParentLink)
  {
    this->bulletFixed = new btGeneric6DofSpringConstraint(
        *bulletParentLink->GetBulletLink(), jointPoseWrtParentLinkBullet, true);
  }
  // Throw an error if no links are given.
  else
  {
    gzerr << "joint without links\n";
    return;
  }

  if (!this->bulletFixed)
  {
    gzerr << "unable to create bullet fixed joint\n";
    return;
  }

  this->constraint = this->bulletFixed;

  // Set the limit value for all degrees of freedom
  this->bulletFixed->setLinearLowerLimit(btVector3(0,0,0));
  this->bulletFixed->setLinearUpperLimit(btVector3(0,0,0));
  this->bulletFixed->setAngularLowerLimit(btVector3(0,0,0));
  this->bulletFixed->setAngularUpperLimit(btVector3(0,0,0));

  // Add the joint to the world
  GZ_ASSERT(this->bulletWorld, "bullet world pointer is NULL");
  this->bulletWorld->addConstraint(this->bulletFixed, true);

  // Allows access to impulse
  this->constraint->enableFeedback(true);

  // Setup Joint force and torque feedback
  this->SetupJointFeedback();
}

//////////////////////////////////////////////////
void BulletFixedJoint::SetAxis(unsigned int /*_index*/,
    const math::Vector3 &/*_axis*/)
{
  gzerr << "called invalid method SetAxis in a fixed joint\n";

  return;
}

//////////////////////////////////////////////////
math::Angle BulletFixedJoint::GetAngleImpl(unsigned int /*_index*/) const
{
  math::Angle result;

  gzerr << "called invalid method GetAngleImpl in a fixed joint\n";

  return result;
}

//////////////////////////////////////////////////
void BulletFixedJoint::SetVelocity(unsigned int /*_index*/, double /*_angle*/)
{
  gzerr << "called invalid method SetVelocity in a fixed joint\n";
}

//////////////////////////////////////////////////
double BulletFixedJoint::GetVelocity(unsigned int /*_index*/) const
{
  gzerr << "called invalid method GetVelocity in a fixed joint\n";

  return 0.0;
}

//////////////////////////////////////////////////
void BulletFixedJoint::SetMaxForce(unsigned int /*_index*/, double /*_t*/)
{
  gzerr << "called invalid method SetMaxForce in a fixed joint\n";

  return;
}

//////////////////////////////////////////////////
double BulletFixedJoint::GetMaxForce(unsigned int /*_index*/)
{
  gzerr << "called invalid method GetMaxForce in a fixed joint\n";

  return 0.0;
}

//////////////////////////////////////////////////
void BulletFixedJoint::SetForceImpl(unsigned int /*_index*/, double /*_effort*/)
{
  gzerr << "called invalid method SetForceImpl in a fixed joint\n";

  return;
}

//////////////////////////////////////////////////
bool BulletFixedJoint::SetHighStop(unsigned int /*_index*/,
                      const math::Angle &/*_angle*/)
{
  gzerr << "called invalid method SetHighStop in a fixed joint\n";

  return false;
}

//////////////////////////////////////////////////
bool BulletFixedJoint::SetLowStop(unsigned int /*_index*/,
                     const math::Angle &/*_angle*/)
{
  gzerr << "called invalid method SetLowStop in a fixed joint\n";

  return false;
}

//////////////////////////////////////////////////
math::Vector3 BulletFixedJoint::GetGlobalAxis(unsigned int /*_index*/) const
{
  gzerr << "called invalid method GetGlobalAxis in a fixed joint\n";

  return math::Vector3();
}
