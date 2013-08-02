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
/* Desc: The base Simbody joint class
 * Author: Nate Koenig, Andrew Howard
 * Date: 15 May 2009
 */

#include "gazebo/common/Exception.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/physics/Model.hh"
#include "gazebo/physics/World.hh"

#include "gazebo/physics/simbody/simbody_inc.h"
#include "gazebo/physics/simbody/SimbodyLink.hh"
#include "gazebo/physics/simbody/SimbodyJoint.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
SimbodyJoint::SimbodyJoint(BasePtr _parent)
  : Joint(_parent)
{
  this->isReversed = false;
  this->mustBreakLoopHere = false;
}

//////////////////////////////////////////////////
SimbodyJoint::~SimbodyJoint()
{
}

//////////////////////////////////////////////////
void SimbodyJoint::Load(sdf::ElementPtr _sdf)
{
  // store a pointer to the simbody physics engine for convenience
  this->simbodyPhysics = boost::shared_dynamic_cast<SimbodyPhysics>(
    this->model->GetWorld()->GetPhysicsEngine());

  Joint::Load(_sdf);

  // read must_be_loop_joint
  // \TODO: clean up
  if (_sdf->HasElement("physics") &&
    _sdf->GetElement("physics")->HasElement("simbody"))
  {
    this->mustBreakLoopHere = _sdf->GetElement("physics")->
      GetElement("simbody")->GetValueBool("must_be_loop_joint");
  }

  if (this->sdf->HasElement("axis"))
  {
    sdf::ElementPtr axisElem = this->sdf->GetElement("axis");
    if (axisElem->HasElement("dynamics"))
    {
      sdf::ElementPtr dynamicsElem = axisElem->GetElement("dynamics");

      /// \TODO: switch to GetElement so default values apply
      /// \TODO: check all physics engines
      if (dynamicsElem->HasElement("damping"))
      {
        this->dampingCoefficient = dynamicsElem->GetValueDouble("damping");
      }
    }
  }

  // Read old style
  //    <pose>pose on child</pose>
  // or new style

  // to support alternative unassembled joint pose specification
  // check if the new style of pose specification exists
  //    <parent>
  //      <link>parentName</link>
  //      <pose>parentPose</pose>
  //    </parent>
  // as compared to old style
  //    <parent>parentName</parent>
  //
  // \TODO: consider storing the unassembled format parent pose when
  // calling Joint::Load(sdf::ElementPtr)

  math::Pose childPose = _sdf->Get<math::Pose>("pose");
  if (_sdf->GetElement("child")->HasElement("pose"))
    childPose = _sdf->GetElement("child")->Get<math::Pose>("pose");

  this->X_CB = physics::SimbodyPhysics::Pose2Transform(childPose);

  math::Pose parentPose;
  if (_sdf->GetElement("parent")->HasElement("pose"))
    this->X_PA = physics::SimbodyPhysics::GetPose(_sdf->GetElement("parent"));
  else
  {
    SimTK::Transform X_MC, X_MP;
    if (this->parentLink)
    {
      X_MP = physics::SimbodyPhysics::Pose2Transform(
        this->parentLink->GetRelativePose());
    }
    else
    {
      // TODO: verify
      // parent frame is at the world frame
      X_MP = ~physics::SimbodyPhysics::Pose2Transform(
        this->model->GetWorldPose());
    }

    if (this->childLink)
    {
      X_MC = physics::SimbodyPhysics::Pose2Transform(
        this->childLink->GetRelativePose());
    }
    else
    {
      // TODO: verify
      X_MC = ~physics::SimbodyPhysics::Pose2Transform(
        this->model->GetWorldPose());
    }

    const SimTK::Transform X_PC = ~X_MP*X_MC;
    this->X_PA = X_PC*this->X_CB; // i.e., A spatially coincident with B 
  }
}

//////////////////////////////////////////////////
void SimbodyJoint::Reset()
{
  Joint::Reset();
}

//////////////////////////////////////////////////
LinkPtr SimbodyJoint::GetJointLink(int _index) const
{
  LinkPtr result;

  if (_index == 0 || _index == 1)
  {
    SimbodyLinkPtr simbodyLink1 =
      boost::shared_static_cast<SimbodyLink>(this->childLink);

    SimbodyLinkPtr simbodyLink2 =
      boost::shared_static_cast<SimbodyLink>(this->parentLink);
  }

  return result;
}

//////////////////////////////////////////////////
bool SimbodyJoint::AreConnected(LinkPtr _one, LinkPtr _two) const
{
  return ((this->childLink.get() == _one.get() &&
           this->parentLink.get() == _two.get()) ||
          (this->childLink.get() == _two.get() &&
           this->parentLink.get() == _one.get()));
}

//////////////////////////////////////////////////
void SimbodyJoint::Detach()
{
  this->childLink.reset();
  this->parentLink.reset();
}

//////////////////////////////////////////////////
void SimbodyJoint::SetAxis(int _index, const math::Vector3 &_axis)
{
  if (this->parentLink)
  {
    math::Pose parentModelPose = this->parentLink->GetModel()->GetWorldPose();

    // Set joint axis
    // assuming incoming axis is defined in the model frame, so rotate them
    // into the inertial frame
    // TODO: switch so the incoming axis is defined in the child frame.
    if (this->sdf->HasElement("axis"))
    {
      this->SetAxis(0, parentModelPose.rot.RotateVector(
            this->sdf->GetElement("axis")->Get<math::Vector3>("xyz")));
    }

    if (this->sdf->HasElement("axis2"))
    {
      this->SetAxis(1, parentModelPose.rot.RotateVector(
            this->sdf->GetElement("axis2")->Get<math::Vector3>("xyz")));
    }
  }
  else
  {
    // if parentLink is NULL, it's name be the world
    this->sdf->GetElement("parent")->Set("world");
    if (this->sdf->HasElement("axis"))
    {
      this->SetAxis(0, this->sdf->GetElement("axis")->Get<math::Vector3>("xyz"));
    }
    if (this->sdf->HasElement("axis2"))
    {
      this->SetAxis(1, this->sdf->GetElement("axis2")->Get<math::Vector3>("xyz"));
    }
  }
}

//////////////////////////////////////////////////
JointWrench SimbodyJoint::GetForceTorque(int _index)
{
  return this->GetForceTorque(static_cast<unsigned int>(_index));
}

//////////////////////////////////////////////////
JointWrench SimbodyJoint::GetForceTorque(unsigned int /*_index*/)
{
  JointWrench wrench;
  const SimTK::State &state = this->simbodyPhysics->integ->getAdvancedState();

  // force calculation of reaction forces
  this->simbodyPhysics->system.realize(state);

  // In simbody, parent is always inboard (closer to ground in the tree),
  //   child is always outboard (further away from ground in the tree).
  SimTK::SpatialVec spatialForceOnInboardBodyInGround =
    this->mobod.findMobilizerReactionOnParentAtFInGround(state);

  SimTK::SpatialVec spatialForceOnOutboardBodyInGround =
    this->mobod.findMobilizerReactionOnBodyAtMInGround(state);

  // determine if outboard body is parent or child based on isReversed flag.
  // determine if inboard body is parent or child based on isReversed flag.
  SimTK::SpatialVec spatialForceOnParentBodyInGround;
  SimTK::SpatialVec spatialForceOnChildBodyInGround;
  // parent and child mobods in gazebo's parent/child tree structure.
  SimTK::MobilizedBody parentMobod;
  SimTK::MobilizedBody childMobod;
  if (!this->isReversed)
  {
    spatialForceOnParentBodyInGround = spatialForceOnInboardBodyInGround;
    spatialForceOnChildBodyInGround = spatialForceOnOutboardBodyInGround;
    childMobod = this->mobod;
    parentMobod = this->mobod.getParentMobilizedBody();
  }
  else
  {
    spatialForceOnParentBodyInGround = spatialForceOnOutboardBodyInGround;
    spatialForceOnChildBodyInGround = spatialForceOnInboardBodyInGround;
    childMobod = this->mobod.getParentMobilizedBody();
    parentMobod = this->mobod;
  }

  // get rotation from ground to child/parent link frames
  const SimTK::Rotation& R_GC = childMobod.getBodyRotation(state);
  const SimTK::Rotation& R_GP = parentMobod.getBodyRotation(state);

  // re-express in child link frame
  SimTK::Vec3 reactionTorqueOnChildBody =
    ~R_GC * spatialForceOnChildBodyInGround[0];
  SimTK::Vec3 reactionForceOnChildBody =
    ~R_GC * spatialForceOnChildBodyInGround[1];

  SimTK::Vec3 reactionTorqueOnParentBody =
    ~R_GP * spatialForceOnParentBodyInGround[0];
  SimTK::Vec3 reactionForceOnParentBody =
    ~R_GP * spatialForceOnParentBodyInGround[1];

  // gzerr << "parent[" << this->GetName()
  //       << "]: t[" << reactionTorqueOnParentBody
  //       << "] f[" << reactionForceOnParentBody
  //       << "]\n";

  // gzerr << "child[" << this->GetName()
  //       << "]: t[" << reactionTorqueOnChildBody
  //       << "] f[" << reactionForceOnChildBody
  //       << "]\n";

  // Note minus sign indicates these are reaction forces
  // by the Link on the Joint in the target Link frame.
  wrench.body1Force =
    -SimbodyPhysics::Vec3ToVector3(reactionForceOnParentBody);
  wrench.body1Torque =
    -SimbodyPhysics::Vec3ToVector3(reactionTorqueOnParentBody);

  wrench.body2Force =
    -SimbodyPhysics::Vec3ToVector3(reactionForceOnChildBody);
  wrench.body2Torque =
    -SimbodyPhysics::Vec3ToVector3(reactionTorqueOnChildBody);

  return wrench;
}
