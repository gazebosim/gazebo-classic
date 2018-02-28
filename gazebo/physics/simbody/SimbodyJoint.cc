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

#include <string>

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
  this->simbodyPhysics = boost::dynamic_pointer_cast<SimbodyPhysics>(
    this->model->GetWorld()->Physics());

  Joint::Load(_sdf);

  // read must_be_loop_joint
  // \TODO: clean up
  if (_sdf->HasElement("physics") &&
    _sdf->GetElement("physics")->HasElement("simbody"))
  {
    this->mustBreakLoopHere = _sdf->GetElement("physics")->
      GetElement("simbody")->Get<bool>("must_be_loop_joint");
  }

  if (this->sdf->HasElement("axis"))
  {
    sdf::ElementPtr axisElem = this->sdf->GetElement("axis");
    if (axisElem->HasElement("dynamics"))
    {
      sdf::ElementPtr dynamicsElem = axisElem->GetElement("dynamics");

      if (dynamicsElem && dynamicsElem->HasElement("friction"))
      {
        sdf::ElementPtr frictionElem = dynamicsElem->GetElement("friction");
        gzlog << "joint friction not implemented\n";
      }
    }
  }

  if (this->sdf->HasElement("axis2"))
  {
    sdf::ElementPtr axisElem = this->sdf->GetElement("axis2");
    if (axisElem->HasElement("dynamics"))
    {
      sdf::ElementPtr dynamicsElem = axisElem->GetElement("dynamics");

      if (dynamicsElem && dynamicsElem->HasElement("friction"))
      {
        sdf::ElementPtr frictionElem = dynamicsElem->GetElement("friction");
        gzlog << "joint friction not implemented\n";
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

  ignition::math::Pose3d childPose = _sdf->Get<ignition::math::Pose3d>("pose");
  if (_sdf->GetElement("child")->HasElement("pose"))
    childPose = _sdf->GetElement("child")->Get<ignition::math::Pose3d>("pose");

  this->xCB = physics::SimbodyPhysics::Pose2Transform(childPose);

  ignition::math::Pose3d parentPose;
  if (_sdf->GetElement("parent")->HasElement("pose"))
    this->xPA = physics::SimbodyPhysics::GetPose(_sdf->GetElement("parent"));
  else
  {
    SimTK::Transform X_MC, X_MP;
    if (this->parentLink)
    {
      X_MP = physics::SimbodyPhysics::Pose2Transform(
          this->parentLink->RelativePose());
    }
    else
    {
      // TODO: verify
      // parent frame is at the world frame
      X_MP = ~physics::SimbodyPhysics::Pose2Transform(this->model->WorldPose());
    }

    if (this->childLink)
    {
      X_MC = physics::SimbodyPhysics::Pose2Transform(
        this->childLink->RelativePose());
    }
    else
    {
      // TODO: verify
      X_MC = ~physics::SimbodyPhysics::Pose2Transform(this->model->WorldPose());
    }

    const SimTK::Transform X_PC = ~X_MP*X_MC;

    // i.e., A spatially coincident with B
    this->xPA = X_PC*this->xCB;
  }
}

//////////////////////////////////////////////////
void SimbodyJoint::Reset()
{
  Joint::Reset();
}

//////////////////////////////////////////////////
void SimbodyJoint::CacheForceTorque()
{
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
  this->wrench.body1Force =
    -SimbodyPhysics::Vec3ToVector3Ign(reactionForceOnParentBody);
  this->wrench.body1Torque =
    -SimbodyPhysics::Vec3ToVector3Ign(reactionTorqueOnParentBody);

  this->wrench.body2Force =
    -SimbodyPhysics::Vec3ToVector3Ign(reactionForceOnChildBody);
  this->wrench.body2Torque =
    -SimbodyPhysics::Vec3ToVector3Ign(reactionTorqueOnChildBody);
}

//////////////////////////////////////////////////
LinkPtr SimbodyJoint::GetJointLink(unsigned int _index) const
{
  LinkPtr result;

  if (_index == 0 || _index == 1)
  {
    SimbodyLinkPtr simbodyLink1 =
      boost::static_pointer_cast<SimbodyLink>(this->childLink);

    SimbodyLinkPtr simbodyLink2 =
      boost::static_pointer_cast<SimbodyLink>(this->parentLink);
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
void SimbodyJoint::SetAxis(const unsigned int _index,
    const ignition::math::Vector3d &/*_axis*/)
{
  ignition::math::Pose3d parentModelPose;
  if (this->parentLink)
    parentModelPose = this->parentLink->GetModel()->WorldPose();

  // Set joint axis
  // assuming incoming axis is defined in the model frame, so rotate them
  // into the inertial frame
  // TODO: switch so the incoming axis is defined in the child frame.
  ignition::math::Vector3d axis = parentModelPose.Rot().RotateVector(
    this->sdf->GetElement("axis")->Get<ignition::math::Vector3d>("xyz"));

  if (_index == 0)
    this->sdf->GetElement("axis")->GetElement("xyz")->Set(axis);
  else if (_index == 1)
    this->sdf->GetElement("axis2")->GetElement("xyz")->Set(axis);
  else
    gzerr << "SetAxis index [" << _index << "] out of bounds\n";
}

//////////////////////////////////////////////////
JointWrench SimbodyJoint::GetForceTorque(unsigned int /*_index*/)
{
  return this->wrench;
}

//////////////////////////////////////////////////
void SimbodyJoint::SetForce(unsigned int _index, double _force)
{
  double force = Joint::CheckAndTruncateForce(_index, _force);
  this->SaveForce(_index, force);
  this->SetForceImpl(_index, force);

  // for engines that supports auto-disable of links
  if (this->childLink) this->childLink->SetEnabled(true);
  if (this->parentLink) this->parentLink->SetEnabled(true);
}

//////////////////////////////////////////////////
double SimbodyJoint::GetForce(unsigned int _index)
{
  if (_index < this->DOF())
  {
    return this->forceApplied[_index];
  }
  else
  {
    gzerr << "Invalid joint index [" << _index
          << "] when trying to get force\n";
    return 0;
  }
}

//////////////////////////////////////////////////
void SimbodyJoint::SaveForce(unsigned int _index, double _force)
{
  // this bit of code actually doesn't do anything physical,
  // it simply records the forces commanded inside forceApplied.
  if (_index < this->DOF())
  {
    if (this->forceAppliedTime < this->GetWorld()->SimTime())
    {
      // reset forces if time step is new
      this->forceAppliedTime = this->GetWorld()->SimTime();
      this->forceApplied[0] = this->forceApplied[1] = 0;
    }

    this->forceApplied[_index] += _force;
  }
  else
    gzerr << "Something's wrong, joint [" << this->GetName()
          << "] index [" << _index
          << "] out of range.\n";
}

//////////////////////////////////////////////////
void SimbodyJoint::SaveSimbodyState(const SimTK::State &/*_state*/)
{
/*
  // Implementation not complete
  // Not implemented
  // skip if not a free joint, state is saved in SimbodyJoint::mobod
  if (!this->masterMobod.isEmptyHandle() &&
      SimTK::MobilizedBody::Free::isInstanceOf(this->masterMobod))
  {
    if (this->simbodyQ.empty())
      this->simbodyQ.resize(this->masterMobod.getNumQ(_state));

    if (this->simbodyU.empty())
      this->simbodyU.resize(this->masterMobod.getNumU(_state));

    for (unsigned int i = 0; i < this->simbodyQ.size(); ++i)
      this->simbodyQ[i] = this->masterMobod.getOneQ(_state, i);

    for (unsigned int i = 0; i < this->simbodyU.size(); ++i)
      this->simbodyU[i] = this->masterMobod.getOneU(_state, i);
  }
  else
  {
    // gzerr << "debug: joint name: " << this->GetScopedName() << "\n";
  }
*/
}

//////////////////////////////////////////////////
void SimbodyJoint::RestoreSimbodyState(SimTK::State &/*_state*/)
{
  // Not implemented
}

//////////////////////////////////////////////////
void SimbodyJoint::SetAnchor(const unsigned int /*_index*/,
    const ignition::math::Vector3d & /*_anchor*/)
{
  gzerr << "SimbodyJoint::SetAnchor:  Not implement in Simbody."
        << " Anchor is set during joint construction in SimbodyPhysics.cc\n";
}

//////////////////////////////////////////////////
void SimbodyJoint::SetDamping(unsigned int _index, const double _damping)
{
  if (_index < this->DOF())
  {
    this->SetStiffnessDamping(_index, this->stiffnessCoefficient[_index],
      _damping);
  }
  else
  {
     gzerr << "SimbodyJoint::SetDamping: index[" << _index
           << "] is out of bounds (DOF() = "
           << this->DOF() << ").\n";
     return;
  }
}

//////////////////////////////////////////////////
void SimbodyJoint::SetStiffness(unsigned int _index, const double _stiffness)
{
  if (_index < this->DOF())
  {
    this->SetStiffnessDamping(_index, _stiffness,
      this->dissipationCoefficient[_index]);
  }
  else
  {
     gzerr << "SimbodyJoint::SetStiffness: index[" << _index
           << "] is out of bounds (DOF() = "
           << this->DOF() << ").\n";
     return;
  }
}

//////////////////////////////////////////////////
void SimbodyJoint::SetStiffnessDamping(unsigned int _index,
  double _stiffness, double _damping, double _reference)
{
  if (_index < this->DOF())
  {
    this->stiffnessCoefficient[_index] = _stiffness;
    this->dissipationCoefficient[_index] = _damping;
    this->springReferencePosition[_index] = _reference;

    if (this->physicsInitialized)
    {
      // set damper coefficient
      this->damper[_index].setDamping(
        this->simbodyPhysics->integ->updAdvancedState(),
        _damping);

      // set spring stiffness and reference position
      this->spring[_index].setStiffness(
        this->simbodyPhysics->integ->updAdvancedState(),
        _stiffness);
      this->spring[_index].setQZero(
        this->simbodyPhysics->integ->updAdvancedState(),
        _reference);
    }
  }
  else
    gzerr << "SetStiffnessDamping _index too large.\n";
}

//////////////////////////////////////////////////
ignition::math::Vector3d SimbodyJoint::Anchor(
    const unsigned int /*_index*/) const
{
  gzerr << "Not implement in Simbody\n";
  return ignition::math::Vector3d::Zero;
}

//////////////////////////////////////////////////
bool SimbodyJoint::SetPosition(
    const unsigned int /*_index*/, const double /*_position*/,
    const bool /*_preserveWorldVelocity*/)
{
  gzerr << "Joint::SetPosition is not available for Simbody\n";
  return false;
}

//////////////////////////////////////////////////
ignition::math::Vector3d SimbodyJoint::LinkForce(
          const unsigned int /*_index*/) const
{
  gzerr << "Not implement in Simbody\n";
  return ignition::math::Vector3d::Zero;
}

//////////////////////////////////////////////////
ignition::math::Vector3d SimbodyJoint::LinkTorque(
          const unsigned int /*_index*/) const
{
  gzerr << "Not implement in Simbody\n";
  return ignition::math::Vector3d::Zero;
}

//////////////////////////////////////////////////
bool SimbodyJoint::SetParam(const std::string &/*_key*/,
    unsigned int /*_index*/, const boost::any &/*_value*/)
{
  gzerr << "Not implement in Simbody\n";
  return false;
}

//////////////////////////////////////////////////
double SimbodyJoint::GetParam(const std::string &_key, unsigned int _index)
{
  return Joint::GetParam(_key, _index);
}

//////////////////////////////////////////////////
void SimbodyJoint::SetUpperLimit(const unsigned int _index,
                                 const double _limit)
{
  Joint::SetUpperLimit(_index, _limit);

  if (_index < this->DOF())
  {
    if (this->physicsInitialized)
    {
      if (!this->limitForce[_index].isEmptyHandle())
      {
        this->limitForce[_index].setBounds(
          this->simbodyPhysics->integ->updAdvancedState(),
          this->LowerLimit(_index), _limit);
      }
      else
      {
        gzerr << "child link is null, force element not initialized, "
              << "SetUpperLimit failed. Please file a report on issue "
              << "tracker.\n";
      }
    }
    else
    {
      gzerr << "SetUpperLimit: State not initialized, SetUpperLimit failed.\n";
    }
  }
  else
  {
    gzerr << "SetUpperLimit: index out of bounds.\n";
  }
}

//////////////////////////////////////////////////
void SimbodyJoint::SetLowerLimit(const unsigned int _index,
                                 const double _limit)
{
  Joint::SetLowerLimit(_index, _limit);

  if (_index < this->DOF())
  {
    if (this->physicsInitialized)
    {
      if (!this->limitForce[_index].isEmptyHandle())
      {
        this->limitForce[_index].setBounds(
          this->simbodyPhysics->integ->updAdvancedState(),
          _limit, this->UpperLimit(_index));
      }
      else
      {
        gzerr << "child link is null, force element not initialized, "
              << "SetLowerLimit failed. Please file a report on issue "
              << "tracker.\n";
      }
    }
    else
    {
      gzerr << "SetLowerLimit: State not initialized, SetLowerLimit failed.\n";
    }
  }
  else
  {
    gzerr << "SetLowerLimit: index out of bounds.\n";
  }
}

//////////////////////////////////////////////////
double SimbodyJoint::UpperLimit(const unsigned int _index) const
{
  /// \todo Simbody is getting the limit from SDF, maybe it should use the base
  /// class Joint::UpperLimit.
  if (_index >= this->DOF())
  {
    gzerr << "Invalid joint index [" << _index
          << "] when trying to get high stop\n";
    return ignition::math::NAN_D;
  }
  else if (_index == 0)
  {
    return this->sdf->GetElement("axis")->GetElement("limit")
             ->Get<double>("upper");
  }
  else if (_index == 1)
  {
    return this->sdf->GetElement("axis2")->GetElement("limit")
             ->Get<double>("upper");
  }
  else
  {
    gzerr << "Should not be here in code, DOF > 2?\n";
    return ignition::math::NAN_D;
  }
}

//////////////////////////////////////////////////
double SimbodyJoint::LowerLimit(const unsigned int _index) const
{
  /// \todo Simbody is getting the limit from SDF, maybe it should use the base
  /// class Joint::LowerLimit.
  if (_index >= this->DOF())
  {
    gzerr << "Invalid joint index [" << _index
          << "] when trying to get low stop\n";
    return ignition::math::NAN_D;
  }
  else if (_index == 0)
  {
    return this->sdf->GetElement("axis")->GetElement("limit")
             ->Get<double>("lower");
  }
  else if (_index == 1)
  {
    return this->sdf->GetElement("axis2")->GetElement("limit")
             ->Get<double>("lower");
  }
  else
  {
    gzerr << "Should not be here in code, DOF > 2?\n";
    return ignition::math::NAN_D;
  }
}
