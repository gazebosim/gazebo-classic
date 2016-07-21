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
#include <string>

#include "gazebo/common/Exception.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/physics/Model.hh"
#include "gazebo/physics/World.hh"

#include "gazebo/physics/simbody/simbody_inc.h"
#include "gazebo/physics/simbody/SimbodyLink.hh"
#include "gazebo/physics/simbody/SimbodyJointPrivate.hh"
#include "gazebo/physics/simbody/SimbodyJoint.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
SimbodyJoint::SimbodyJoint(BasePtr _parent)
: Joint(*new SimbodyJointPrivate, _parent),
  simbodyJointDPtr(static_cast<SimbodyJointPrivate*>(this->jointDPtr))
{
  this->simbodyJointDPtr->isReversed = false;
  this->simbodyJointDPtr->mustBreakLoopHere = false;
}

//////////////////////////////////////////////////
SimbodyJoint::~SimbodyJoint()
{
}

//////////////////////////////////////////////////
void SimbodyJoint::Load(sdf::ElementPtr _sdf)
{
  // store a pointer to the simbody physics engine for convenience
  this->simbodyJointDPtr->simbodyPhysics =
  std::dynamic_pointer_cast<SimbodyPhysics>(
      this->simbodyJointDPtr->model->World()->Physics());

  Joint::Load(_sdf);

  // read must_be_loop_joint
  // \TODO: clean up
  if (_sdf->HasElement("physics") &&
    _sdf->GetElement("physics")->HasElement("simbody"))
  {
    this->simbodyJointDPtr->mustBreakLoopHere = _sdf->GetElement("physics")->
      GetElement("simbody")->Get<bool>("must_be_loop_joint");
  }

  if (this->simbodyJointDPtr->sdf->HasElement("axis"))
  {
    sdf::ElementPtr axisElem = this->simbodyJointDPtr->sdf->GetElement("axis");
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

  if (this->simbodyJointDPtr->sdf->HasElement("axis2"))
  {
    sdf::ElementPtr axisElem = this->simbodyJointDPtr->sdf->GetElement("axis2");
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

  ignition::math::Pose3d childPose =
    _sdf->Get<ignition::math::Pose3d>("pose");

  if (_sdf->GetElement("child")->HasElement("pose"))
    childPose = _sdf->GetElement("child")->Get<ignition::math::Pose3d>("pose");

  this->simbodyJointDPtr->xCB =
    physics::SimbodyPhysics::Pose2Transform(childPose);

  ignition::math::Pose3d parentPose;
  if (_sdf->GetElement("parent")->HasElement("pose"))
  {
    this->simbodyJointDPtr->xPA =
      physics::SimbodyPhysics::Pose(_sdf->GetElement("parent"));
  }
  else
  {
    SimTK::Transform X_MC, X_MP;
    if (this->simbodyJointDPtr->parentLink)
    {
      X_MP = physics::SimbodyPhysics::Pose2Transform(
        this->simbodyJointDPtr->parentLink->RelativePose());
    }
    else
    {
      // TODO: verify
      // parent frame is at the world frame
      X_MP = ~physics::SimbodyPhysics::Pose2Transform(
        this->simbodyJointDPtr->model->WorldPose());
    }

    if (this->simbodyJointDPtr->childLink)
    {
      X_MC = physics::SimbodyPhysics::Pose2Transform(
        this->simbodyJointDPtr->childLink->RelativePose());
    }
    else
    {
      // TODO: verify
      X_MC = ~physics::SimbodyPhysics::Pose2Transform(
        this->simbodyJointDPtr->model->WorldPose());
    }

    const SimTK::Transform X_PC = ~X_MP*X_MC;

    // i.e., A spatially coincident with B
    this->simbodyJointDPtr->xPA = X_PC*this->simbodyJointDPtr->xCB;
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
  const SimTK::State &state =
    this->simbodyJointDPtr->simbodyPhysics->Integ()->getAdvancedState();

  // force calculation of reaction forces
  this->simbodyJointDPtr->simbodyPhysics->System().realize(state);

  // In simbody, parent is always inboard (closer to ground in the tree),
  //   child is always outboard (further away from ground in the tree).
  SimTK::SpatialVec spatialForceOnInboardBodyInGround =
    this->simbodyJointDPtr->mobod.findMobilizerReactionOnParentAtFInGround(
        state);

  SimTK::SpatialVec spatialForceOnOutboardBodyInGround =
    this->simbodyJointDPtr->mobod.findMobilizerReactionOnBodyAtMInGround(state);

  // determine if outboard body is parent or child based on isReversed flag.
  // determine if inboard body is parent or child based on isReversed flag.
  SimTK::SpatialVec spatialForceOnParentBodyInGround;
  SimTK::SpatialVec spatialForceOnChildBodyInGround;
  // parent and child mobods in gazebo's parent/child tree structure.
  SimTK::MobilizedBody parentMobod;
  SimTK::MobilizedBody childMobod;
  if (!this->simbodyJointDPtr->isReversed)
  {
    spatialForceOnParentBodyInGround = spatialForceOnInboardBodyInGround;
    spatialForceOnChildBodyInGround = spatialForceOnOutboardBodyInGround;
    childMobod = this->simbodyJointDPtr->mobod;
    parentMobod = this->simbodyJointDPtr->mobod.getParentMobilizedBody();
  }
  else
  {
    spatialForceOnParentBodyInGround = spatialForceOnOutboardBodyInGround;
    spatialForceOnChildBodyInGround = spatialForceOnInboardBodyInGround;
    childMobod = this->simbodyJointDPtr->mobod.getParentMobilizedBody();
    parentMobod = this->simbodyJointDPtr->mobod;
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

  // gzerr << "parent[" << this->Name()
  //       << "]: t[" << reactionTorqueOnParentBody
  //       << "] f[" << reactionForceOnParentBody
  //       << "]\n";

  // gzerr << "child[" << this->Name()
  //       << "]: t[" << reactionTorqueOnChildBody
  //       << "] f[" << reactionForceOnChildBody
  //       << "]\n";

  // Note minus sign indicates these are reaction forces
  // by the Link on the Joint in the target Link frame.
  this->simbodyJointDPtr->wrench.body1Force =
    -SimbodyPhysics::Vec3ToVector3Ign(reactionForceOnParentBody);
  this->simbodyJointDPtr->wrench.body1Torque =
    -SimbodyPhysics::Vec3ToVector3Ign(reactionTorqueOnParentBody);

  this->simbodyJointDPtr->wrench.body2Force =
    -SimbodyPhysics::Vec3ToVector3Ign(reactionForceOnChildBody);
  this->simbodyJointDPtr->wrench.body2Torque =
    -SimbodyPhysics::Vec3ToVector3Ign(reactionTorqueOnChildBody);
}

//////////////////////////////////////////////////
LinkPtr SimbodyJoint::JointLink(unsigned int _index) const
{
  LinkPtr result;

  if (_index == 0 || _index == 1)
  {
    SimbodyLinkPtr simbodyLink1 =
      std::static_pointer_cast<SimbodyLink>(this->simbodyJointDPtr->childLink);

    SimbodyLinkPtr simbodyLink2 =
      std::static_pointer_cast<SimbodyLink>(this->simbodyJointDPtr->parentLink);
  }

  return result;
}

//////////////////////////////////////////////////
bool SimbodyJoint::AreConnected(LinkPtr _one, LinkPtr _two) const
{
  return ((this->simbodyJointDPtr->childLink.get() == _one.get() &&
           this->simbodyJointDPtr->parentLink.get() == _two.get()) ||
          (this->simbodyJointDPtr->childLink.get() == _two.get() &&
           this->simbodyJointDPtr->parentLink.get() == _one.get()));
}

//////////////////////////////////////////////////
void SimbodyJoint::Detach()
{
  this->simbodyJointDPtr->childLink.reset();
  this->simbodyJointDPtr->parentLink.reset();
}

//////////////////////////////////////////////////
void SimbodyJoint::SetAxis(const unsigned int _index,
    const ignition::math::Vector3d &/*_axis*/)
{
  ignition::math::Pose3d parentModelPose;
  if (this->simbodyJointDPtr->parentLink)
    parentModelPose =
      this->simbodyJointDPtr->parentLink->ParentModel()->WorldPose();

  // Set joint axis
  // assuming incoming axis is defined in the model frame, so rotate them
  // into the inertial frame
  // TODO: switch so the incoming axis is defined in the child frame.
  ignition::math::Vector3d axis = parentModelPose.Rot().RotateVector(
    this->simbodyJointDPtr->sdf->GetElement(
      "axis")->Get<ignition::math::Vector3d>("xyz"));

  if (_index == 0)
  {
    this->simbodyJointDPtr->sdf->GetElement(
        "axis")->GetElement("xyz")->Set(axis);
  }
  else if (_index == 1)
  {
    this->simbodyJointDPtr->sdf->GetElement(
        "axis2")->GetElement("xyz")->Set(axis);
  }
  else
  {
    gzerr << "SetAxis index [" << _index << "] out of bounds\n";
  }
}

//////////////////////////////////////////////////
JointWrench SimbodyJoint::ForceTorque(const unsigned int /*_index*/) const
{
  return this->simbodyJointDPtr->wrench;
}

//////////////////////////////////////////////////
void SimbodyJoint::SetForce(const unsigned int _index, const double _force)
{
  double force = Joint::CheckAndTruncateForce(_index, _force);
  this->SaveForce(_index, force);
  this->SetForceImpl(_index, force);

  // for engines that supports auto-disable of links
  if (this->simbodyJointDPtr->childLink)
    this->simbodyJointDPtr->childLink->SetEnabled(true);
  if (this->simbodyJointDPtr->parentLink)
    this->simbodyJointDPtr->parentLink->SetEnabled(true);
}

//////////////////////////////////////////////////
double SimbodyJoint::Force(const unsigned int _index) const
{
  if (_index < this->AngleCount())
  {
    return this->simbodyJointDPtr->forceApplied[_index];
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
  if (_index < this->AngleCount())
  {
    if (this->simbodyJointDPtr->forceAppliedTime < this->World()->SimTime())
    {
      // reset forces if time step is new
      this->simbodyJointDPtr->forceAppliedTime = this->World()->SimTime();
      this->simbodyJointDPtr->forceApplied[0] =
        this->simbodyJointDPtr->forceApplied[1] = 0;
    }

    this->simbodyJointDPtr->forceApplied[_index] += _force;
  }
  else
    gzerr << "Something's wrong, joint [" << this->Name()
          << "] index [" << _index
          << "] out of range.\n";
}

//////////////////////////////////////////////////
void SimbodyJoint::SaveSimbodyState(const SimTK::State &/*_state*/)
{
//  // Implementation not complete
//  // Not implemented
//  // skip if not a free joint, state is saved in SimbodyJoint::mobod
//  if (!this->masterMobod.isEmptyHandle() &&
//      SimTK::MobilizedBody::Free::isInstanceOf(this->masterMobod))
//  {
//    if (this->simbodyQ.empty())
//      this->simbodyQ.resize(this->masterMobod.getNumQ(_state));
//
//    if (this->simbodyU.empty())
//      this->simbodyU.resize(this->masterMobod.getNumU(_state));
//
//    for (unsigned int i = 0; i < this->simbodyQ.size(); ++i)
//      this->simbodyQ[i] = this->masterMobod.getOneQ(_state, i);
//
//    for (unsigned int i = 0; i < this->simbodyU.size(); ++i)
//      this->simbodyU[i] = this->masterMobod.getOneU(_state, i);
//  }
//  else
//  {
//    // gzerr << "debug: joint name: " << this->ScopedName() << "\n";
//  }
}

//////////////////////////////////////////////////
void SimbodyJoint::RestoreSimbodyState(SimTK::State &/*_state*/)
{
  // Not implemented
}

//////////////////////////////////////////////////
void SimbodyJoint::SetAnchor(const unsigned int /*_index*/,
    const ignition::math::Vector3d &/*_anchor*/)
{
  gzerr << "SimbodyJoint::SetAnchor:  Not implement in Simbody."
        << " Anchor is set during joint construction in SimbodyPhysics.cc\n";
}

//////////////////////////////////////////////////
void SimbodyJoint::SetDamping(const unsigned int _index, const double _damping)
{
  if (_index < this->AngleCount())
  {
    this->SetStiffnessDamping(_index,
        this->simbodyJointDPtr->stiffnessCoefficient[_index], _damping);
  }
  else
  {
     gzerr << "SimbodyJoint::SetDamping: index[" << _index
           << "] is out of bounds (AngleCount() = "
           << this->AngleCount() << ").\n";
     return;
  }
}

//////////////////////////////////////////////////
void SimbodyJoint::SetStiffness(const unsigned int _index,
    const double _stiffness)
{
  if (_index < this->AngleCount())
  {
    this->SetStiffnessDamping(_index, _stiffness,
      this->simbodyJointDPtr->dissipationCoefficient[_index]);
  }
  else
  {
     gzerr << "SimbodyJoint::SetStiffness: index[" << _index
           << "] is out of bounds (AngleCount() = "
           << this->AngleCount() << ").\n";
     return;
  }
}

//////////////////////////////////////////////////
void SimbodyJoint::SetStiffnessDamping(const unsigned int _index,
  const double _stiffness, const double _damping, const double _reference)
{
  if (_index < this->AngleCount())
  {
    this->simbodyJointDPtr->stiffnessCoefficient[_index] = _stiffness;
    this->simbodyJointDPtr->dissipationCoefficient[_index] = _damping;
    this->simbodyJointDPtr->springReferencePosition[_index] = _reference;

    if (this->simbodyJointDPtr->physicsInitialized)
    {
      // set damper coefficient
      this->simbodyJointDPtr->damper[_index].setDamping(
        this->simbodyJointDPtr->simbodyPhysics->Integ()->updAdvancedState(),
        _damping);

      // set spring stiffness and reference position
      this->simbodyJointDPtr->spring[_index].setStiffness(
        this->simbodyJointDPtr->simbodyPhysics->Integ()->updAdvancedState(),
        _stiffness);
      this->simbodyJointDPtr->spring[_index].setQZero(
        this->simbodyJointDPtr->simbodyPhysics->Integ()->updAdvancedState(),
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
    const unsigned int /*_index*/, const boost::any &/*_value*/)
{
  gzerr << "Not implement in Simbody\n";
  return false;
}

//////////////////////////////////////////////////
double SimbodyJoint::Param(const std::string &_key,
    const unsigned int _index) const
{
  return Joint::Param(_key, _index);
}

//////////////////////////////////////////////////
bool SimbodyJoint::SetHighStop(const unsigned int _index,
    const ignition::math::Angle &_angle)
{
  Joint::SetHighStop(_index, _angle);

  if (_index < this->AngleCount())
  {
    if (this->simbodyJointDPtr->physicsInitialized)
    {
      if (!this->simbodyJointDPtr->limitForce[_index].isEmptyHandle())
      {
        this->simbodyJointDPtr->limitForce[_index].setBounds(
          this->simbodyJointDPtr->simbodyPhysics->Integ()->updAdvancedState(),
          this->LowStop(_index).Radian(), _angle.Radian());
      }
      else
      {
        gzerr << "child link is null, force element not initialized, "
              << "SetHighStop failed. Please file a report on issue tracker.\n";
        return false;
      }
    }
    else
    {
      gzerr << "SetHighStop: State not initialized, SetHighStop failed.\n";
      return false;
    }
  }
  else
  {
    gzerr << "SetHighStop: index out of bounds.\n";
    return false;
  }
  return true;
}

//////////////////////////////////////////////////
bool SimbodyJoint::SetLowStop(
    const unsigned int _index, const ignition::math::Angle &_angle)
{
  Joint::SetLowStop(_index, _angle);

  if (_index < this->AngleCount())
  {
    if (this->simbodyJointDPtr->physicsInitialized)
    {
      if (!this->simbodyJointDPtr->limitForce[_index].isEmptyHandle())
      {
        this->simbodyJointDPtr->limitForce[_index].setBounds(
          this->simbodyJointDPtr->simbodyPhysics->Integ()->updAdvancedState(),
          _angle.Radian(),
          this->HighStop(_index).Radian());
      }
      else
      {
        gzerr << "child link is null, force element not initialized, "
              << "SetLowStop failed. Please file a report on issue tracker.\n";
        return false;
      }
    }
    else
    {
      gzerr << "SetLowStop: State not initialized, SetLowStop failed.\n";
      return false;
    }
  }
  else
  {
    gzerr << "SetLowStop: index out of bounds.\n";
    return false;
  }
  return true;
}

//////////////////////////////////////////////////
ignition::math::Angle SimbodyJoint::HighStop(const unsigned int _index) const
{
  if (_index >= this->AngleCount())
  {
    gzerr << "Invalid joint index [" << _index
          << "] when trying to get high stop\n";
    /// \TODO: should return NaN
    return ignition::math::Angle(0.0);
  }
  else if (_index == 0)
  {
    return ignition::math::Angle(
        this->simbodyJointDPtr->sdf->GetElement("axis")->GetElement(
          "limit")->Get<double>("upper"));
  }
  else if (_index == 1)
  {
    return ignition::math::Angle(
        this->simbodyJointDPtr->sdf->GetElement("axis2")->GetElement(
          "limit")->Get<double>("upper"));
  }
  else
  {
    gzerr << "Should not be here in code, AngleCount > 2?\n";
    /// \TODO: should return NaN
    return ignition::math::Angle(0.0);
  }
}

//////////////////////////////////////////////////
ignition::math::Angle SimbodyJoint::LowStop(const unsigned int _index) const
{
  if (_index >= this->AngleCount())
  {
    gzerr << "Invalid joint index [" << _index
          << "] when trying to get low stop\n";
    /// \TODO: should return NaN
    return ignition::math::Angle(0.0);
  }
  else if (_index == 0)
  {
    return ignition::math::Angle(
        this->simbodyJointDPtr->sdf->GetElement("axis")->GetElement(
          "limit")->Get<double>("lower"));
  }
  else if (_index == 1)
  {
    return ignition::math::Angle(
        this->simbodyJointDPtr->sdf->GetElement("axis2")->GetElement(
          "limit")->Get<double>("lower"));
  }
  else
  {
    gzerr << "Should not be here in code, AngleCount > 2?\n";
    /// \TODO: should return NaN
    return ignition::math::Angle::Zero;
  }
}

/////////////////////////////////////////////////
void SimbodyJoint::SetPhysicsInitialized(const bool _value)
{
  this->simbodyJointDPtr->physicsInitialized = _value;
}

/////////////////////////////////////////////////
bool SimbodyJoint::MustBreakLoopHere() const
{
  return this->simbodyJointDPtr->mustBreakLoopHere;
}

/////////////////////////////////////////////////
SimTK::Transform SimbodyJoint::XPA() const
{
  return this->simbodyJointDPtr->xPA;
}

/////////////////////////////////////////////////
SimTK::Transform SimbodyJoint::XCB() const
{
  return this->simbodyJointDPtr->xCB;
}

/////////////////////////////////////////////////
SimTK::Transform SimbodyJoint::DefxAB() const
{
  return this->simbodyJointDPtr->defxAB;
}

/////////////////////////////////////////////////
SimTK::Force::MobilityLinearStop SimbodyJoint::LimitForce(
    unsigned int _index) const
{
  if (_index >= MAX_JOINT_AXIS)
  {
    _index = 0;
    gzerr << "Axis index with value[" << _index << "] must be between zero and "
      << MAX_JOINT_AXIS << ". Using an index of zero.\n";
  }

  return this->simbodyJointDPtr->limitForce[_index];
}

/////////////////////////////////////////////////
void SimbodyJoint::SetLimitForce(const unsigned int _index,
    const SimTK::Force::MobilityLinearStop _limit)
{
  if (_index >= MAX_JOINT_AXIS)
  {
    gzerr << "Axis index with value[" << _index << "] must be between zero and "
      << MAX_JOINT_AXIS << ".\n";
    return;
  }

  this->simbodyJointDPtr->limitForce[_index] = _limit;
}

/////////////////////////////////////////////////
SimTK::Force::MobilityLinearDamper SimbodyJoint::Damper(
    unsigned int _index) const
{
  if (_index >= MAX_JOINT_AXIS)
  {
    _index = 0;
    gzerr << "Axis index with value[" << _index << "] must be between zero and "
      << MAX_JOINT_AXIS << ". Using an index of zero.\n";
  }

  return this->simbodyJointDPtr->damper[_index];
}

/////////////////////////////////////////////////
void SimbodyJoint::SetDamper(const unsigned int _index,
    const SimTK::Force::MobilityLinearDamper _damper)
{
  if (_index >= MAX_JOINT_AXIS)
  {
    gzerr << "Axis index with value[" << _index << "] must be between zero and "
      << MAX_JOINT_AXIS << ".\n";
    return;
  }

  this->simbodyJointDPtr->damper[_index] = _damper;
}

/////////////////////////////////////////////////
SimTK::Force::MobilityLinearSpring SimbodyJoint::Spring(
    unsigned int _index) const
{
  if (_index >= MAX_JOINT_AXIS)
  {
    _index = 0;
    gzerr << "Axis index with value[" << _index << "] must be between zero and "
      << MAX_JOINT_AXIS << ". Using an index of zero.\n";
  }

  return this->simbodyJointDPtr->spring[_index];
}

/////////////////////////////////////////////////
void SimbodyJoint::SetSpring(const unsigned int _index,
    const SimTK::Force::MobilityLinearSpring _spring)
{
  if (_index >= MAX_JOINT_AXIS)
  {
    gzerr << "Axis index with value[" << _index << "] must be between zero and "
      << MAX_JOINT_AXIS << ".\n";
    return;
  }

  this->simbodyJointDPtr->spring[_index] = _spring;
}

/////////////////////////////////////////////////
void SimbodyJoint::SetMobod(const SimTK::MobilizedBody &_mobod)
{
  this->simbodyJointDPtr->mobod = _mobod;
}

/////////////////////////////////////////////////
bool SimbodyJoint::IsReversed() const
{
  return this->simbodyJointDPtr->isReversed;
}

/////////////////////////////////////////////////
void SimbodyJoint::SetIsReversed(const bool _value)
{
  this->simbodyJointDPtr->isReversed = _value;
}
