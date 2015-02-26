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
    this->model->GetWorld()->GetPhysicsEngine());

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

  math::Pose childPose = _sdf->Get<math::Pose>("pose");
  if (_sdf->GetElement("child")->HasElement("pose"))
    childPose = _sdf->GetElement("child")->Get<math::Pose>("pose");

  this->xCB = physics::SimbodyPhysics::Pose2Transform(childPose);

  math::Pose parentPose;
  if (_sdf->GetElement("parent")->HasElement("pose"))
    this->xPA = physics::SimbodyPhysics::GetPose(_sdf->GetElement("parent"));
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
    -SimbodyPhysics::Vec3ToVector3(reactionForceOnParentBody);
  this->wrench.body1Torque =
    -SimbodyPhysics::Vec3ToVector3(reactionTorqueOnParentBody);

  this->wrench.body2Force =
    -SimbodyPhysics::Vec3ToVector3(reactionForceOnChildBody);
  this->wrench.body2Torque =
    -SimbodyPhysics::Vec3ToVector3(reactionTorqueOnChildBody);
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
void SimbodyJoint::SetAxis(unsigned int _index, const math::Vector3 &/*_axis*/)
{
  math::Pose parentModelPose;
  if (this->parentLink)
    parentModelPose = this->parentLink->GetModel()->GetWorldPose();

  // Set joint axis
  // assuming incoming axis is defined in the model frame, so rotate them
  // into the inertial frame
  // TODO: switch so the incoming axis is defined in the child frame.
  math::Vector3 axis = parentModelPose.rot.RotateVector(
    this->sdf->GetElement("axis")->Get<math::Vector3>("xyz"));

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
  if (_index < this->GetAngleCount())
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
  if (_index < this->GetAngleCount())
  {
    if (this->forceAppliedTime < this->GetWorld()->GetSimTime())
    {
      // reset forces if time step is new
      this->forceAppliedTime = this->GetWorld()->GetSimTime();
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
void SimbodyJoint::SetAnchor(unsigned int /*_index*/,
    const gazebo::math::Vector3 & /*_anchor*/)
{
  gzerr << "SimbodyJoint::SetAnchor:  Not implement in Simbody."
        << " Anchor is set during joint construction in SimbodyPhysics.cc\n";
}

//////////////////////////////////////////////////
void SimbodyJoint::SetDamping(unsigned int _index, const double _damping)
{
  if (_index < this->GetAngleCount())
  {
    this->SetStiffnessDamping(_index, this->stiffnessCoefficient[_index],
      _damping);
  }
  else
  {
     gzerr << "SimbodyJoint::SetDamping: index[" << _index
           << "] is out of bounds (GetAngleCount() = "
           << this->GetAngleCount() << ").\n";
     return;
  }
}

//////////////////////////////////////////////////
void SimbodyJoint::SetStiffness(unsigned int _index, const double _stiffness)
{
  if (_index < this->GetAngleCount())
  {
    this->SetStiffnessDamping(_index, _stiffness,
      this->dissipationCoefficient[_index]);
  }
  else
  {
     gzerr << "SimbodyJoint::SetStiffness: index[" << _index
           << "] is out of bounds (GetAngleCount() = "
           << this->GetAngleCount() << ").\n";
     return;
  }
}

//////////////////////////////////////////////////
void SimbodyJoint::SetStiffnessDamping(unsigned int _index,
  double _stiffness, double _damping, double _reference)
{
  if (_index < this->GetAngleCount())
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
math::Vector3 SimbodyJoint::GetAnchor(unsigned int /*_index*/) const
{
  gzerr << "Not implement in Simbody\n";
  return math::Vector3();
}

//////////////////////////////////////////////////
math::Vector3 SimbodyJoint::GetLinkForce(unsigned int /*_index*/) const
{
  gzerr << "Not implement in Simbody\n";
  return math::Vector3();
}

//////////////////////////////////////////////////
math::Vector3 SimbodyJoint::GetLinkTorque(unsigned int /*_index*/) const
{
  gzerr << "Not implement in Simbody\n";
  return math::Vector3();
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
bool SimbodyJoint::SetHighStop(unsigned int _index, const math::Angle &_angle)
{
  Joint::SetHighStop(_index, _angle);

  if (_index < this->GetAngleCount())
  {
    if (this->physicsInitialized)
    {
      if (!this->limitForce[_index].isEmptyHandle())
      {
        this->limitForce[_index].setBounds(
          this->simbodyPhysics->integ->updAdvancedState(),
          this->GetLowStop(_index).Radian(), _angle.Radian());
      }
      else
      {
        gzerr << "child link is NULL, force element not initialized, "
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
bool SimbodyJoint::SetLowStop(unsigned int _index, const math::Angle &_angle)
{
  Joint::SetLowStop(_index, _angle);

  if (_index < this->GetAngleCount())
  {
    if (this->physicsInitialized)
    {
      if (!this->limitForce[_index].isEmptyHandle())
      {
        this->limitForce[_index].setBounds(
          this->simbodyPhysics->integ->updAdvancedState(),
          _angle.Radian(),
          this->GetHighStop(_index).Radian());
      }
      else
      {
        gzerr << "child link is NULL, force element not initialized, "
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
math::Angle SimbodyJoint::GetHighStop(unsigned int _index)
{
  if (_index >= this->GetAngleCount())
  {
    gzerr << "Invalid joint index [" << _index
          << "] when trying to get high stop\n";
    /// \TODO: should return NaN
    return math::Angle(0.0);
  }
  else if (_index == 0)
  {
    return math::Angle(this->sdf->GetElement("axis")->GetElement("limit")
             ->Get<double>("upper"));
  }
  else if (_index == 1)
  {
    return math::Angle(this->sdf->GetElement("axis2")->GetElement("limit")
             ->Get<double>("upper"));
  }
  else
  {
    gzerr << "Should not be here in code, GetAngleCount > 2?\n";
    /// \TODO: should return NaN
    return math::Angle(0.0);
  }
}

//////////////////////////////////////////////////
math::Angle SimbodyJoint::GetLowStop(unsigned int _index)
{
  if (_index >= this->GetAngleCount())
  {
    gzerr << "Invalid joint index [" << _index
          << "] when trying to get low stop\n";
    /// \TODO: should return NaN
    return math::Angle(0.0);
  }
  else if (_index == 0)
  {
    return math::Angle(this->sdf->GetElement("axis")->GetElement("limit")
             ->Get<double>("lower"));
  }
  else if (_index == 1)
  {
    return math::Angle(this->sdf->GetElement("axis2")->GetElement("limit")
             ->Get<double>("lower"));
  }
  else
  {
    gzerr << "Should not be here in code, GetAngleCount > 2?\n";
    /// \TODO: should return NaN
    return math::Angle(0.0);
  }
}
