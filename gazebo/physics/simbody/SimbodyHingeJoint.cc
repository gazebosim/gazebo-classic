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

#include "gazebo/physics/Model.hh"
#include "gazebo/physics/simbody/SimbodyLink.hh"
#include "gazebo/physics/simbody/SimbodyPhysics.hh"
#include "gazebo/physics/simbody/SimbodyJointPrivate.hh"
#include "gazebo/physics/simbody/SimbodyHingeJoint.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
SimbodyHingeJoint::SimbodyHingeJoint(SimTK::MultibodySystem * /*_world*/,
                                     BasePtr _parent)
: HingeJoint<SimbodyJoint>(_parent)
{
  this->simbodyJointDPtr->physicsInitialized = false;
}

//////////////////////////////////////////////////
SimbodyHingeJoint::~SimbodyHingeJoint()
{
}

//////////////////////////////////////////////////
void SimbodyHingeJoint::Load(sdf::ElementPtr _sdf)
{
  HingeJoint<SimbodyJoint>::Load(_sdf);
}

//////////////////////////////////////////////////
void SimbodyHingeJoint::SetAxis(unsigned int /*_index*/,
    const ignition::math::Vector3d &/*_axis*/)
{
  // Simbody seems to handle setAxis improperly. It readjust all the pivot
  // points
  gzdbg << "SetAxis: setting axis is not yet implemented.  The axis are set "
        << " during joint construction in SimbodyPhysics.cc for now.\n";
}

//////////////////////////////////////////////////
void SimbodyHingeJoint::SetVelocity(
    const unsigned int _index, const double _rate)
{
  if (_index < this->AngleCount())
  {
    this->simbodyJointDPtr->mobod.setOneU(
      this->simbodyJointDPtr->simbodyPhysics->Integ()->updAdvancedState(),
      SimTK::MobilizerUIndex(_index), _rate);

    this->simbodyJointDPtr->simbodyPhysics->System().realize(
      this->simbodyJointDPtr->simbodyPhysics->Integ()->getAdvancedState(),
      SimTK::Stage::Velocity);
  }
  else
    gzerr << "SetVelocity _index too large.\n";
}

//////////////////////////////////////////////////
double SimbodyHingeJoint::Velocity(const unsigned int _index) const
{
  if (_index < this->AngleCount())
  {
    if (this->simbodyJointDPtr->physicsInitialized &&
        this->simbodyJointDPtr->simbodyPhysics->PhysicsInitialized())

      return this->simbodyJointDPtr->mobod.getOneU(
        this->simbodyJointDPtr->simbodyPhysics->Integ()->getState(),
        SimTK::MobilizerUIndex(_index));
    else
    {
      gzdbg << "GetVelocity() simbody not yet initialized, "
            << "initial velocity should be zero until restart from "
            << "state has been implemented.\n";
      return 0.0;
    }
  }
  else
  {
    gzerr << "Invalid index for joint, returning NaN\n";
    return ignition::math::NAN_D;
  }
  return ignition::math::NAN_D;
}

//////////////////////////////////////////////////
void SimbodyHingeJoint::SetForceImpl(
    const unsigned int _index, const double _torque)
{
  if (_index < this->AngleCount() && this->simbodyJointDPtr->physicsInitialized)
  {
    this->simbodyJointDPtr->simbodyPhysics->DiscreteForces().
      setOneMobilityForce(
          this->simbodyJointDPtr->simbodyPhysics->Integ()->updAdvancedState(),
          this->simbodyJointDPtr->mobod,
          SimTK::MobilizerUIndex(_index), _torque);
  }
}

//////////////////////////////////////////////////
ignition::math::Vector3d SimbodyHingeJoint::GlobalAxis(
    const unsigned int _index) const
{
  if (this->simbodyJointDPtr->simbodyPhysics &&
      this->simbodyJointDPtr->simbodyPhysics->PhysicsStepped() &&
      _index < this->AngleCount())
  {
    if (!this->simbodyJointDPtr->mobod.isEmptyHandle())
    {
      const SimTK::Transform &xom =
        this->simbodyJointDPtr->mobod.getOutboardFrame(
        this->simbodyJointDPtr->simbodyPhysics->Integ()->getState());

      // express Z-axis of xom in world frame
      SimTK::Vec3 zw(this->simbodyJointDPtr->mobod.expressVectorInGroundFrame(
        this->simbodyJointDPtr->simbodyPhysics->Integ()->getState(), xom.z()));

      return SimbodyPhysics::Vec3ToVector3Ign(zw);
    }
    else
    {
      gzerr << "Joint mobod not initialized correctly.  Returning"
            << " initial axis vector in world frame (not valid if"
            << " joint frame has moved). Please file"
            << " a report on issue tracker.\n";
      return this->AxisFrame(_index).RotateVector(
        this->LocalAxis(_index));
    }
  }
  else
  {
    if (_index >= this->AngleCount())
    {
      gzerr << "index out of bound\n";
      return ignition::math::Vector3d(ignition::math::NAN_D,
          ignition::math::NAN_D, ignition::math::NAN_D);
    }
    else
    {
      gzdbg << "GetGlobalAxis() sibmody physics engine not initialized yet, "
            << "use local axis and initial pose to compute "
            << "global axis.\n";
      // if local axis specified in model frame (to be changed)
      // switch to code below if issue #494 is to be addressed
      return this->AxisFrame(_index).RotateVector(this->LocalAxis(_index));
    }
  }

  return ignition::math::Vector3d(ignition::math::NAN_D,
      ignition::math::NAN_D, ignition::math::NAN_D);
}

//////////////////////////////////////////////////
ignition::math::Angle SimbodyHingeJoint::AngleImpl(
    const unsigned int _index) const
{
  if (_index < this->AngleCount())
  {
    if (this->simbodyJointDPtr->physicsInitialized &&
        this->simbodyJointDPtr->simbodyPhysics->PhysicsInitialized())
    {
      if (!this->simbodyJointDPtr->mobod.isEmptyHandle())
      {
        return ignition::math::Angle(this->simbodyJointDPtr->mobod.getOneQ(
          this->simbodyJointDPtr->simbodyPhysics->Integ()->getState(), _index));
      }
      else
      {
        gzerr << "Joint mobod not initialized correctly.  Please file"
              << " a report on issue tracker.\n";
        return ignition::math::Angle(0.0);
      }
    }
    else
    {
      gzdbg << "GetAngleImpl() simbody not yet initialized, "
            << "initial angle should be zero until <initial_angle> "
            << "is implemented.\n";
      return ignition::math::Angle::Zero;
    }
  }
  else
  {
    gzerr << "index out of bound\n";
    return ignition::math::Angle(ignition::math::NAN_D);
  }
  return ignition::math::Angle(0.0);
}

//////////////////////////////////////////////////
void SimbodyHingeJoint::SaveSimbodyState(const SimTK::State &_state)
{
  if (!this->simbodyJointDPtr->mobod.isEmptyHandle())
  {
    if (this->simbodyQ.empty())
      this->simbodyQ.resize(this->simbodyJointDPtr->mobod.getNumQ(_state));

    if (this->simbodyU.empty())
      this->simbodyU.resize(this->simbodyJointDPtr->mobod.getNumU(_state));

    for (unsigned int i = 0; i < this->simbodyQ.size(); ++i)
      this->simbodyQ[i] = this->simbodyJointDPtr->mobod.getOneQ(_state, i);

    for (unsigned int i = 0; i < this->simbodyU.size(); ++i)
      this->simbodyU[i] = this->simbodyJointDPtr->mobod.getOneU(_state, i);
  }
  else
  {
    // gzerr << "debug: joint name: " << this->GetScopedName() << "\n";
  }
}

//////////////////////////////////////////////////
void SimbodyHingeJoint::RestoreSimbodyState(SimTK::State &_state)
{
  if (!this->simbodyJointDPtr->mobod.isEmptyHandle())
  {
    for (unsigned int i = 0; i < this->simbodyQ.size(); ++i)
      this->simbodyJointDPtr->mobod.setOneQ(_state, i, this->simbodyQ[i]);

    for (unsigned int i = 0; i < this->simbodyU.size(); ++i)
      this->simbodyJointDPtr->mobod.setOneU(_state, i, this->simbodyU[i]);
  }
  else
  {
    // gzerr << "restoring model [" << this->GetScopedName()
    //       << "] failed due to uninitialized mobod\n";
  }
}
