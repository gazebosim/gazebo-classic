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
#include "gazebo/common/Console.hh"

#include "gazebo/physics/Model.hh"
#include "gazebo/physics/simbody/SimbodyJointPrivate.hh"
#include "gazebo/physics/simbody/SimbodyLink.hh"
#include "gazebo/physics/simbody/SimbodyTypes.hh"
#include "gazebo/physics/simbody/SimbodyUniversalJoint.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
SimbodyUniversalJoint::SimbodyUniversalJoint(SimTK::MultibodySystem* /*_world*/,
  BasePtr _parent) : UniversalJoint<SimbodyJoint>(_parent)
{
  this->simbodyJointDPtr->physicsInitialized = false;
}

//////////////////////////////////////////////////
SimbodyUniversalJoint::~SimbodyUniversalJoint()
{
}

//////////////////////////////////////////////////
void SimbodyUniversalJoint::Load(sdf::ElementPtr _sdf)
{
  UniversalJoint<SimbodyJoint>::Load(_sdf);
}

//////////////////////////////////////////////////
ignition::math::Vector3d SimbodyUniversalJoint::Anchor(
    const unsigned int /*_index*/) const
{
  return this->simbodyJointDPtr->anchorPos;
}

//////////////////////////////////////////////////
ignition::math::Vector3d SimbodyUniversalJoint::Axis(
    const unsigned int /*_index*/) const
{
  return ignition::math::Vector3d::Zero;
}

//////////////////////////////////////////////////
void SimbodyUniversalJoint::SetAxis(const unsigned int /*_index*/,
    const ignition::math::Vector3d &/*_axis*/)
{
  /// Universal Joint are built in SimbodyPhysics.cc, so this init block
  /// actually does nothing.
  gzdbg << "SetAxis: setting axis is not yet implemented. The axes are set "
        << "during joint construction in SimbodyPhyiscs.cc for now.\n";
}

//////////////////////////////////////////////////
double SimbodyUniversalJoint::Velocity(unsigned int _index) const
{
  if (_index < this->AngleCount())
  {
    if (this->simbodyJointDPtr->physicsInitialized &&
        this->simbodyJointDPtr->simbodyPhysics->PhysicsInitialized())
    {
      return this->simbodyJointDPtr->mobod.getOneU(
        this->simbodyJointDPtr->simbodyPhysics->Integ()->getState(),
        SimTK::MobilizerUIndex(_index));
    }
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
}

//////////////////////////////////////////////////
void SimbodyUniversalJoint::SetVelocity(const unsigned int _index,
    const double _rate)
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
  {
    gzerr << "SetVelocity _index too large.\n";
  }
}

//////////////////////////////////////////////////
void SimbodyUniversalJoint::SetForceImpl(const unsigned int _index,
    const double _torque)
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
ignition::math::Vector3d SimbodyUniversalJoint::GlobalAxis(
    const unsigned int _index) const
{
  if (this->simbodyJointDPtr->simbodyPhysics &&
      this->simbodyJointDPtr->simbodyPhysics->PhysicsStepped() &&
      _index < this->AngleCount())
  {
    if (!this->simbodyJointDPtr->mobod.isEmptyHandle())
    {
      if (_index == UniversalJoint::AXIS_PARENT)
      {
        // express X-axis of X_IF in world frame
        const SimTK::Transform &X_IF =
          this->simbodyJointDPtr->mobod.getInboardFrame(
              this->simbodyJointDPtr->simbodyPhysics->Integ()->getState());

        SimTK::Vec3 x_W(
            this->simbodyJointDPtr->mobod.getParentMobilizedBody().
            expressVectorInGroundFrame(
              this->simbodyJointDPtr->simbodyPhysics->Integ()->getState(),
              X_IF.x()));

        return SimbodyPhysics::Vec3ToVector3Ign(x_W);
      }
      else if (_index == UniversalJoint::AXIS_CHILD)
      {
        // express Y-axis of xom in world frame
        const SimTK::Transform &xom =
          this->simbodyJointDPtr->mobod.getOutboardFrame(
              this->simbodyJointDPtr->simbodyPhysics->Integ()->getState());

        SimTK::Vec3 yw(
            this->simbodyJointDPtr->mobod.expressVectorInGroundFrame(
              this->simbodyJointDPtr->simbodyPhysics->Integ()->getState(),
              xom.y()));

        return SimbodyPhysics::Vec3ToVector3Ign(yw);
      }
      else
      {
        gzerr << "GetGlobalAxis: internal error, AngleCount < 0.\n";
        return ignition::math::Vector3d(ignition::math::NAN_D,
            ignition::math::NAN_D, ignition::math::NAN_D);
      }
    }
    else
    {
      gzerr << "Joint mobod not initialized correctly.  Returning"
        << " initial axis vector in world frame (not valid if"
        << " joint frame has moved). Please file"
        << " a report on issue tracker.\n";
      return this->AxisFrame(_index).RotateVector(this->LocalAxis(_index));
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
      gzdbg << "GetGlobalAxis() sibmody physics engine not yet initialized, "
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
ignition::math::Angle SimbodyUniversalJoint::AngleImpl(
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
        return ignition::math::Angle::Zero;
      }
    }
    else
    {
      gzdbg << "GetAngleImpl(): simbody not yet initialized, "
            << "initial angle should be zero until <initial_angle> "
            << "is implemented.\n";
      return ignition::math::Angle::Zero;
    }
  }
  else
  {
    gzerr << "index out of bound\n";
  }
  return ignition::math::Angle(ignition::math::NAN_D);
}
