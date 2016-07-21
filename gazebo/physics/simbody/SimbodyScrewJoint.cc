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

#include "gazebo/common/Console.hh"
#include "gazebo/common/Exception.hh"

#include "gazebo/physics/Model.hh"
#include "gazebo/physics/simbody/SimbodyJointPrivate.hh"
#include "gazebo/physics/simbody/SimbodyLink.hh"
#include "gazebo/physics/simbody/SimbodyPhysics.hh"
#include "gazebo/physics/simbody/SimbodyTypes.hh"
#include "gazebo/physics/simbody/SimbodyScrewJoint.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
SimbodyScrewJoint::SimbodyScrewJoint(SimTK::MultibodySystem * /*_world*/,
                                     BasePtr _parent)
: ScrewJoint<SimbodyJoint>(_parent)
{
  this->simbodyJointDPtr->physicsInitialized = false;
}

//////////////////////////////////////////////////
SimbodyScrewJoint::~SimbodyScrewJoint()
{
}

//////////////////////////////////////////////////
void SimbodyScrewJoint::Load(sdf::ElementPtr _sdf)
{
  ScrewJoint<SimbodyJoint>::Load(_sdf);
}

//////////////////////////////////////////////////
double SimbodyScrewJoint::Velocity(unsigned int _index) const
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
      gzdbg << "SimbodyScrewJoint::Velocity() simbody not yet initialized, "
        << "initial velocity should be zero until restart from "
        << "state has been implemented.\n";
      return 0.0;
    }
  }
  else
  {
    gzerr << "SimbodyScrewJoint::Invalid index for joint, returning NaN\n";
    return ignition::math::NAN_D;
  }
}

//////////////////////////////////////////////////
void SimbodyScrewJoint::SetVelocity(const unsigned int _index,
    const double _rate)
{
  if (_index < this->AngleCount())
  {
    this->simbodyJointDPtr->mobod.setOneU(
      this->simbodyJointDPtr->simbodyPhysics->Integ()->updAdvancedState(),
      SimTK::MobilizerUIndex(_index), _rate);
  }
  else
  {
    gzerr << "SimbodyScrewJoint::SetVelocity _index too large.\n";
  }
}

//////////////////////////////////////////////////
void SimbodyScrewJoint::SetAxis(const unsigned int /*_index*/,
    const ignition::math::Vector3d &/*_axis*/)
{
  // Simbody seems to handle setAxis improperly. It readjust all the pivot
  // points
  gzdbg << "SimbodyScrewJoint::SetAxis: setting axis is "
        << "not yet implemented.  The axis are set during joint construction "
        << "in SimbodyPhysics.cc for now.\n";
}

//////////////////////////////////////////////////
void SimbodyScrewJoint::SetThreadPitch(const unsigned int /*_index*/,
    const double _threadPitch)
{
  gzdbg << "SimbodyScrewJoint::SetThreadPitch: setting thread pitch is "
        << "not yet tested.  The pitch are set during joint construction "
        << "in SimbodyPhysics.cc for now.\n";

  this->SetThreadPitch(_threadPitch);
}

//////////////////////////////////////////////////
void SimbodyScrewJoint::SetThreadPitch(const double /*_threadPitch*/)
{
  gzerr << "SimbodyScrewJoint::SetThreadPitch: setting thread pitch is "
        << "not yet tested.  The pitch are set during joint construction "
        << "in SimbodyPhysics.cc for now.\n";

  // need to figure out how to down cast correctly
  // if (this->simbodyJointDPtr->physicsInitialized &&
  //     this->simbodyJointDPtr->simbodyPhysics->simbodyPhysicsInitialized)
  // {
  //   // downcast mobod to screw mobod first
  //   // the way pitch is defined in simbody is -1.0 / gazebo thread pitch
  //   SimTK::MobilizedBody::Screw screw =
  //     static_cast<SimTK::MobilizedBody::Screw>(
  //     this->simbodyJointDPtr->mobod);
  //   screw.setDefaultPitch(-1.0/_threadPitch);
  // }
  // else
  // {
  //   gzwarn << "SimbodyScrewJoint physics not initialized yet, or failed"
  //          << " to initialize. Returning thread pitch from SDF.\n"
  //   return this->threadPitch;
  // }
}

//////////////////////////////////////////////////
void SimbodyScrewJoint::SetForceImpl(const unsigned int _index,
    const double _torque)
{
  if (_index < this->AngleCount() &&
      this->simbodyJointDPtr->physicsInitialized)
  {
    this->simbodyJointDPtr->simbodyPhysics->DiscreteForces(
        ).setOneMobilityForce(
          this->simbodyJointDPtr->simbodyPhysics->Integ()->updAdvancedState(),
          this->simbodyJointDPtr->mobod,
          SimTK::MobilizerUIndex(_index), _torque);
  }
}

//////////////////////////////////////////////////
ignition::math::Vector3d SimbodyScrewJoint::GlobalAxis(
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

      // express Z-axis of X_OM in world frame
      SimTK::Vec3 zw(this->simbodyJointDPtr->mobod.expressVectorInGroundFrame(
            this->simbodyJointDPtr->simbodyPhysics->Integ()->getState(),
            xom.z()));

      return SimbodyPhysics::Vec3ToVector3Ign(zw);
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
      gzdbg << "SimbodyScrewJoint::GlobalAxis() sibmody physics"
        << " engine not initialized yet, "
        << "use local axis and initial pose to compute "
        << "global axis.\n";

      // if local axis specified in model frame (to be changed)
      // switch to code below if issue #494 is to be addressed
      return this->AxisFrame(_index).RotateVector(this->LocalAxis(_index));
    }
  }
}

//////////////////////////////////////////////////
ignition::math::Angle SimbodyScrewJoint::AngleImpl(
    const unsigned int _index) const
{
  if (_index < this->AngleCount())
  {
    if (this->simbodyJointDPtr->physicsInitialized &&
        this->simbodyJointDPtr->simbodyPhysics->PhysicsInitialized())
    {
      if (!this->simbodyJointDPtr->mobod.isEmptyHandle())
      {
        // simbody screw joint only has one dof
        // _index=0: angular dof
        // _index=1: linear dof
        ignition::math::Angle angle(this->simbodyJointDPtr->mobod.getOneQ(
          this->simbodyJointDPtr->simbodyPhysics->Integ()->getState(), 0));

        if (_index == 1)
        {
          // return linear position
          // thread pitch units rad/m
          angle /= ignition::math::Angle(this->threadPitch);
        }
        return angle;
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
      gzdbg << "SimbodyScrewJoint::AngleImpl() simbody not yet initialized, "
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
}

//////////////////////////////////////////////////
double SimbodyScrewJoint::ThreadPitch(const unsigned int /*_index*/) const
{
  return this->ThreadPitch();
}

//////////////////////////////////////////////////
double SimbodyScrewJoint::ThreadPitch() const
{
  if (!this->simbodyJointDPtr->mobod.isEmptyHandle() &&
      this->simbodyJointDPtr->physicsInitialized &&
      this->simbodyJointDPtr->simbodyPhysics->PhysicsInitialized())
  {
    // downcast mobod to screw mobod first
    // the way pitch is defined in simbody is -1.0 / gazebo thread pitch
    return -1.0/SimTK::MobilizedBody::Screw::downcast(
          this->simbodyJointDPtr->mobod).getDefaultPitch();
  }
  else
  {
    gzwarn << "SimbodyScrewJoint physics not initialized yet, or failed"
           << " to initialize. Returning thread pitch from SDF.\n";

    return this->threadPitch;
  }
}

//////////////////////////////////////////////////
bool SimbodyScrewJoint::SetParam(const std::string &_key,
  const unsigned int _index,
  const boost::any &_value)
{
  if (_key  == "thread_pitch")
  {
    try
    {
      this->threadPitch = boost::any_cast<double>(_value);
    }
    catch(const boost::bad_any_cast &e)
    {
      gzerr << "boost any_cast error:" << e.what() << "\n";
      return false;
    }
  }
  else
    return SimbodyJoint::SetParam(_key, _index, _value);

  return true;
}

//////////////////////////////////////////////////
double SimbodyScrewJoint::Param(const std::string &_key,
  const unsigned int _index)
{
  if (_key  == "thread_pitch")
    return this->threadPitch;
  else
    return SimbodyJoint::Param(_key, _index);
}

//////////////////////////////////////////////////
bool SimbodyScrewJoint::SetHighStop(
  const unsigned int _index, const ignition::math::Angle &_angle)
{
  Joint::SetHighStop(_index, _angle);

  if (_index < this->AngleCount())
  {
    if (this->simbodyJointDPtr->physicsInitialized)
    {
      // check if limitForce is initialized
      if (this->simbodyJointDPtr->limitForce[_index].isEmptyHandle())
      {
        gzerr << "child link is null, force element not initialized, "
              << "SetHighStop failed. Please file a report on issue tracker.\n";
        return false;
      }

      if (_index == 0)
      {
        // angular limit is specified
        this->simbodyJointDPtr->limitForce[0].setBounds(
          this->simbodyJointDPtr->simbodyPhysics->Integ()->updAdvancedState(),
          this->LowStop(_index).Radian(), _angle.Radian());
      }
      else if (_index == 1)
      {
        double tp = this->ThreadPitch();
        if (ignition::math::equal(tp, 0.0))
        {
          gzerr << "thread pitch should not be zero (joint is a slider?)"
                << " using thread pitch = 1.0e6\n";
          tp = 1.0e6;
        }

        // onlye angular limiting force element is added for
        // screw joints in SimbodyPhysics.cc
        if (tp > 0)
        {
          // incoming _angle is the linear dof, which is _angle / thread_pitch.
          // convert linear limit to angular limit
          double upper = _angle.Radian() / tp;
          this->simbodyJointDPtr->limitForce[0].setBounds(
            this->simbodyJointDPtr->simbodyPhysics->Integ()->updAdvancedState(),
            this->LowStop(_index).Radian(), upper);
        }
        else
        {
          // incoming _angle is the linear dof, which is _angle / thread_pitch.
          // convert linear limit to angular limit
          // tp is negative, this is actually upper linear limit, or the
          // lower angular limit.
          double lower = _angle.Radian() / tp;
          this->simbodyJointDPtr->limitForce[0].setBounds(
            this->simbodyJointDPtr->simbodyPhysics->Integ()->updAdvancedState(),
            lower, this->HighStop(_index).Radian());
        }
      }
      else
      {
        gzerr << "Should never be here. Joint index invalid limit not set.\n";
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
bool SimbodyScrewJoint::SetLowStop(
  const unsigned int _index, const ignition::math::Angle &_angle)
{
  Joint::SetLowStop(_index, _angle);

  if (_index < this->AngleCount())
  {
    if (this->simbodyJointDPtr->physicsInitialized)
    {
      // check if limitForce is initialized
      if (this->simbodyJointDPtr->limitForce[_index].isEmptyHandle())
      {
        gzerr << "child link is null, force element not initialized, "
              << "SetHighStop failed. Please file a report on issue tracker.\n";
        return false;
      }

      if (_index == 0)
      {
        // angular limit is specified
        this->simbodyJointDPtr->limitForce[0].setBounds(
          this->simbodyJointDPtr->simbodyPhysics->Integ()->updAdvancedState(),
          _angle.Radian(),
          this->HighStop(_index).Radian());
      }
      else if (_index == 1)
      {
        double tp = this->ThreadPitch();
        if (ignition::math::equal(tp, 0.0))
        {
          gzerr << "thread pitch should not be zero (joint is a slider?)"
                << " using thread pitch = 1.0e6\n";
          tp = 1.0e6;
        }

        // onlye angular limiting force element is added for
        // screw joints in SimbodyPhysics.cc
        if (tp > 0)
        {
          // incoming _angle is the linear dof, which is _angle / thread_pitch.
          // convert linear limit to angular limit
          double lower = _angle.Radian() / tp;
          this->simbodyJointDPtr->limitForce[0].setBounds(
            this->simbodyJointDPtr->simbodyPhysics->Integ()->updAdvancedState(),
            lower, this->HighStop(_index).Radian());
        }
        else
        {
          // incoming _angle is the linear dof, which is _angle / thread_pitch.
          // convert linear limit to angular limit
          // tp is negative, this is actually lower linear limit, or the
          // upper angular limit.
          double upper = _angle.Radian() / tp;
          this->simbodyJointDPtr->limitForce[0].setBounds(
            this->simbodyJointDPtr->simbodyPhysics->Integ()->updAdvancedState(),
            this->HighStop(_index).Radian(), upper);
        }
      }
      else
      {
        gzerr << "Should never be here. Joint index invalid limit not set.\n";
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
ignition::math::Angle SimbodyScrewJoint::HighStop(const unsigned int _index)
{
  if (_index >= this->AngleCount())
  {
    gzerr << "Invalid joint index [" << _index
          << "] when trying to get high stop\n";
    /// \TODO: should return NaN
    return ignition::math::Angle::Zero;
  }
  else if (_index == 0)
  {
    return this->UpperLimit(0);
  }
  else if (_index == 1)
  {
    double tp = this->ThreadPitch();
    if (math::equal(tp, 0.0))
    {
      gzerr << "thread pitch should not be zero (joint is a slider?)"
            << " using thread pitch = 1.0e6\n";
      tp = 1.0e6;
    }
    if (tp > 0)
    {
      return this->UpperLimit(0) / tp;
    }
    else
    {
      return this->LowerLimit(0) / tp;
    }
  }
  else
  {
    gzerr << "Should not be here in code, AngleCount > 2?\n";
    /// \TODO: should return NaN
    return ignition::math::Angle::Zero;
  }
}

//////////////////////////////////////////////////
ignition::math::Angle SimbodyScrewJoint::LowStop(const unsigned int _index)
{
  if (_index >= this->AngleCount())
  {
    gzerr << "Invalid joint index [" << _index
          << "] when trying to get low stop\n";
    /// \TODO: should return NaN
    return ignition::math::Angle::Zero;
  }
  else if (_index == 0)
  {
    return this->LowerLimit(0);
  }
  else if (_index == 1)
  {
    double tp = this->ThreadPitch();
    if (ignition::math::equal(tp, 0.0))
    {
      gzerr << "thread pitch should not be zero (joint is a slider?)"
            << " using thread pitch = 1.0e6\n";
      tp = 1.0e6;
    }
    if (tp > 0)
    {
      return this->LowerLimit(0) / tp;
    }
    else
    {
      return this->UpperLimit(0) / tp;
    }
  }
  else
  {
    gzerr << "Should not be here in code, AngleCount > 2?\n";
    /// \TODO: should return NaN
    return ignition::math::Angle(0.0);
  }
}
