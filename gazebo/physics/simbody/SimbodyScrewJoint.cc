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
#include <ignition/math/Helpers.hh>

#include "gazebo/common/Console.hh"
#include "gazebo/common/Exception.hh"

#include "gazebo/physics/Model.hh"
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
  this->physicsInitialized = false;
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
double SimbodyScrewJoint::GetVelocity(unsigned int _index) const
{
  if (_index < this->DOF())
  {
    if (this->physicsInitialized &&
        this->simbodyPhysics->simbodyPhysicsInitialized)
      return this->mobod.getOneU(
        this->simbodyPhysics->integ->getState(),
        SimTK::MobilizerUIndex(_index));
    else
    {
      gzdbg << "SimbodyScrewJoint::GetVelocity() simbody not yet initialized, "
            << "initial velocity should be zero until restart from "
            << "state has been implemented.\n";
      return 0.0;
    }
  }
  else
  {
    gzerr << "SimbodyScrewJoint::Invalid index for joint, returning NaN\n";
    return SimTK::NaN;
  }
}

//////////////////////////////////////////////////
void SimbodyScrewJoint::SetVelocity(unsigned int _index, double _rate)
{
  if (_index < this->DOF())
    this->mobod.setOneU(
      this->simbodyPhysics->integ->updAdvancedState(),
      SimTK::MobilizerUIndex(_index), _rate);
  else
    gzerr << "SimbodyScrewJoint::SetVelocity _index too large.\n";
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
void SimbodyScrewJoint::SetThreadPitch(unsigned int /*_index*/,
    double _threadPitch)
{
  gzdbg << "SimbodyScrewJoint::SetThreadPitch: setting thread pitch is "
        << "not yet tested.  The pitch are set during joint construction "
        << "in SimbodyPhysics.cc for now.\n";
  this->SetThreadPitch(_threadPitch);
}

//////////////////////////////////////////////////
void SimbodyScrewJoint::SetThreadPitch(double /*_threadPitch*/)
{
  gzerr << "SimbodyScrewJoint::SetThreadPitch: setting thread pitch is "
        << "not yet tested.  The pitch are set during joint construction "
        << "in SimbodyPhysics.cc for now.\n";
  /* need to figure out how to down cast correctly
  if (this->physicsInitialized &&
      this->simbodyPhysics->simbodyPhysicsInitialized)
  {
    // downcast mobod to screw mobod first
    // the way pitch is defined in simbody is -1.0 / gazebo thread pitch
    SimTK::MobilizedBody::Screw screw =
      static_cast<SimTK::MobilizedBody::Screw>(this->mobod);
    screw.setDefaultPitch(-1.0/_threadPitch);
  }
  else
  {
    gzwarn << "SimbodyScrewJoint physics not initialized yet, or failed"
           << " to initialize. Returning thread pitch from SDF.\n"
    return this->threadPitch;
  }
  */
}

//////////////////////////////////////////////////
void SimbodyScrewJoint::SetForceImpl(unsigned int _index, double _torque)
{
  if (_index < this->DOF() &&
      this->physicsInitialized)
    this->simbodyPhysics->discreteForces.setOneMobilityForce(
      this->simbodyPhysics->integ->updAdvancedState(),
      this->mobod, SimTK::MobilizerUIndex(_index), _torque);
}

//////////////////////////////////////////////////
ignition::math::Vector3d SimbodyScrewJoint::GlobalAxis(
    const unsigned int _index) const
{
  if (this->simbodyPhysics &&
      this->simbodyPhysics->simbodyPhysicsStepped &&
      _index < this->DOF())
  {
    if (!this->mobod.isEmptyHandle())
    {
      const SimTK::Transform &X_OM = this->mobod.getOutboardFrame(
        this->simbodyPhysics->integ->getState());

      // express Z-axis of X_OM in world frame
      SimTK::Vec3 z_W(this->mobod.expressVectorInGroundFrame(
        this->simbodyPhysics->integ->getState(), X_OM.z()));

      return SimbodyPhysics::Vec3ToVector3Ign(z_W);
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
    if (_index >= this->DOF())
    {
      gzerr << "index out of bound\n";
      return ignition::math::Vector3d(SimTK::NaN, SimTK::NaN, SimTK::NaN);
    }
    else
    {
      gzdbg << "SimbodyScrewJoint::GlobalAxis() sibmody physics"
            << " engine not initialized yet, "
            << "use local axis and initial pose to compute "
            << "global axis.\n";
      // if local axis specified in model frame (to be changed)
      // switch to code below if issue #494 is to be addressed
      return this->AxisFrame(_index).RotateVector(
        this->LocalAxis(_index));
    }
  }
}

//////////////////////////////////////////////////
double SimbodyScrewJoint::PositionImpl(const unsigned int _index) const
{
  if (_index < this->DOF())
  {
    if (this->physicsInitialized &&
        this->simbodyPhysics->simbodyPhysicsInitialized)
    {
      if (!this->mobod.isEmptyHandle())
      {
        // simbody screw joint only has one dof
        // _index=0: angular dof
        // _index=1: linear dof
        double position = this->mobod.getOneQ(
          this->simbodyPhysics->integ->getState(), 0);
        if (_index == 1)
        {
          // return linear position
          // thread pitch units rad/m
          position /= this->threadPitch;
        }
        return position;
      }
      else
      {
        gzerr << "Joint mobod not initialized correctly.  Please file"
              << " a report on issue tracker.\n";
        return ignition::math::NAN_D;
      }
    }
    else
    {
      gzdbg << "SimbodyScrewJoint::PositionImpl() simbody not yet initialized, "
            << "initial angle should be zero until <initial_angle> "
            << "is implemented.\n";
      return 0.0;
    }
  }
  else
  {
    gzerr << "Invalid index[" << _index << "]\n";
    return ignition::math::NAN_D;
  }
}

//////////////////////////////////////////////////
double SimbodyScrewJoint::GetThreadPitch(unsigned int /*_index*/)
{
  return this->GetThreadPitch();
}

//////////////////////////////////////////////////
double SimbodyScrewJoint::GetThreadPitch()
{
  if (!this->mobod.isEmptyHandle() && this->physicsInitialized &&
      this->simbodyPhysics->simbodyPhysicsInitialized)
  {
    // downcast mobod to screw mobod first
    // the way pitch is defined in simbody is -1.0 / gazebo thread pitch
    return
      -1.0/SimTK::MobilizedBody::Screw::downcast(this->mobod).getDefaultPitch();
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
  unsigned int _index,
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
double SimbodyScrewJoint::GetParam(const std::string &_key,
  unsigned int _index)
{
  if (_key  == "thread_pitch")
    return this->threadPitch;
  else
    return SimbodyJoint::GetParam(_key, _index);
}

//////////////////////////////////////////////////
void SimbodyScrewJoint::SetUpperLimit(const unsigned int _index,
                                      const double _limit)
{
  Joint::SetUpperLimit(_index, _limit);

  if (_index < this->DOF())
  {
    if (this->physicsInitialized)
    {
      // check if limitForce is initialized
      if (this->limitForce[_index].isEmptyHandle())
      {
        gzerr << "child link is null, force element not initialized, "
              << "SetUpperLimit failed. Please file a report on issue "
              << "tracker.\n";
        return;
      }

      if (_index == 0)
      {
        // angular limit is specified
        this->limitForce[0].setBounds(
          this->simbodyPhysics->integ->updAdvancedState(),
          this->LowerLimit(_index), _limit);
      }
      else if (_index == 1)
      {
        double tp = this->GetThreadPitch();
        if (ignition::math::equal(tp, 0.0))
        {
          gzerr << "thread pitch should not be zero (joint is a slider?)"
                << " using thread pitch = 1.0e6\n";
          tp = 1.0e6;
        }
        // only angular limiting force element is added for
        // screw joints in SimbodyPhysics.cc
        if (tp > 0)
        {
          // incoming _limit is the linear dof, which is angle / thread_pitch.
          // convert linear limit to angular limit
          double upper = _limit / tp;
          this->limitForce[0].setBounds(
            this->simbodyPhysics->integ->updAdvancedState(),
            this->LowerLimit(_index), upper);
        }
        else
        {
          // incoming _limit is the linear dof, which is angle / thread_pitch.
          // convert linear limit to angular limit
          // tp is negative, this is actually upper linear limit, or the
          // lower angular limit.
          double lower = _limit / tp;
          this->limitForce[0].setBounds(
            this->simbodyPhysics->integ->updAdvancedState(),
            lower, this->UpperLimit(_index));
        }
      }
      else
      {
        gzerr << "Should never be here. Joint index invalid limit not set.\n";
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
void SimbodyScrewJoint::SetLowerLimit(const unsigned int _index,
                                      const double _limit)
{
  Joint::SetLowerLimit(_index, _limit);

  if (_index < this->DOF())
  {
    if (this->physicsInitialized)
    {
      // check if limitForce is initialized
      if (this->limitForce[_index].isEmptyHandle())
      {
        gzerr << "child link is null, force element not initialized, "
              << "SetLowerLimit failed. Please file a report on issue "
              << "tracker.\n";
        return;
      }

      if (_index == 0)
      {
        // angular limit is specified
        this->limitForce[0].setBounds(
          this->simbodyPhysics->integ->updAdvancedState(),
          _limit,
          this->UpperLimit(_index));
      }
      else if (_index == 1)
      {
        double tp = this->GetThreadPitch();
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
          // incoming _limit is the linear dof, which is angle / thread_pitch.
          // convert linear limit to angular limit
          double lower = _limit / tp;
          this->limitForce[0].setBounds(
            this->simbodyPhysics->integ->updAdvancedState(),
            lower, this->UpperLimit(_index));
        }
        else
        {
          // incoming _limit is the linear dof, which is angle / thread_pitch.
          // convert linear limit to angular limit
          // tp is negative, this is actually lower linear limit, or the
          // upper angular limit.
          double upper = _limit / tp;
          this->limitForce[0].setBounds(
            this->simbodyPhysics->integ->updAdvancedState(),
            this->UpperLimit(_index), upper);
        }
      }
      else
      {
        gzerr << "Should never be here. Joint index invalid limit not set.\n";
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
double SimbodyScrewJoint::UpperLimit(const unsigned int _index) const
{
  if (_index >= this->DOF())
  {
    gzerr << "Invalid joint index [" << _index
          << "] when trying to get high stop\n";
    return ignition::math::NAN_D;
  }
  else if (_index == 0)
  {
    return SimbodyJoint::UpperLimit(0);
  }
  else if (_index == 1)
  {
    /// \todo Make GetThreadPitch const
    double tp = const_cast<SimbodyScrewJoint *>(this)->GetThreadPitch();
    if (ignition::math::equal(tp, 0.0))
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
    gzerr << "Should not be here in code, DOF > 2?\n";
    return ignition::math::NAN_D;
  }
}

//////////////////////////////////////////////////
double SimbodyScrewJoint::LowerLimit(const unsigned int _index) const
{
  if (_index >= this->DOF())
  {
    gzerr << "Invalid joint index [" << _index
          << "] when trying to get low stop\n";
    return ignition::math::NAN_D;
  }
  else if (_index == 0)
  {
    return SimbodyJoint::LowerLimit(0);
  }
  else if (_index == 1)
  {
    double tp = const_cast<SimbodyScrewJoint *>(this)->GetThreadPitch();
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
    gzerr << "Should not be here in code, DOF > 2?\n";
    return ignition::math::NAN_D;
  }
}
