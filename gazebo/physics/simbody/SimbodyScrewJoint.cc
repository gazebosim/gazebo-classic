/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
  if (_index < this->GetAngleCount())
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
  if (_index < this->GetAngleCount())
    this->mobod.setOneU(
      this->simbodyPhysics->integ->updAdvancedState(),
      SimTK::MobilizerUIndex(_index), _rate);
  else
    gzerr << "SimbodyScrewJoint::SetVelocity _index too large.\n";
}

//////////////////////////////////////////////////
void SimbodyScrewJoint::SetAxis(unsigned int /*_index*/,
    const math::Vector3 &/*_axis*/)
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
  if (_index < this->GetAngleCount() &&
      this->physicsInitialized)
    this->simbodyPhysics->discreteForces.setOneMobilityForce(
      this->simbodyPhysics->integ->updAdvancedState(),
      this->mobod, SimTK::MobilizerUIndex(_index), _torque);
}

//////////////////////////////////////////////////
void SimbodyScrewJoint::SetMaxForce(unsigned int /*_index*/, double /*_force*/)
{
  gzdbg << "SimbodyScrewJoint::SetMaxForce: doesn't make sense in simbody...\n";
}

//////////////////////////////////////////////////
double SimbodyScrewJoint::GetMaxForce(unsigned int /*index*/)
{
  gzdbg << "SimbodyScrewJoint::GetMaxForce: doesn't make sense in simbody...\n";
  return 0;
}

//////////////////////////////////////////////////
math::Vector3 SimbodyScrewJoint::GetGlobalAxis(unsigned int _index) const
{
  if (this->simbodyPhysics &&
      this->simbodyPhysics->simbodyPhysicsStepped &&
      _index < this->GetAngleCount())
  {
    if (!this->mobod.isEmptyHandle())
    {
      const SimTK::Transform &X_OM = this->mobod.getOutboardFrame(
        this->simbodyPhysics->integ->getState());

      // express Z-axis of X_OM in world frame
      SimTK::Vec3 z_W(this->mobod.expressVectorInGroundFrame(
        this->simbodyPhysics->integ->getState(), X_OM.z()));

      return SimbodyPhysics::Vec3ToVector3(z_W);
    }
    else
    {
      gzerr << "Joint mobod not initialized correctly.  Returning"
            << " initial axis vector in world frame (not valid if"
            << " joint frame has moved). Please file"
            << " a report on issue tracker.\n";
      return this->GetAxisFrame(_index).RotateVector(
        this->GetLocalAxis(_index));
    }
  }
  else
  {
    if (_index >= this->GetAngleCount())
    {
      gzerr << "index out of bound\n";
      return math::Vector3(SimTK::NaN, SimTK::NaN, SimTK::NaN);
    }
    else
    {
      gzdbg << "SimbodyScrewJoint::GetGlobalAxis() sibmody physics"
            << " engine not initialized yet, "
            << "use local axis and initial pose to compute "
            << "global axis.\n";
      // if local axis specified in model frame (to be changed)
      // switch to code below if issue #494 is to be addressed
      return this->GetAxisFrame(_index).RotateVector(
        this->GetLocalAxis(_index));
    }
  }
}

//////////////////////////////////////////////////
math::Angle SimbodyScrewJoint::GetAngleImpl(unsigned int _index) const
{
  if (_index < this->GetAngleCount())
  {
    if (this->physicsInitialized &&
        this->simbodyPhysics->simbodyPhysicsInitialized)
    {
      if (!this->mobod.isEmptyHandle())
      {
        // simbody screw joint only has one dof
        // _index=0: angular dof
        // _index=1: linear dof
        math::Angle angle(this->mobod.getOneQ(
          this->simbodyPhysics->integ->getState(), 0));
        if (_index == 1)
        {
          // return linear position
          // thread pitch units rad/m
          angle /= math::Angle(this->threadPitch);
        }
        return angle;
      }
      else
      {
        gzerr << "Joint mobod not initialized correctly.  Please file"
              << " a report on issue tracker.\n";
        return math::Angle(0.0);
      }
    }
    else
    {
      gzdbg << "SimbodyScrewJoint::GetAngleImpl() simbody not yet initialized, "
            << "initial angle should be zero until <initial_angle> "
            << "is implemented.\n";
      return math::Angle(0.0);
    }
  }
  else
  {
    gzerr << "index out of bound\n";
    return math::Angle(SimTK::NaN);
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
  if (this->physicsInitialized &&
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
void SimbodyScrewJoint::SetAttribute(const std::string &_key,
  unsigned int _index,
  const boost::any &_value)
{
  this->SetParam(_key, _index, _value);
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
double SimbodyScrewJoint::GetAttribute(const std::string &_key,
  unsigned int _index)
{
  return this->GetParam(_key, _index);
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
bool SimbodyScrewJoint::SetHighStop(
  unsigned int _index, const math::Angle &_angle)
{
  Joint::SetHighStop(_index, _angle);

  if (_index < this->GetAngleCount())
  {
    if (this->physicsInitialized)
    {
      if (_index == 0)
      {
        // angular limit is specified
        this->limitForce[0].setBounds(
          this->simbodyPhysics->integ->updAdvancedState(),
          this->GetLowStop(_index).Radian(), _angle.Radian());
      }
      else if (_index == 1)
      {
        double tp = this->GetThreadPitch();
        if (math::equal(tp, 0.0))
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
          this->limitForce[0].setBounds(
            this->simbodyPhysics->integ->updAdvancedState(),
            this->GetLowStop(_index).Radian(), upper);
        }
        else
        {
          // incoming _angle is the linear dof, which is _angle / thread_pitch.
          // convert linear limit to angular limit
          // tp is negative, this is actually upper linear limit, or the
          // lower angular limit.
          double lower = _angle.Radian() / tp;
          this->limitForce[0].setBounds(
            this->simbodyPhysics->integ->updAdvancedState(),
            lower, this->GetHighStop(_index).Radian());
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
  unsigned int _index, const math::Angle &_angle)
{
  Joint::SetLowStop(_index, _angle);

  if (_index < this->GetAngleCount())
  {
    if (this->physicsInitialized)
    {
      if (_index == 0)
      {
        // angular limit is specified
        this->limitForce[0].setBounds(
          this->simbodyPhysics->integ->updAdvancedState(),
          _angle.Radian(),
          this->GetHighStop(_index).Radian());
      }
      else if (_index == 1)
      {
        double tp = this->GetThreadPitch();
        if (math::equal(tp, 0.0))
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
          this->limitForce[0].setBounds(
            this->simbodyPhysics->integ->updAdvancedState(),
            lower, this->GetHighStop(_index).Radian());
        }
        else
        {
          // incoming _angle is the linear dof, which is _angle / thread_pitch.
          // convert linear limit to angular limit
          // tp is negative, this is actually lower linear limit, or the
          // upper angular limit.
          double upper = _angle.Radian() / tp;
          this->limitForce[0].setBounds(
            this->simbodyPhysics->integ->updAdvancedState(),
            this->GetHighStop(_index).Radian(), upper);
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
math::Angle SimbodyScrewJoint::GetHighStop(unsigned int _index)
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
    return this->GetUpperLimit(0);
  }
  else if (_index == 1)
  {
    double tp = this->GetThreadPitch();
    if (math::equal(tp, 0.0))
    {
      gzerr << "thread pitch should not be zero (joint is a slider?)"
            << " using thread pitch = 1.0e6\n";
      tp = 1.0e6;
    }
    if (tp > 0)
    {
      return this->GetUpperLimit(0) / tp;
    }
    else
    {
      return this->GetLowerLimit(0) / tp;
    }
  }
  else
  {
    gzerr << "Should not be here in code, GetAngleCount > 2?\n";
    /// \TODO: should return NaN
    return math::Angle(0.0);
  }
}

//////////////////////////////////////////////////
math::Angle SimbodyScrewJoint::GetLowStop(unsigned int _index)
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
    return this->GetLowerLimit(0);
  }
  else if (_index == 1)
  {
    double tp = this->GetThreadPitch();
    if (math::equal(tp, 0.0))
    {
      gzerr << "thread pitch should not be zero (joint is a slider?)"
            << " using thread pitch = 1.0e6\n";
      tp = 1.0e6;
    }
    if (tp > 0)
    {
      return this->GetLowerLimit(0) / tp;
    }
    else
    {
      return this->GetUpperLimit(0) / tp;
    }
  }
  else
  {
    gzerr << "Should not be here in code, GetAngleCount > 2?\n";
    /// \TODO: should return NaN
    return math::Angle(0.0);
  }
}
