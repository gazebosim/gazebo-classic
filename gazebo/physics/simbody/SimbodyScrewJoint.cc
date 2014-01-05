/*
 * Copyright (C) 2012-2013 Open Source Robotics Foundation
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
void SimbodyScrewJoint::SetAxis(int /*_index*/, const math::Vector3 &/*_axis*/)
{
  // Simbody seems to handle setAxis improperly. It readjust all the pivot
  // points
  gzdbg << "SimbodyScrewJoint::SetAxis: setting axis is "
        << "not yet implemented.  The axis are set during joint construction "
        << "in SimbodyPhysics.cc for now.\n";
}

//////////////////////////////////////////////////
void SimbodyScrewJoint::SetVelocity(int _index, double _rate)
{
  if (_index < static_cast<int>(this->GetAngleCount()))
    this->mobod.setOneU(
      this->simbodyPhysics->integ->updAdvancedState(),
      SimTK::MobilizerUIndex(_index), _rate);
  else
    gzerr << "SimbodyScrewJoint::SetVelocity _index too large.\n";
}

//////////////////////////////////////////////////
double SimbodyScrewJoint::GetVelocity(int _index) const
{
  if (_index < static_cast<int>(this->GetAngleCount()))
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
void SimbodyScrewJoint::SetMaxForce(int /*_index*/, double /*_force*/)
{
  gzdbg << "SimbodyScrewJoint::SetMaxForce: doesn't make sense in simbody...\n";
}

//////////////////////////////////////////////////
double SimbodyScrewJoint::GetMaxForce(int /*index*/)
{
  gzdbg << "SimbodyScrewJoint::GetMaxForce: doesn't make sense in simbody...\n";
  return 0;
}

//////////////////////////////////////////////////
void SimbodyScrewJoint::SetForceImpl(int _index, double _torque)
{
  if (_index < static_cast<int>(this->GetAngleCount()) &&
      this->physicsInitialized)
    this->simbodyPhysics->discreteForces.setOneMobilityForce(
      this->simbodyPhysics->integ->updAdvancedState(),
      this->mobod, SimTK::MobilizerUIndex(_index), _torque);
}

//////////////////////////////////////////////////
void SimbodyScrewJoint::SetHighStop(int _index,
  const math::Angle &_angle)
{
  if (_index < static_cast<int>(this->GetAngleCount()))
  {
    Joint::SetHighStop(_index, _angle);
    if (this->physicsInitialized)
    {
      this->limitForce[_index].setBounds(
        this->simbodyPhysics->integ->updAdvancedState(),
        this->limitForce[_index].getLowerBound(
          this->simbodyPhysics->integ->updAdvancedState()),
        _angle.Radian());
    }
    else
    {
      gzerr << "SimbodyScrewJoint::SetHighStop: State not "
            << "initialized, SetLowStop failed.\n";
    }
  }
  else
    gzerr << "SimbodyScrewJoint::SetHighStop: index out of bounds.\n";
}

//////////////////////////////////////////////////
void SimbodyScrewJoint::SetLowStop(int _index,
  const math::Angle &_angle)
{
  if (_index < static_cast<int>(this->GetAngleCount()))
  {
    Joint::SetLowStop(_index, _angle);
    if (this->physicsInitialized)
    {
      this->limitForce[_index].setBounds(
        this->simbodyPhysics->integ->updAdvancedState(),
        _angle.Radian(),
        this->limitForce[_index].getUpperBound(
          this->simbodyPhysics->integ->updAdvancedState()));
    }
    else
    {
      gzerr << "SimbodyScrewJoint::SetLowStop: State not "
            << "initialized, SetLowStop failed.\n";
    }
  }
  else
    gzerr << "SimbodyScrewJoint::SetLowStop: index out of bounds.\n";
}

//////////////////////////////////////////////////
math::Angle SimbodyScrewJoint::GetHighStop(int _index)
{
  if (_index >= static_cast<int>(this->GetAngleCount()))
  {
    gzerr << "SimbodyScrewJoint::GetHighStop: Invalid joint index ["
          << _index << "] when trying to get high stop\n";
    return math::Angle(0.0);  /// \TODO: should return NaN
  }
  else if (_index == 0)
  {
    return math::Angle(this->sdf->GetElement("axis")->GetElement("limit")
             ->Get<double>("upper"));
  }
  else
  {
    gzerr << "SimbodyScrewJoint::GetHighStop: Should not be here in code,"
          << " GetAngleCount < 0.\n";
    return math::Angle(0.0);  /// \TODO: should return NaN
  }
}

//////////////////////////////////////////////////
math::Angle SimbodyScrewJoint::GetLowStop(int _index)
{
  if (_index >= static_cast<int>(this->GetAngleCount()))
  {
    gzerr << "SimbodyScrewJoint::GetLowStop: Invalid joint index [" << _index
          << "] when trying to get low stop\n";
    return math::Angle(0.0);  /// \TODO: should return NaN
  }
  else if (_index == 0)
  {
    return math::Angle(this->sdf->GetElement("axis")->GetElement("limit")
             ->Get<double>("lower"));
  }
  else
  {
    gzerr << "SimbodyScrewJoint::GetLowStop: Should not be here in code,"
          << " GetAngleCount < 0.\n";
    return math::Angle(0.0);  /// \TODO: should return NaN
  }
}

//////////////////////////////////////////////////
math::Vector3 SimbodyScrewJoint::GetGlobalAxis(int _index) const
{
  if (this->simbodyPhysics &&
      this->simbodyPhysics->simbodyPhysicsStepped &&
      _index < static_cast<int>(this->GetAngleCount()))
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
    if (_index >= static_cast<int>(this->GetAngleCount()))
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
      return this->model->GetWorldPose().rot.RotateVector(
        this->GetLocalAxis(_index));

      // if local axis specified in joint frame (Issue #494)
      // Remember to remove include of Model.hh when switching.
      // if (this->childLink)
      // {
      //   math::Pose jointPose =
      //    this->anchorPose + this->childLink->GetWorldPose();
      //   return jointPose.rot.RotateVector(this->GetLocalAxis(_index));
      // }
      // else
      // {
      //   gzerr << "Joint [" << this->GetName() << "] missing child link.\n";
      //   return math::Vector3(SimTK::NaN, SimTK::NaN, SimTK::NaN);
      // }
    }
  }
}

//////////////////////////////////////////////////
math::Angle SimbodyScrewJoint::GetAngleImpl(int _index) const
{
  if (_index < static_cast<int>(this->GetAngleCount()))
  {
    if (this->physicsInitialized &&
        this->simbodyPhysics->simbodyPhysicsInitialized)
      return math::Angle(this->mobod.getOneQ(
        this->simbodyPhysics->integ->getState(), _index));
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
void SimbodyScrewJoint::SetThreadPitch(int /*_index*/, double /*_threadPitch*/)
{
  gzdbg << "SimbodyScrewJoint::SetThreadPitch: setting thread pitch is "
        << "not yet implemented.  The pitch are set during joint construction "
        << "in SimbodyPhysics.cc for now.\n";
}

//////////////////////////////////////////////////
void SimbodyScrewJoint::SetThreadPitch(double /*_threadPitch*/)
{
  gzdbg << "SimbodyScrewJoint::SetThreadPitch: setting thread pitch is "
        << "not yet implemented.  The pitch are set during joint construction "
        << "in SimbodyPhysics.cc for now.\n";
}

//////////////////////////////////////////////////
double SimbodyScrewJoint::GetThreadPitch(unsigned int _index)
{
  return this->threadPitch;
}

//////////////////////////////////////////////////
double SimbodyScrewJoint::GetThreadPitch()
{
  return this->threadPitch;

  /* if we want to get active thread pitch, use below
  /// \TODO: deprecate _index parameter, thread pitch is a property of the
  /// joint, not related to an axis.
  if (this->physicsInitialized &&
      this->simbodyPhysics->simbodyPhysicsInitialized)
  {
    // downcast mobod to screw mobod first
    return SimTK::MobilizedBody::Screw::downcast(this->mobod).getDefaultPitch();
  }
  else
  {
    gzdbg << "SimbodyScrewJoint::GetThreadPitch() failed, "
          << " simbody not yet initialized\n";
    return 0.0;
  }
  */
}
