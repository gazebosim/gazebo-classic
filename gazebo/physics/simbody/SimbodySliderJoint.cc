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

#include "gazebo/common/Console.hh"
#include "gazebo/common/Exception.hh"

#include "gazebo/physics/Model.hh"
#include "gazebo/physics/simbody/simbody_inc.h"
#include "gazebo/physics/simbody/SimbodyLink.hh"
#include "gazebo/physics/simbody/SimbodyPhysics.hh"
#include "gazebo/physics/simbody/SimbodySliderJoint.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
SimbodySliderJoint::SimbodySliderJoint(SimTK::MultibodySystem * /*_world*/,
                                     BasePtr _parent)
    : SliderJoint<SimbodyJoint>(_parent)
{
  this->physicsInitialized = false;
}

//////////////////////////////////////////////////
SimbodySliderJoint::~SimbodySliderJoint()
{
}

//////////////////////////////////////////////////
void SimbodySliderJoint::Load(sdf::ElementPtr _sdf)
{
  SliderJoint<SimbodyJoint>::Load(_sdf);
}

//////////////////////////////////////////////////
void SimbodySliderJoint::SetAxis(int /*_index*/, const math::Vector3 &/*_axis*/)
{
  // Simbody seems to handle setAxis improperly. It readjust all the pivot
  // points
  gzdbg << "SetAxis Not implemented...\n";
}

//////////////////////////////////////////////////
void SimbodySliderJoint::SetDamping(int _index, double _damping)
{
  if (_index < static_cast<int>(this->GetAngleCount()))
  {
    this->dampingCoefficient = _damping;
    this->damper.setDamping(
      this->simbodyPhysics->integ->updAdvancedState(),
      _damping);
  }
  else
    gzerr << "SetDamping _index too large.\n";
}

//////////////////////////////////////////////////
void SimbodySliderJoint::SetVelocity(int _index, double _rate)
{
  if (_index < static_cast<int>(this->GetAngleCount()))
    this->mobod.setOneU(
      this->simbodyPhysics->integ->updAdvancedState(),
      SimTK::MobilizerUIndex(_index), _rate);
  else
    gzerr << "SetDamping _index too large.\n";
}

//////////////////////////////////////////////////
double SimbodySliderJoint::GetVelocity(int _index) const
{
  if (_index < static_cast<int>(this->GetAngleCount()))
  {
    if (this->simbodyPhysics->simbodyPhysicsInitialized)
      return this->mobod.getOneU(
        this->simbodyPhysics->integ->getState(),
        SimTK::MobilizerUIndex(_index));
    else
    {
      gzdbg << "Simbody::GetVelocity() simbody not yet initialized, "
            << "initial velocity should be zero until restart from "
            << "state has been implemented.\n";
      return 0.0;
    }
  }
  else
  {
    gzerr << "Invalid index for joint, returning NaN\n";
    return SimTK::NaN;
  }
}

//////////////////////////////////////////////////
void SimbodySliderJoint::SetMaxForce(int /*_index*/, double /*_t*/)
{
  gzdbg << "SetMaxForce doesn't make sense in simbody...\n";
}

//////////////////////////////////////////////////
double SimbodySliderJoint::GetMaxForce(int /*_index*/)
{
  gzdbg << "GetMaxForce doesn't make sense in simbody...\n";
  return 0;
}

//////////////////////////////////////////////////
void SimbodySliderJoint::SetForceImpl(int _index, double _torque)
{
  if (_index < static_cast<int>(this->GetAngleCount()))
    this->simbodyPhysics->discreteForces.setOneMobilityForce(
      this->simbodyPhysics->integ->updAdvancedState(),
      this->mobod, SimTK::MobilizerUIndex(_index), _torque);
}

//////////////////////////////////////////////////
void SimbodySliderJoint::SetHighStop(int _index,
                                   const math::Angle &_angle)
{
  if (_index < static_cast<int>(this->GetAngleCount()))
  {
    Joint::SetHighStop(_index, _angle);
    if (this->physicsInitialized)
    {
      this->limitForce.setBounds(
        this->simbodyPhysics->integ->updAdvancedState(),
        this->limitForce.getLowerBound(
          this->simbodyPhysics->integ->updAdvancedState()),
        _angle.Radian());
    }
    else
    {
      gzerr << "SetHighStop: State not initialized, SetLowStop failed.\n";
    }
  }
  else
    gzerr << "SetHighStop: index out of bounds.\n";
}

//////////////////////////////////////////////////
void SimbodySliderJoint::SetLowStop(int _index,
                                  const math::Angle &_angle)
{
  if (_index < static_cast<int>(this->GetAngleCount()))
  {
    Joint::SetLowStop(_index, _angle);
    if (this->physicsInitialized)
    {
      this->limitForce.setBounds(
        this->simbodyPhysics->integ->updAdvancedState(),
        _angle.Radian(),
        this->limitForce.getUpperBound(
          this->simbodyPhysics->integ->updAdvancedState()));
    }
    else
    {
      gzerr << "SetLowStop: State not initialized, SetLowStop failed.\n";
    }
  }
  else
    gzerr << "SetLowStop: index out of bounds.\n";
}

//////////////////////////////////////////////////
math::Angle SimbodySliderJoint::GetHighStop(int _index)
{
  if (_index >= static_cast<int>(this->GetAngleCount()))
  {
    gzerr << "Invalid joint index [" << _index
          << "] when trying to get high stop\n";
    return math::Angle(0.0);  /// \TODO: should return NaN
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
    return math::Angle(0.0);  /// \TODO: should return NaN
  }
}

//////////////////////////////////////////////////
math::Angle SimbodySliderJoint::GetLowStop(int _index)
{
  if (_index >= static_cast<int>(this->GetAngleCount()))
  {
    gzerr << "Invalid joint index [" << _index
          << "] when trying to get low stop\n";
    return math::Angle(0.0);  /// \TODO: should return NaN
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
    return math::Angle(0.0);  /// \TODO: should return NaN
  }
}

//////////////////////////////////////////////////
math::Vector3 SimbodySliderJoint::GetGlobalAxis(int _index) const
{
  if (this->simbodyPhysics->simbodyPhysicsStepped &&
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
      gzdbg << "Simbody::GetGlobalAxis() sibmody physics"
            << " engine not initialized yet, "
            << "use local axis and initial pose to compute "
            << "global axis.\n";

      // if local axis specified in model frame (to be changed)
      // switch to code below if issue #494 is to be addressed
      return this->model->GetWorldPose().rot.RotateVector(
        this->GetLocalAxis(_index));

      // if local axis specified in joint frame (Issue #494)
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
math::Angle SimbodySliderJoint::GetAngleImpl(int _index) const
{
  if (_index < static_cast<int>(this->GetAngleCount()))
  {
    if (this->simbodyPhysics->simbodyPhysicsInitialized)
      return math::Angle(this->mobod.getOneQ(
        this->simbodyPhysics->integ->getState(), _index));
    else
    {
      gzdbg << "Simbody::GetAngleImpl() simbody not yet initialized, "
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
