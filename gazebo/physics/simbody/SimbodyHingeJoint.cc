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

#include "ignition/common/Assert.hh"
#include "ignition/common/Console.hh"
#include "ignition/common/Exception.hh"

#include "gazebo/physics/Model.hh"
#include "gazebo/physics/simbody/SimbodyLink.hh"
#include "gazebo/physics/simbody/SimbodyPhysics.hh"
#include "gazebo/physics/simbody/SimbodyHingeJoint.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
SimbodyHingeJoint::SimbodyHingeJoint(SimTK::MultibodySystem * /*_world*/,
                                     BasePtr _parent)
    : HingeJoint<SimbodyJoint>(_parent)
{
  this->physicsInitialized = false;
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
    const ignition::math::Vector3 &/*_axis*/)
{
  // Simbody seems to handle setAxis improperly. It readjust all the pivot
  // points
  igndbg << "SetAxis: setting axis is not yet implemented.  The axis are set "
        << " during joint construction in SimbodyPhysics.cc for now.\n";
}

//////////////////////////////////////////////////
void SimbodyHingeJoint::SetVelocity(unsigned int _index, double _rate)
{
  if (_index < this->GetAngleCount())
    this->mobod.setOneU(
      this->simbodyPhysics->integ->updAdvancedState(),
      SimTK::MobilizerUIndex(_index), _rate);
  else
    ignerr << "SetVelocity _index too large.\n";
}

//////////////////////////////////////////////////
double SimbodyHingeJoint::GetVelocity(unsigned int _index) const
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
      igndbg << "GetVelocity() simbody not yet initialized, "
            << "initial velocity should be zero until restart from "
            << "state has been implemented.\n";
      return 0.0;
    }
  }
  else
  {
    ignerr << "Invalid index for joint, returning NaN\n";
    return SimTK::NaN;
  }
}

//////////////////////////////////////////////////
void SimbodyHingeJoint::SetMaxForce(unsigned int /*_index*/, double /*_t*/)
{
  igndbg << "SetMaxForce: doesn't make sense in simbody...\n";
}

//////////////////////////////////////////////////
double SimbodyHingeJoint::GetMaxForce(unsigned int /*_index*/)
{
  igndbg << "GetMaxForce: doesn't make sense in simbody...\n";
  return 0;
}

//////////////////////////////////////////////////
void SimbodyHingeJoint::SetForceImpl(unsigned int _index, double _torque)
{
  if (_index < this->GetAngleCount() && this->physicsInitialized)
    this->simbodyPhysics->discreteForces.setOneMobilityForce(
      this->simbodyPhysics->integ->updAdvancedState(),
      this->mobod, SimTK::MobilizerUIndex(_index), _torque);
}

//////////////////////////////////////////////////
void SimbodyHingeJoint::SetHighStop(unsigned int _index,
                                   const ignition::math::Angle &_angle)
{
  if (_index < this->GetAngleCount())
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
      ignerr << "SetHighStop: State not initialized, SetLowStop failed.\n";
    }
  }
  else
    ignerr << "SetHighStop: index out of bounds.\n";
}

//////////////////////////////////////////////////
void SimbodyHingeJoint::SetLowStop(unsigned int _index,
                                  const ignition::math::Angle &_angle)
{
  if (_index < this->GetAngleCount())
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
      ignerr << "SetLowStop: State not initialized, SetLowStop failed.\n";
    }
  }
  else
    ignerr << "SetLowStop: index out of bounds.\n";
}

//////////////////////////////////////////////////
ignition::math::Angle SimbodyHingeJoint::GetHighStop(unsigned int _index)
{
  if (_index >= this->GetAngleCount())
  {
    ignerr << "GetHighStop: Invalid joint index ["
          << _index << "] when trying to get high stop\n";
    return ignition::math::Angle(0.0);  /// \TODO: should return NaN
  }
  else if (_index == 0)
  {
    return ignition::math::Angle(
        this->sdf->GetElement("axis")->GetElement(
          "limit")->Get<double>("upper"));
  }
  else
  {
    ignerr << "GetHighStop: Should not be here in code, GetAngleCount < 0.\n";
    return ignition::math::Angle(0.0);  /// \TODO: should return NaN
  }
}

//////////////////////////////////////////////////
ignition::math::Angle SimbodyHingeJoint::GetLowStop(unsigned int _index)
{
  if (_index >= this->GetAngleCount())
  {
    ignerr << "GetLowStop: Invalid joint index [" << _index
          << "] when trying to get low stop\n";
    return ignition::math::Angle(0.0);  /// \TODO: should return NaN
  }
  else if (_index == 0)
  {
    return ignition::math::Angle(
        this->sdf->GetElement("axis")->GetElement(
          "limit")->Get<double>("lower"));
  }
  else
  {
    ignerr << "GetLowStop: Should not be here in code, GetAngleCount < 0.\n";
    return ignition::math::Angle(0.0);  /// \TODO: should return NaN
  }
}

//////////////////////////////////////////////////
ignition::math::Vector3 SimbodyHingeJoint::GetGlobalAxis(
    unsigned int _index) const
{
  if (this->simbodyPhysics &&
      this->simbodyPhysics->simbodyPhysicsStepped &&
      _index < this->GetAngleCount())
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
    if (_index >= this->GetAngleCount())
    {
      ignerr << "index out of bound\n";
      return ignition::math::Vector3(SimTK::NaN, SimTK::NaN, SimTK::NaN);
    }
    else
    {
      igndbg << "GetGlobalAxis() sibmody physics engine not initialized yet, "
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
      //  ignition::math::Pose jointPose =
      //    this->anchorPose + this->childLink->GetWorldPose();
      //   return jointPose.rot.RotateVector(this->GetLocalAxis(_index));
      // }
      // else
      // {
      //   ignerr << "Joint [" << this->GetName() << "] missing child link.\n";
      //   return ignition::math::Vector3(SimTK::NaN, SimTK::NaN, SimTK::NaN);
      // }
    }
  }
}

//////////////////////////////////////////////////
ignition::math::Angle SimbodyHingeJoint::GetAngleImpl(unsigned int _index) const
{
  if (_index < this->GetAngleCount())
  {
    if (this->physicsInitialized &&
        this->simbodyPhysics->simbodyPhysicsInitialized)
      return ignition::math::Angle(this->mobod.getOneQ(
        this->simbodyPhysics->integ->getState(), _index));
    else
    {
      igndbg << "GetAngleImpl() simbody not yet initialized, "
            << "initial angle should be zero until <initial_angle> "
            << "is implemented.\n";
      return ignition::math::Angle(0.0);
    }
  }
  else
  {
    ignerr << "index out of bound\n";
    return ignition::math::Angle(SimTK::NaN);
  }
}

//////////////////////////////////////////////////
void SimbodyHingeJoint::SaveSimbodyState(const SimTK::State &_state)
{
  if (!this->mobod.isEmptyHandle())
  {
    if (this->simbodyQ.empty())
      this->simbodyQ.resize(this->mobod.getNumQ(_state));

    if (this->simbodyU.empty())
      this->simbodyU.resize(this->mobod.getNumU(_state));

    for (unsigned int i = 0; i < this->simbodyQ.size(); ++i)
      this->simbodyQ[i] = this->mobod.getOneQ(_state, i);

    for (unsigned int i = 0; i < this->simbodyU.size(); ++i)
      this->simbodyU[i] = this->mobod.getOneU(_state, i);
  }
  else
  {
    // ignerr << "debug: joint name: " << this->GetScopedName() << "\n";
  }
}

//////////////////////////////////////////////////
void SimbodyHingeJoint::RestoreSimbodyState(SimTK::State &_state)
{
  if (!this->mobod.isEmptyHandle())
  {
    for (unsigned int i = 0; i < this->simbodyQ.size(); ++i)
      this->mobod.setOneQ(_state, i, this->simbodyQ[i]);

    for (unsigned int i = 0; i < this->simbodyU.size(); ++i)
      this->mobod.setOneU(_state, i, this->simbodyU[i]);
  }
  else
  {
    // ignerr << "restoring model [" << this->GetScopedName()
    //       << "] failed due to uninitialized mobod\n";
  }
}
