/*
 * Copyright 2012 Open Source Robotics Foundation
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
/* Desc: A SimbodyHingeJoint
 * Author: Nate Koenig, Andrew Howard
 * Date: 21 May 2003
 */
#include "gazebo/common/Assert.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Exception.hh"

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
void SimbodyHingeJoint::SetAxis(int /*_index*/, const math::Vector3 &/*_axis*/)
{
  // Simbody seems to handle setAxis improperly. It readjust all the pivot
  // points
  gzdbg << "SetAxis Not implemented...\n";
}

//////////////////////////////////////////////////
void SimbodyHingeJoint::SetDamping(int _index, double _damping)
{
  if (_index < this->GetAngleCount())
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
void SimbodyHingeJoint::SetVelocity(int _index, double _rate)
{
  if (_index < this->GetAngleCount())
    this->mobod.setOneU(
      this->simbodyPhysics->integ->updAdvancedState(),
      SimTK::MobilizerUIndex(_index), _rate);
  else
    gzerr << "SetDamping _index too large.\n";
}

//////////////////////////////////////////////////
double SimbodyHingeJoint::GetVelocity(int _index) const
{
  if (_index < this->GetAngleCount())
  {
    if (this->simbodyPhysics->simbodyPhysicsInitialized)
      return this->mobod.getOneU(
        this->simbodyPhysics->integ->getState(),
        SimTK::MobilizerUIndex(_index));
    else
    {
      gzwarn << "Simbody::GetVelocity() simbody not yet initialized\n";
      return SimTK::NaN;
    }
  }
  else
  {
    gzerr << "Invalid index for joint, returning NaN\n";
    return SimTK::NaN;
  }
}

//////////////////////////////////////////////////
void SimbodyHingeJoint::SetMaxForce(int /*_index*/, double _t)
{
  gzdbg << "SetMaxForce doesn't make sense in simbody...\n";
}

//////////////////////////////////////////////////
double SimbodyHingeJoint::GetMaxForce(int /*_index*/)
{
  gzdbg << "GetMaxForce doesn't make sense in simbody...\n";
  return 0;
}

//////////////////////////////////////////////////
void SimbodyHingeJoint::SetForce(int _index, double _torque)
{
  SimbodyJoint::SetForce(_index, _torque);

  if (_index < this->GetAngleCount())
    this->simbodyPhysics->discreteForces.setOneMobilityForce(
      this->simbodyPhysics->integ->updAdvancedState(),
      this->mobod, SimTK::MobilizerUIndex(_index), _torque);
}

//////////////////////////////////////////////////
void SimbodyHingeJoint::SetHighStop(int _index,
                                   const math::Angle &_angle)
{
  gzdbg << "SetHighStop\n";
  if (_index < this->GetAngleCount())
  {
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
      gzwarn << "SetHighStop: State not initialized.\n";
    }
  }
  else
    gzerr << "SetHighStop: index out of bounds.\n";
}

//////////////////////////////////////////////////
void SimbodyHingeJoint::SetLowStop(int _index,
                                  const math::Angle &_angle)
{
  gzdbg << "SetLowStop\n";
  if (_index < this->GetAngleCount())
  {
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
      gzwarn << "SetLowStop: State not initialized.\n";
    }
  }
  else
    gzerr << "SetLowStop: index out of bounds.\n";
}

//////////////////////////////////////////////////
math::Angle SimbodyHingeJoint::GetHighStop(int _index)
{
  if (_index < this->GetAngleCount())
    return math::Angle(this->limitForce.getUpperBound(
            this->simbodyPhysics->integ->getAdvancedState()));
  else
  {
    gzerr << "GetHighStop _index too large.\n";
    return math::Angle(SimTK::NaN);
  }
}

//////////////////////////////////////////////////
math::Angle SimbodyHingeJoint::GetLowStop(int _index)
{
  if (_index < this->GetAngleCount())
    return math::Angle(this->limitForce.getLowerBound(
            this->simbodyPhysics->integ->getAdvancedState()));
  else
  {
    gzerr << "GetLowStop _index too large.\n";
    return math::Angle(SimTK::NaN);
  }
}

//////////////////////////////////////////////////
math::Vector3 SimbodyHingeJoint::GetGlobalAxis(int _index) const
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
      gzerr << "index out of bound\n";
    else
      gzwarn << "Simbody::GetGlobalAxis() sibmody physics"
             << " engine not initialized yet.\n";
    return math::Vector3(SimTK::NaN, SimTK::NaN, SimTK::NaN);
  }
}

//////////////////////////////////////////////////
math::Angle SimbodyHingeJoint::GetAngleImpl(int _index) const
{
  if (_index < static_cast<int>(this->GetAngleCount()))
  {
    if (this->simbodyPhysics->simbodyPhysicsInitialized)
      return math::Angle(this->mobod.getOneQ(
        this->simbodyPhysics->integ->getState(), _index));
    else
    {
      gzwarn << "Simbody::GetAngleImpl() simbody not yet initialized\n";
      return math::Angle(SimTK::NaN);
    }
  }
  else
  {
    gzerr << "index out of bound\n";
    return math::Angle(SimTK::NaN);
  }
}
