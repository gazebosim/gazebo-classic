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
math::Vector3 SimbodyHingeJoint::GetAnchor(int /*_index*/) const
{
  gzerr << "Not implemented...\n";
  return math::Vector3();
}

//////////////////////////////////////////////////
void SimbodyHingeJoint::SetAnchor(int /*_index*/,
                                 const math::Vector3 &/*_anchor*/)
{
  gzerr << "SetAnchor Not implemented...\n";
  // The anchor (pivot in Simbody lingo), can only be set on creation
}

//////////////////////////////////////////////////
void SimbodyHingeJoint::SetAxis(int /*_index*/, const math::Vector3 &/*_axis*/)
{
  // Simbody seems to handle setAxis improperly. It readjust all the pivot
  // points
  gzerr << "SetAxis Not implemented...\n";
}

//////////////////////////////////////////////////
void SimbodyHingeJoint::SetDamping(int /*index*/, double /*_damping*/)
{
  gzerr << "SetDamping Not implemented\n";
}

//////////////////////////////////////////////////
void SimbodyHingeJoint::SetVelocity(int /*_index*/, double /*_angle*/)
{
  gzerr << "SetVelocity Not implemented...\n";
}

//////////////////////////////////////////////////
double SimbodyHingeJoint::GetVelocity(int _index) const
{
  if (_index < this->GetAngleCount())
    return this->mobod.getOneU(
      this->simbodyPhysics->integ->getState(),
      SimTK::MobilizerUIndex(_index));
  else
    gzerr << "Invalid index for joint, returning NaN\n";
  return SimTK::NaN;
}

//////////////////////////////////////////////////
void SimbodyHingeJoint::SetMaxForce(int /*_index*/, double _t)
{
  gzerr << "Not implemented...\n";
}

//////////////////////////////////////////////////
double SimbodyHingeJoint::GetMaxForce(int /*_index*/)
{
  gzerr << "Not implemented...\n";
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
      if (this->limitForce)
        this->limitForce->setBounds(
          this->simbodyPhysics->integ->updAdvancedState(),
          this->limitForce->getLowerBound(
            this->simbodyPhysics->integ->updAdvancedState()),
          _angle.Radian()
          );
      else
        gzdbg << "limitForce for joint ["
              << this->GetName() << "] is NULL, new model added?\n";
    }
    else
    {
      gzerr << "SetHighStop: State not initialized.\n";
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
      if (this->limitForce)
        this->limitForce->setBounds(
          this->simbodyPhysics->integ->updAdvancedState(),
          _angle.Radian(),
          this->limitForce->getUpperBound(
            this->simbodyPhysics->integ->updAdvancedState())
          );
      else
        gzdbg << "limitForce for joint ["
              << this->GetName() << "] is NULL, new model added?\n";
    }
    else
    {
      gzerr << "SetLowStop: State not initialized.\n";
    }
  }
  else
    gzerr << "SetLowStop: index out of bounds.\n";
}

//////////////////////////////////////////////////
math::Angle SimbodyHingeJoint::GetHighStop(int /*_index*/)
{
  math::Angle result;
  // gzerr << "Not implemented...\n";
  return result;
}

//////////////////////////////////////////////////
math::Angle SimbodyHingeJoint::GetLowStop(int /*_index*/)
{
  math::Angle result;
  // gzerr << "Not implemented...\n";
  return result;
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
    gzerr << "index out of bound\n";
    return math::Vector3();
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
      gzwarn << "simbody not yet initialized\n";
      return math::Angle();
    }
  }
  else
  {
    gzerr << "index out of bound\n";
    return math::Angle();
  }
}
