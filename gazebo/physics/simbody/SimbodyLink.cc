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
/* Desc: Link class
 * Author: Nate Koenig
 * Date: 13 Feb 2006
 */

#include <boost/thread.hpp>

#include "gazebo/common/Console.hh"
#include "gazebo/common/Exception.hh"

#include "gazebo/physics/World.hh"

#include "gazebo/physics/simbody/SimbodyCollision.hh"
#include "gazebo/physics/simbody/SimbodyPhysics.hh"
#include "gazebo/physics/simbody/SimbodyLink.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
SimbodyLink::SimbodyLink(EntityPtr _parent)
    : Link(_parent)
{
  this->mustBeBaseLink = false;
  this->physicsInitialized = false;
  this->gravityMode = false;
  this->staticLinkDirty = false;
  this->staticLink = false;
  this->simbodyPhysics.reset();
}

//////////////////////////////////////////////////
SimbodyLink::~SimbodyLink()
{
}

//////////////////////////////////////////////////
void SimbodyLink::Load(sdf::ElementPtr _sdf)
{
  this->simbodyPhysics = boost::dynamic_pointer_cast<SimbodyPhysics>(
      this->GetWorld()->GetPhysicsEngine());

  if (this->simbodyPhysics == NULL)
    gzthrow("Not using the simbody physics engine");

  if (_sdf->HasElement("must_be_base_link"))
    this->mustBeBaseLink = _sdf->Get<bool>("must_be_base_link");

  Link::Load(_sdf);
}

//////////////////////////////////////////////////
void SimbodyLink::Init()
{
  Link::Init();

  math::Vector3 cogVec = this->inertial->GetCoG();

  // Set the initial pose of the body

  for (Base_V::iterator iter = this->children.begin();
       iter != this->children.end(); ++iter)
  {
    if ((*iter)->HasType(Base::COLLISION))
    {
      SimbodyCollisionPtr collision;
      collision = boost::static_pointer_cast<SimbodyCollision>(*iter);

      math::Pose relativePose = collision->GetRelativePose();
      relativePose.pos -= cogVec;
    }
  }

  // Create a construction info object
  // Create the new rigid body

  // lock or unlock the link if requested by user
  this->staticLinkConnection = event::Events::ConnectWorldUpdateEnd(
    boost::bind(&SimbodyLink::ProcessSetLinkStatic, this));
}

//////////////////////////////////////////////////
void SimbodyLink::Fini()
{
  event::Events::DisconnectWorldUpdateEnd(this->staticLinkConnection);
  Link::Fini();
}

//////////////////////////////////////////////////
void SimbodyLink::SetGravityMode(bool _mode)
{
  this->sdf->GetElement("gravity")->Set(_mode);
  this->gravityMode = _mode;
  if (this->physicsInitialized)
  {
    this->simbodyPhysics->gravity.setBodyIsExcluded(
      this->simbodyPhysics->integ->updAdvancedState(),
      this->masterMobod, !_mode);
  }
  else
  {
    gzlog << "SetGravityMode [" << _mode
          << "], but physics not initialized, caching\n";
  }
}

//////////////////////////////////////////////////
bool SimbodyLink::GetGravityMode() const
{
  if (this->physicsInitialized)
  {
    return this->simbodyPhysics->gravity.getBodyIsExcluded(
      this->simbodyPhysics->integ->getState(), this->masterMobod);
  }
  else
  {
    gzlog << "GetGravityMode [" << this->gravityMode
          << "], but physics not initialized, returning cached value\n";
    return this->gravityMode;
  }
}

//////////////////////////////////////////////////
void SimbodyLink::SetSelfCollide(bool /*_collide*/)
{
}

//////////////////////////////////////////////////
/*void SimbodyLink::AttachCollision(Collision *_collision)
{
  Link::AttachCollision(_collision);

  SimbodyCollision *bcollision = dynamic_cast<SimbodyCollision*>(_collision);

  if (_collision == NULL)
    gzthrow("requires SimbodyCollision");

  math::Pose relativePose = _collision->GetRelativePose();
}
  */

//////////////////////////////////////////////////
/// changed
void SimbodyLink::OnPoseChange()
{
  Link::OnPoseChange();

  if (!this->simbodyPhysics->simbodyPhysicsInitialized)
    return;

  /// \TODO: limited functionality for now.
  /// Setting 6 dof pose of a link works in simbody only if
  /// the inboard joint is a free joint to the ground for now.

  if (this->masterMobod.isEmptyHandle())
    return;

  if (!this->masterMobod.isGround() &&
      this->masterMobod.getParentMobilizedBody().isGround())
  {
    this->masterMobod.setQToFitTransform(
       this->simbodyPhysics->integ->updAdvancedState(),
       SimbodyPhysics::Pose2Transform(this->GetWorldPose()));
  }
  // else
  //   gzdbg << "Joint [" << this->GetScopedName()
  //         << "] P[" << this->GetWorldPose() << "]\n";


  /*
  math::Pose pose = this->GetWorldPose();

  */
}

//////////////////////////////////////////////////
void SimbodyLink::SaveSimbodyState(const SimTK::State &_state)
{
  // skip if not a free joint, state is saved in SimbodyJoint::mobod
  if (!this->masterMobod.isEmptyHandle() &&
      SimTK::MobilizedBody::Free::isInstanceOf(this->masterMobod))
  {
    if (this->simbodyQ.empty())
      this->simbodyQ.resize(this->masterMobod.getNumQ(_state));

    if (this->simbodyU.empty())
      this->simbodyU.resize(this->masterMobod.getNumU(_state));

    for (unsigned int i = 0; i < this->simbodyQ.size(); ++i)
      this->simbodyQ[i] = this->masterMobod.getOneQ(_state, i);

    for (unsigned int i = 0; i < this->simbodyU.size(); ++i)
      this->simbodyU[i] = this->masterMobod.getOneU(_state, i);
  }
  else
  {
    // gzerr << "debug: joint name: " << this->GetScopedName() << "\n";
  }
}

//////////////////////////////////////////////////
void SimbodyLink::RestoreSimbodyState(SimTK::State &_state)
{
  // skip if not a free joint, state is restored by SimbodyJoint::mobod
  if (!this->masterMobod.isEmptyHandle() &&
      SimTK::MobilizedBody::Free::isInstanceOf(this->masterMobod))
  {
    for (unsigned int i = 0; i < this->simbodyQ.size(); ++i)
      this->masterMobod.setOneQ(_state, i, this->simbodyQ[i]);

    for (unsigned int i = 0; i < this->simbodyU.size(); ++i)
      this->masterMobod.setOneU(_state, i, this->simbodyU[i]);
  }
  else
  {
    // gzerr << "debug: joint name: " << this->GetScopedName() << "\n";
  }
}

//////////////////////////////////////////////////
void SimbodyLink::SetLinkStatic(bool _static)
{
  if (!this->staticLinkDirty)
  {
    this->staticLinkDirty = true;
    this->staticLink = _static;
  }
  else
    gzerr << "Trying to SetLinkStatic before last setting is processed.\n";
}


//////////////////////////////////////////////////
void SimbodyLink::ProcessSetLinkStatic()
{
  if (this->masterMobod.isEmptyHandle())
    return;

  // check if inboard body is ground
  if (this->staticLinkDirty &&
      this->masterMobod.getParentMobilizedBody().isGround())
  {
    if (this->staticLink)
      this->masterMobod.lock(
       this->simbodyPhysics->integ->updAdvancedState());
    else
      this->masterMobod.unlock(
       this->simbodyPhysics->integ->updAdvancedState());

    // re-realize
    this->simbodyPhysics->system.realize(
      this->simbodyPhysics->integ->getAdvancedState(), SimTK::Stage::Velocity);
  }
  else
  {
    // gzerr << "debug: joint name: " << this->GetScopedName() << "\n";
  }

  this->staticLinkDirty = false;
}

//////////////////////////////////////////////////
void SimbodyLink::SetEnabled(bool /*_enable*/) const
{
}

//////////////////////////////////////////////////
void SimbodyLink::SetLinearVel(const math::Vector3 & /*_vel*/)
{
}

//////////////////////////////////////////////////
math::Vector3 SimbodyLink::GetWorldLinearVel(
  const math::Vector3& _offset) const
{
  SimTK::Vec3 station = SimbodyPhysics::Vector3ToVec3(_offset);
  math::Vector3 v;

  if (this->simbodyPhysics->simbodyPhysicsInitialized)
  {
    // lock physics update mutex to ensure thread safety
    boost::recursive_mutex::scoped_lock lock(
      *this->world->GetPhysicsEngine()->GetPhysicsUpdateMutex());
    v = SimbodyPhysics::Vec3ToVector3(
      this->masterMobod.findStationVelocityInGround(
      this->simbodyPhysics->integ->getState(), station));
  }
  else
    gzwarn << "SimbodyLink::GetWorldLinearVel: simbody physics"
           << " not yet initialized\n";

  return v;
}

//////////////////////////////////////////////////
math::Vector3 SimbodyLink::GetWorldLinearVel(
  const math::Vector3 &_offset,
  const math::Quaternion &_q) const
{
  math::Vector3 v;

  if (this->simbodyPhysics->simbodyPhysicsInitialized)
  {
    SimTK::Rotation R_WF(SimbodyPhysics::QuadToQuad(_q));
    SimTK::Vec3 p_F(SimbodyPhysics::Vector3ToVec3(_offset));
    SimTK::Vec3 p_W(R_WF * p_F);

    // lock physics update mutex to ensure thread safety
    boost::recursive_mutex::scoped_lock lock(
      *this->world->GetPhysicsEngine()->GetPhysicsUpdateMutex());

    const SimTK::Rotation &R_WL = this->masterMobod.getBodyRotation(
      this->simbodyPhysics->integ->getState());
    SimTK::Vec3 p_B(~R_WL * p_W);
    v = SimbodyPhysics::Vec3ToVector3(
      this->masterMobod.findStationVelocityInGround(
      this->simbodyPhysics->integ->getState(), p_B));
  }
  else
    gzwarn << "SimbodyLink::GetWorldLinearVel: simbody physics"
           << " not yet initialized\n";

  return v;
}

//////////////////////////////////////////////////
math::Vector3 SimbodyLink::GetWorldCoGLinearVel() const
{
  math::Vector3 v;

  if (this->simbodyPhysics->simbodyPhysicsInitialized)
  {
    // lock physics update mutex to ensure thread safety
    boost::recursive_mutex::scoped_lock lock(
      *this->world->GetPhysicsEngine()->GetPhysicsUpdateMutex());
    SimTK::Vec3 station = this->masterMobod.getBodyMassCenterStation(
       this->simbodyPhysics->integ->getState());
    v = SimbodyPhysics::Vec3ToVector3(
      this->masterMobod.findStationVelocityInGround(
      this->simbodyPhysics->integ->getState(), station));
  }
  else
    gzwarn << "SimbodyLink::GetWorldCoGLinearVel: simbody physics"
           << " not yet initialized\n";

  return v;
}

//////////////////////////////////////////////////
void SimbodyLink::SetAngularVel(const math::Vector3 &/*_vel*/)
{
}

//////////////////////////////////////////////////
math::Vector3 SimbodyLink::GetWorldAngularVel() const
{
  // lock physics update mutex to ensure thread safety
  boost::recursive_mutex::scoped_lock lock(
    *this->world->GetPhysicsEngine()->GetPhysicsUpdateMutex());
  SimTK::Vec3 w =
    this->masterMobod.getBodyAngularVelocity(
    this->simbodyPhysics->integ->getState());
  return SimbodyPhysics::Vec3ToVector3(w);
}

//////////////////////////////////////////////////
void SimbodyLink::SetForce(const math::Vector3 &_force)
{
  SimTK::Vec3 f(SimbodyPhysics::Vector3ToVec3(_force));

  this->simbodyPhysics->discreteForces.setOneBodyForce(
    this->simbodyPhysics->integ->updAdvancedState(),
    this->masterMobod, SimTK::SpatialVec(SimTK::Vec3(0), f));
}

//////////////////////////////////////////////////
math::Vector3 SimbodyLink::GetWorldForce() const
{
  return math::Vector3();
}

//////////////////////////////////////////////////
void SimbodyLink::SetTorque(const math::Vector3 &/*_torque*/)
{
}

//////////////////////////////////////////////////
math::Vector3 SimbodyLink::GetWorldTorque() const
{
  return math::Vector3();
}

//////////////////////////////////////////////////
void SimbodyLink::SetLinearDamping(double /*_damping*/)
{
}

//////////////////////////////////////////////////
void SimbodyLink::SetAngularDamping(double /*_damping*/)
{
}

/////////////////////////////////////////////////
void SimbodyLink::AddForce(const math::Vector3 &/*_force*/)
{
}

/////////////////////////////////////////////////
void SimbodyLink::AddRelativeForce(const math::Vector3 &/*_force*/)
{
}

/////////////////////////////////////////////////
void SimbodyLink::AddForceAtWorldPosition(const math::Vector3 &/*_force*/,
                                         const math::Vector3 &/*_pos*/)
{
}

/////////////////////////////////////////////////
void SimbodyLink::AddForceAtRelativePosition(const math::Vector3 &/*_force*/,
                  const math::Vector3 &/*_relpos*/)
{
}

/////////////////////////////////////////////////
void SimbodyLink::AddTorque(const math::Vector3 &/*_torque*/)
{
}

/////////////////////////////////////////////////
void SimbodyLink::AddRelativeTorque(const math::Vector3 &/*_torque*/)
{
}

/////////////////////////////////////////////////
void SimbodyLink::SetAutoDisable(bool /*_disable*/)
{
}

/////////////////////////////////////////////////
SimTK::MassProperties SimbodyLink::GetMassProperties() const
{
  gzlog << "SimbodyLink::GetMassProperties for ["
        << this->GetScopedName() << "]\n";

  if (!this->IsStatic())
  {
    const SimTK::Real mass = this->inertial->GetMass();
    SimTK::Transform X_LI = physics::SimbodyPhysics::Pose2Transform(
      this->inertial->GetPose());
    const SimTK::Vec3 &com_L = X_LI.p();  // vector from Lo to com, exp. in L

    if (math::equal(mass, 0.0))
      return SimTK::MassProperties(mass, com_L, SimTK::UnitInertia(1, 1, 1));

    // Get mass-weighted central inertia, expressed in I frame.
    SimTK::Inertia Ic_I(this->inertial->GetIXX(),
                 this->inertial->GetIYY(),
                 this->inertial->GetIZZ(),
                 this->inertial->GetIXY(),
                 this->inertial->GetIXZ(),
                 this->inertial->GetIYZ());
    // Re-express the central inertia from the I frame to the L frame.
    SimTK::Inertia Ic_L = Ic_I.reexpress(~X_LI.R());  // Ic_L=R_LI*Ic_I*R_IL
    // Shift to L frame origin.
    SimTK::Inertia Io_L = Ic_L.shiftFromMassCenter(-com_L, mass);
    return SimTK::MassProperties(mass, com_L, Io_L);  // convert to unit inertia
  }
  else
  {
    gzerr << "inertial block no specified, using unit mass properties\n";
    return SimTK::MassProperties(1, SimTK::Vec3(0),
      SimTK::UnitInertia(0.1, 0.1, 0.1));
  }
}

/////////////////////////////////////////////////
// When a link is broken into several fragments (master and slaves), they
// share the mass equally. Given the number of fragments, this returns the
// appropriate mass properties to use for each fragment. Per Simbody's
// convention, COM is measured from, and inertia taken about, the link
// origin and both are expressed in the link frame.
SimTK::MassProperties SimbodyLink::GetEffectiveMassProps(
  int _numFragments) const
{
    SimTK::MassProperties massProps = this->GetMassProperties();
    assert(_numFragments > 0);  // must be at least 1 for the master
    return SimTK::MassProperties(massProps.getMass()/_numFragments,
                          massProps.getMassCenter(),
                          massProps.getUnitInertia());
}

/////////////////////////////////////////////////
bool SimbodyLink::GetEnabled() const
{
  return true;
}

/////////////////////////////////////////////////
void SimbodyLink::SetDirtyPose(const math::Pose &_pose)
{
  this->dirtyPose = _pose;
}
