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
#include "gazebo/common/Assert.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Events.hh"

#include "gazebo/physics/World.hh"

#include "gazebo/physics/simbody/SimbodyCollision.hh"
#include "gazebo/physics/simbody/SimbodyPhysics.hh"
#include "gazebo/physics/simbody/SimbodyLinkPrivate.hh"
#include "gazebo/physics/simbody/SimbodyLink.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
SimbodyLink::SimbodyLink(EntityPtr _parent)
: Link(*new SimbodyLinkPrivate, _parent),
  simbodyLinkDPtr(static_cast<SimbodyLinkPrivate*>(this->linkDPtr))
{
  this->simbodyLinkDPtr->mustBeBaseLink = false;
  this->simbodyLinkDPtr->physicsInitialized = false;
  this->simbodyLinkDPtr->gravityMode = false;
  this->simbodyLinkDPtr->staticLinkDirty = false;
  this->simbodyLinkDPtr->staticLink = false;
  this->simbodyLinkDPtr->simbodyPhysics.reset();
  this->simbodyLinkDPtr->gravityModeDirty = false;
}

//////////////////////////////////////////////////
SimbodyLink::~SimbodyLink()
{
}

//////////////////////////////////////////////////
void SimbodyLink::Load(sdf::ElementPtr _sdf)
{
  this->simbodyLinkDPtr->simbodyPhysics =
    std::dynamic_pointer_cast<SimbodyPhysics>(
      this->World()->Physics());

  if (this->simbodyLinkDPtr->simbodyPhysics == NULL)
  {
    gzerr << "Not using the simbody physics engine\n";
    return;
  }

  if (_sdf->HasElement("must_be_base_link"))
  {
    this->simbodyLinkDPtr->mustBeBaseLink =
      _sdf->Get<bool>("must_be_base_link");
  }

  this->SetKinematic(_sdf->Get<bool>("kinematic"));
  this->SetGravityMode(_sdf->Get<bool>("gravity"));

  Link::Load(_sdf);
}

//////////////////////////////////////////////////
void SimbodyLink::Init()
{
  /// \TODO: implement following
  // this->SetLinearDamping(this->GetLinearDamping());
  // this->SetAngularDamping(this->GetAngularDamping());

  Link::Init();

  ignition::math::Vector3d cogVec = this->simbodyLinkDPtr->inertial.CoG();

  // Set the initial pose of the body

  for (Base_V::iterator iter = this->simbodyLinkDPtr->children.begin();
       iter != this->simbodyLinkDPtr->children.end(); ++iter)
  {
    if ((*iter)->HasType(Base::COLLISION))
    {
      SimbodyCollisionPtr collision;
      collision = std::static_pointer_cast<SimbodyCollision>(*iter);

      ignition::math::Pose3d relativePose = collision->RelativePose();
      relativePose.Pos() -= cogVec;
    }
  }

  // Create a construction info object
  // Create the new rigid body

  // change link's gravity mode if requested by user
  this->simbodyLinkDPtr->gravityModeConnection =
    event::Events::ConnectWorldUpdateBegin(
        std::bind(&SimbodyLink::ProcessSetGravityMode, this));

  // lock or unlock the link if requested by user
  this->simbodyLinkDPtr->staticLinkConnection =
    event::Events::ConnectWorldUpdateEnd(
        std::bind(&SimbodyLink::ProcessSetLinkStatic, this));
}

//////////////////////////////////////////////////
void SimbodyLink::Fini()
{
  event::Events::DisconnectWorldUpdateBegin(
      this->simbodyLinkDPtr->gravityModeConnection);
  event::Events::DisconnectWorldUpdateEnd(
      this->simbodyLinkDPtr->staticLinkConnection);
  Link::Fini();
}

/////////////////////////////////////////////////////////////////////
void SimbodyLink::UpdateMass()
{
}

//////////////////////////////////////////////////
void SimbodyLink::SetGravityMode(bool _mode)
{
  if (!this->simbodyLinkDPtr->gravityModeDirty)
  {
    this->simbodyLinkDPtr->gravityModeDirty = true;
    this->simbodyLinkDPtr->gravityMode = _mode;
  }
  else
    gzerr << "Trying to SetGravityMode for link [" << this->ScopedName()
          << "] before last setting is processed.\n";
}

//////////////////////////////////////////////////
void SimbodyLink::ProcessSetGravityMode()
{
  if (this->simbodyLinkDPtr->gravityModeDirty)
  {
    if (this->simbodyLinkDPtr->physicsInitialized)
    {
      this->simbodyLinkDPtr->sdf->GetElement(
          "gravity")->Set(this->simbodyLinkDPtr->gravityMode);
      this->simbodyLinkDPtr->simbodyPhysics->SimbodyGravity().setBodyIsExcluded(
          this->simbodyLinkDPtr->simbodyPhysics->Integ()->updAdvancedState(),
          this->simbodyLinkDPtr->masterMobod,
          !this->simbodyLinkDPtr->gravityMode);

      // realize system after changing gravity mode
      this->simbodyLinkDPtr->simbodyPhysics->System().realize(
        this->simbodyLinkDPtr->simbodyPhysics->Integ()->getState(),
        SimTK::Stage::Velocity);

      this->simbodyLinkDPtr->gravityModeDirty = false;
    }
    else
    {
      gzlog << "SetGravityMode [" << this->simbodyLinkDPtr->gravityMode
            << "], but physics not initialized, caching\n";
    }
  }
}

//////////////////////////////////////////////////
bool SimbodyLink::GravityMode() const
{
  if (this->simbodyLinkDPtr->physicsInitialized)
  {
    return this->simbodyLinkDPtr->simbodyPhysics->SimbodyGravity(
        ).getBodyIsExcluded(
      this->simbodyLinkDPtr->simbodyPhysics->Integ()->getState(),
      this->simbodyLinkDPtr->masterMobod);
  }
  else
  {
    gzlog << "GravityMode [" << this->simbodyLinkDPtr->gravityMode
          << "], but physics not initialized, returning cached value\n";
    return this->simbodyLinkDPtr->gravityMode;
  }
}

//////////////////////////////////////////////////
void SimbodyLink::SetSelfCollide(bool /*_collide*/)
{
}

//////////////////////////////////////////////////
// void SimbodyLink::AttachCollision(Collision *_collision)
// {
//   Link::AttachCollision(_collision);
//
//   SimbodyCollision *bcollision = dynamic_cast<SimbodyCollision*>(_collision);
//
//   if (_collision == NULL)
//   {
//     gzerr << "requires SimbodyCollision\n";
//     return;
//   }
//
//   ignition::math::Pose relativePose = _collision->RelativePose();
// }

//////////////////////////////////////////////////
/// changed
void SimbodyLink::OnPoseChange()
{
  Link::OnPoseChange();

  if (!this->simbodyLinkDPtr->simbodyPhysics->PhysicsInitialized())
    return;

  if (this->simbodyLinkDPtr->masterMobod.isEmptyHandle())
    return;

  // debug
  // gzerr << "original: [" << SimbodyPhysics::Transform2Pose(
  //   this->simbodyLinkDPtr->masterMobod.getBodyTransform(
  //   this->simbodyLinkDPtr->simbodyPhysics->Integ()->updAdvancedState()))
  //       << "]\n";

  if (!this->simbodyLinkDPtr->masterMobod.isGround())
  {
    if (this->simbodyLinkDPtr->masterMobod.getParentMobilizedBody().isGround())
    {
      /// If parent is ground:
      /// Setting 6 dof pose of a link works in simbody only if
      /// the inboard joint is a free joint to the ground for now.
      this->simbodyLinkDPtr->masterMobod.setQToFitTransform(
         this->simbodyLinkDPtr->simbodyPhysics->Integ()->updAdvancedState(),
         SimbodyPhysics::Pose2Transform(this->WorldPose()));
    }
    else
    {
      gzerr << "SetWorldPose (OnPoseChange) for child links need testing.\n";
      /*
      /// If the inboard joint is not free, simbody tries to project
      /// target pose into available DOF's.
      /// But first convert to relative pose to parent mobod.
      ignition::math::Pose parentPose = SimbodyPhysics::Transform2Pose(
        this->simbodyLinkDPtr->masterMobod.getBodyTransform(
        this->simbodyLinkDPtr->simbodyPhysics->Integ()->updAdvancedState()));
      ignition::math::Pose relPose = this->WorldPose() - parentPose;
      this->simbodyLinkDPtr->masterMobod.setQToFitTransform(
         this->simbodyLinkDPtr->simbodyPhysics->Integ()->updAdvancedState(),
         SimbodyPhysics::Pose2Transform(relPose));
      */
    }
    // realize system after updating Q's
    this->simbodyLinkDPtr->simbodyPhysics->System().realize(
        this->simbodyLinkDPtr->simbodyPhysics->Integ()->getState(),
        SimTK::Stage::Position);
  }
}

//////////////////////////////////////////////////
void SimbodyLink::SaveSimbodyState(const SimTK::State &_state)
{
  // skip if not a free joint, state is saved in SimbodyJoint::mobod
  if (!this->simbodyLinkDPtr->masterMobod.isEmptyHandle() &&
      SimTK::MobilizedBody::Free::isInstanceOf(
        this->simbodyLinkDPtr->masterMobod))
  {
    if (this->simbodyLinkDPtr->simbodyQ.empty())
    {
      this->simbodyLinkDPtr->simbodyQ.resize(
          this->simbodyLinkDPtr->masterMobod.getNumQ(_state));
    }

    if (this->simbodyLinkDPtr->simbodyU.empty())
    {
      this->simbodyLinkDPtr->simbodyU.resize(
          this->simbodyLinkDPtr->masterMobod.getNumU(_state));
    }

    for (unsigned int i = 0; i < this->simbodyLinkDPtr->simbodyQ.size(); ++i)
    {
      this->simbodyLinkDPtr->simbodyQ[i] =
        this->simbodyLinkDPtr->masterMobod.getOneQ(_state, i);
    }

    for (unsigned int i = 0; i < this->simbodyLinkDPtr->simbodyU.size(); ++i)
    {
      this->simbodyLinkDPtr->simbodyU[i] =
        this->simbodyLinkDPtr->masterMobod.getOneU(_state, i);
    }
  }
  else
  {
    // gzerr << "debug: joint name: " << this->ScopedName() << "\n";
  }
}

//////////////////////////////////////////////////
void SimbodyLink::RestoreSimbodyState(SimTK::State &_state)
{
  // skip if not a free joint, state is restored by SimbodyJoint::mobod
  if (!this->simbodyLinkDPtr->masterMobod.isEmptyHandle() &&
      SimTK::MobilizedBody::Free::isInstanceOf(
        this->simbodyLinkDPtr->masterMobod))
  {
    for (unsigned int i = 0; i < this->simbodyLinkDPtr->simbodyQ.size(); ++i)
    {
      this->simbodyLinkDPtr->masterMobod.setOneQ(_state, i,
          this->simbodyLinkDPtr->simbodyQ[i]);
    }

    for (unsigned int i = 0; i < this->simbodyLinkDPtr->simbodyU.size(); ++i)
    {
      this->simbodyLinkDPtr->masterMobod.setOneU(_state, i,
          this->simbodyLinkDPtr->simbodyU[i]);
    }
  }
  else
  {
    // gzerr << "debug: joint name: " << this->ScopedName() << "\n";
  }
}

//////////////////////////////////////////////////
void SimbodyLink::SetLinkStatic(const bool _static)
{
  if (!this->simbodyLinkDPtr->staticLinkDirty)
  {
    this->simbodyLinkDPtr->staticLinkDirty = true;
    this->simbodyLinkDPtr->staticLink = _static;
  }
  else
    gzerr << "Trying to SetLinkStatic before last setting is processed.\n";
}


//////////////////////////////////////////////////
void SimbodyLink::ProcessSetLinkStatic()
{
  if (this->simbodyLinkDPtr->masterMobod.isEmptyHandle())
    return;

  // check if inboard body is ground
  if (this->simbodyLinkDPtr->staticLinkDirty &&
      this->simbodyLinkDPtr->masterMobod.getParentMobilizedBody().isGround())
  {
    if (this->simbodyLinkDPtr->staticLink)
      this->simbodyLinkDPtr->masterMobod.lock(
       this->simbodyLinkDPtr->simbodyPhysics->Integ()->updAdvancedState());
    else
      this->simbodyLinkDPtr->masterMobod.unlock(
       this->simbodyLinkDPtr->simbodyPhysics->Integ()->updAdvancedState());

    // re-realize
    this->simbodyLinkDPtr->simbodyPhysics->System().realize(
      this->simbodyLinkDPtr->simbodyPhysics->Integ()->getAdvancedState(),
      SimTK::Stage::Velocity);
  }
  else
  {
    // gzerr << "debug: joint name: " << this->ScopedName() << "\n";
  }

  this->simbodyLinkDPtr->staticLinkDirty = false;
}

//////////////////////////////////////////////////
void SimbodyLink::SetEnabled(const bool /*_enable*/) const
{
}

//////////////////////////////////////////////////
void SimbodyLink::SetLinearVel(const ignition::math::Vector3d & _vel)
{
  this->simbodyLinkDPtr->masterMobod.setUToFitLinearVelocity(
    this->simbodyLinkDPtr->simbodyPhysics->Integ()->updAdvancedState(),
    SimbodyPhysics::Vector3ToVec3(_vel));

  this->simbodyLinkDPtr->simbodyPhysics->System().realize(
    this->simbodyLinkDPtr->simbodyPhysics->Integ()->getAdvancedState(),
    SimTK::Stage::Velocity);
}

//////////////////////////////////////////////////
ignition::math::Vector3d SimbodyLink::WorldLinearVel(
  const ignition::math::Vector3d &_offset) const
{
  SimTK::Vec3 station = SimbodyPhysics::Vector3ToVec3(_offset);
  ignition::math::Vector3d v;

  if (this->simbodyLinkDPtr->simbodyPhysics->PhysicsInitialized())
  {
    // lock physics update mutex to ensure thread safety
    std::lock_guard<std::recursive_mutex> lock(
      this->simbodyLinkDPtr->world->Physics()->PhysicsUpdateMutex());

    v = SimbodyPhysics::Vec3ToVector3Ign(
      this->simbodyLinkDPtr->masterMobod.findStationVelocityInGround(
      this->simbodyLinkDPtr->simbodyPhysics->Integ()->getState(), station));
  }
  else
  {
    gzwarn << "SimbodyLink::WorldLinearVel: simbody physics"
           << " not yet initialized\n";
  }

  return v;
}

//////////////////////////////////////////////////
ignition::math::Vector3d SimbodyLink::WorldLinearVel(
  const ignition::math::Vector3d &_offset,
  const ignition::math::Quaterniond &_q) const
{
  ignition::math::Vector3d v;

  if (this->simbodyLinkDPtr->simbodyPhysics->PhysicsInitialized())
  {
    SimTK::Rotation rwf(SimbodyPhysics::QuadToQuad(_q));
    SimTK::Vec3 pf(SimbodyPhysics::Vector3ToVec3(_offset));
    SimTK::Vec3 pw(rwf * pf);

    // lock physics update mutex to ensure thread safety
    std::lock_guard<std::recursive_mutex> lock(
      this->simbodyLinkDPtr->world->Physics()->PhysicsUpdateMutex());

    const SimTK::Rotation &rwl =
      this->simbodyLinkDPtr->masterMobod.getBodyRotation(
          this->simbodyLinkDPtr->simbodyPhysics->Integ()->getState());

    SimTK::Vec3 p_B(~rwl * pw);
    v = SimbodyPhysics::Vec3ToVector3Ign(
      this->simbodyLinkDPtr->masterMobod.findStationVelocityInGround(
      this->simbodyLinkDPtr->simbodyPhysics->Integ()->getState(), p_B));
  }
  else
  {
    gzwarn << "SimbodyLink::WorldLinearVel: simbody physics"
           << " not yet initialized\n";
  }

  return v;
}

//////////////////////////////////////////////////
ignition::math::Vector3d SimbodyLink::WorldCoGLinearVel() const
{
  ignition::math::Vector3d v;

  if (this->simbodyLinkDPtr->simbodyPhysics->PhysicsInitialized())
  {
    // lock physics update mutex to ensure thread safety
    std::lock_guard<std::recursive_mutex> lock(
      this->simbodyLinkDPtr->world->Physics()->PhysicsUpdateMutex());

    SimTK::Vec3 station =
      this->simbodyLinkDPtr->masterMobod.getBodyMassCenterStation(
          this->simbodyLinkDPtr->simbodyPhysics->Integ()->getState());

    v = SimbodyPhysics::Vec3ToVector3Ign(
        this->simbodyLinkDPtr->masterMobod.findStationVelocityInGround(
          this->simbodyLinkDPtr->simbodyPhysics->Integ()->getState(), station));
  }
  else
  {
    gzwarn << "SimbodyLink::WorldCoGLinearVel: simbody physics"
           << " not yet initialized\n";
  }

  return v;
}

//////////////////////////////////////////////////
void SimbodyLink::SetAngularVel(const ignition::math::Vector3d &_vel)
{
  this->simbodyLinkDPtr->masterMobod.setUToFitAngularVelocity(
      this->simbodyLinkDPtr->simbodyPhysics->Integ()->updAdvancedState(),
      SimbodyPhysics::Vector3ToVec3(_vel));

  this->simbodyLinkDPtr->simbodyPhysics->System().realize(
      this->simbodyLinkDPtr->simbodyPhysics->Integ()->getAdvancedState(),
      SimTK::Stage::Velocity);
}

//////////////////////////////////////////////////
ignition::math::Vector3d SimbodyLink::WorldAngularVel() const
{
  // lock physics update mutex to ensure thread safety
  std::lock_guard<std::recursive_mutex> lock(
    this->simbodyLinkDPtr->world->Physics()->PhysicsUpdateMutex());

  SimTK::Vec3 w =
    this->simbodyLinkDPtr->masterMobod.getBodyAngularVelocity(
    this->simbodyLinkDPtr->simbodyPhysics->Integ()->getState());

  return SimbodyPhysics::Vec3ToVector3Ign(w);
}

//////////////////////////////////////////////////
void SimbodyLink::SetForce(const ignition::math::Vector3d &_force)
{
  SimTK::Vec3 f(SimbodyPhysics::Vector3ToVec3(_force));

  this->simbodyLinkDPtr->simbodyPhysics->DiscreteForces().setOneBodyForce(
      this->simbodyLinkDPtr->simbodyPhysics->Integ()->updAdvancedState(),
      this->simbodyLinkDPtr->masterMobod, SimTK::SpatialVec(SimTK::Vec3(0), f));
}

//////////////////////////////////////////////////
ignition::math::Vector3d SimbodyLink::WorldForce() const
{
  SimTK::SpatialVec sv =
    this->simbodyLinkDPtr->simbodyPhysics->DiscreteForces().getOneBodyForce(
        this->simbodyLinkDPtr->simbodyPhysics->Integ()->getState(),
        this->simbodyLinkDPtr->masterMobod);

  // get translational component
  SimTK::Vec3 f = sv[1];

  return SimbodyPhysics::Vec3ToVector3Ign(f);
}

//////////////////////////////////////////////////
void SimbodyLink::SetTorque(const ignition::math::Vector3d &_torque)
{
  SimTK::Vec3 t(SimbodyPhysics::Vector3ToVec3(_torque));

  this->simbodyLinkDPtr->simbodyPhysics->DiscreteForces().setOneBodyForce(
      this->simbodyLinkDPtr->simbodyPhysics->Integ()->updAdvancedState(),
      this->simbodyLinkDPtr->masterMobod, SimTK::SpatialVec(t, SimTK::Vec3(0)));
}

//////////////////////////////////////////////////
ignition::math::Vector3d SimbodyLink::WorldTorque() const
{
  SimTK::SpatialVec sv =
    this->simbodyLinkDPtr->simbodyPhysics->DiscreteForces().getOneBodyForce(
        this->simbodyLinkDPtr->simbodyPhysics->Integ()->getState(),
        this->simbodyLinkDPtr->masterMobod);

  // get rotational component
  SimTK::Vec3 t = sv[0];

  return SimbodyPhysics::Vec3ToVector3Ign(t);
}

//////////////////////////////////////////////////
void SimbodyLink::SetLinearDamping(const double /*_damping*/)
{
  gzerr << "Not implemented.\n";
}

//////////////////////////////////////////////////
void SimbodyLink::SetAngularDamping(const double /*_damping*/)
{
  gzerr << "Not implemented.\n";
}

/////////////////////////////////////////////////
void SimbodyLink::AddForce(const ignition::math::Vector3d &_force)
{
  SimTK::Vec3 f(SimbodyPhysics::Vector3ToVec3(_force));

  this->simbodyLinkDPtr->simbodyPhysics->DiscreteForces().addForceToBodyPoint(
      this->simbodyLinkDPtr->simbodyPhysics->Integ()->updAdvancedState(),
      this->simbodyLinkDPtr->masterMobod,
      SimTK::Vec3(0), f);
}

/////////////////////////////////////////////////
void SimbodyLink::AddRelativeForce(const ignition::math::Vector3d &/*_force*/)
{
  gzerr << "Not implemented.\n";
}

/////////////////////////////////////////////////
void SimbodyLink::AddForceAtWorldPosition(
    const ignition::math::Vector3d &/*_force*/,
    const ignition::math::Vector3d &/*_pos*/)
{
  gzerr << "Not implemented.\n";
}

/////////////////////////////////////////////////
void SimbodyLink::AddForceAtRelativePosition(
    const ignition::math::Vector3d &/*_force*/,
    const ignition::math::Vector3d &/*_relpos*/)
{
  gzerr << "Not implemented.\n";
}

//////////////////////////////////////////////////
void SimbodyLink::AddLinkForce(const ignition::math::Vector3d &/*_force*/,
    const ignition::math::Vector3d &/*_offset*/)
{
  gzlog << "SimbodyLink::AddLinkForce not yet implemented (issue #1478)."
        << std::endl;
}

/////////////////////////////////////////////////
void SimbodyLink::AddTorque(const ignition::math::Vector3d &/*_torque*/)
{
}

/////////////////////////////////////////////////
void SimbodyLink::AddRelativeTorque(const ignition::math::Vector3d &/*_torque*/)
{
  gzerr << "Not implemented.\n";
}

/////////////////////////////////////////////////
void SimbodyLink::SetAutoDisable(bool /*_disable*/)
{
  gzerr << "Not implemented.\n";
}

/////////////////////////////////////////////////
SimTK::MassProperties SimbodyLink::MassProperties() const
{
  gzlog << "SimbodyLink::MassProperties for ["
    << this->ScopedName() << "]\n";

  if (!this->IsStatic())
  {
    const SimTK::Real mass = this->simbodyLinkDPtr->inertial.Mass();
    SimTK::Transform xli = physics::SimbodyPhysics::Pose2Transform(
        this->simbodyLinkDPtr->inertial.Pose());

    // vector from Lo to com, exp. in L
    const SimTK::Vec3 &coml = xli.p();

    if (ignition::math::equal(mass, 0.0))
      return SimTK::MassProperties(mass, coml, SimTK::UnitInertia(1, 1, 1));

    // Get mass-weighted central inertia, expressed in I frame.
    SimTK::Inertia ici(
        this->simbodyLinkDPtr->inertial.IXX(),
        this->simbodyLinkDPtr->inertial.IYY(),
        this->simbodyLinkDPtr->inertial.IZZ(),
        this->simbodyLinkDPtr->inertial.IXY(),
        this->simbodyLinkDPtr->inertial.IXZ(),
        this->simbodyLinkDPtr->inertial.IYZ());
    // Re-express the central inertia from the I frame to the L frame.
    // icl=ril*ici*ril
    SimTK::Inertia icl = ici.reexpress(~xli.R());

    // Shift to L frame origin.
    SimTK::Inertia iol = icl.shiftFromMassCenter(-coml, mass);

    // convert to unit inertia
    return SimTK::MassProperties(mass, coml, iol);
  }

  gzerr << "inertial block no specified, using unit mass properties\n";

  return SimTK::MassProperties(1, SimTK::Vec3(0),
      SimTK::UnitInertia(0.1, 0.1, 0.1));
}

/////////////////////////////////////////////////
// When a link is broken into several fragments (master and slaves), they
// share the mass equally. Given the number of fragments, this returns the
// appropriate mass properties to use for each fragment. Per Simbody's
// convention, COM is measured from, and inertia taken about, the link
// origin and both are expressed in the link frame.
SimTK::MassProperties SimbodyLink::EffectiveMassProps(
  int _numFragments) const
{
  SimTK::MassProperties massProps = this->MassProperties();
  GZ_ASSERT(_numFragments > 0,
      "_numFragments must be at least 1 for the master");
  return SimTK::MassProperties(massProps.getMass() / _numFragments,
      massProps.getMassCenter(),
      massProps.getUnitInertia());
}

/////////////////////////////////////////////////
bool SimbodyLink::Enabled() const
{
  return true;
}

/////////////////////////////////////////////////
void SimbodyLink::SetDirtyPose(const ignition::math::Pose3d &_pose)
{
  this->simbodyLinkDPtr->dirtyPose = _pose;
}

/////////////////////////////////////////////////
void SimbodyLink::SetPhysicsInitialized(const bool _value)
{
  this->simbodyLinkDPtr->physicsInitialized = _value;
}

/////////////////////////////////////////////////
SimTK::MobilizedBody &SimbodyLink::MobilizedBody() const
{
  return this->simbodyLinkDPtr->masterMobod;
}

/////////////////////////////////////////////////
bool SimbodyLink::MustBeBaseLink() const
{
  return this->simbodyLinkDPtr->mustBeBaseLink;
}

/////////////////////////////////////////////////
size_t SimbodyLink::SlaveMobodsCount() const
{
  return this->simbodyLinkDPtr->slaveMobods.size();
}

/////////////////////////////////////////////////
void SimbodyLink::AddSlaveMobod(SimTK::MobilizedBody _mobod)
{
  this->simbodyLinkDPtr->slaveMobods.push_back(_mobod);
}

/////////////////////////////////////////////////
SimTK::MobilizedBody &SimbodyLink::SlaveMobod(unsigned int _index) const
{
  if (_index >= this->simbodyLinkDPtr->slaveMobods.size())
  {
    _index = 0;
    gzerr << "Index with value[" << _index << "] must be between zero and "
      << this->simbodyLinkDPtr->slaveMobods.size()
      << ". Using a value of zero\n";
  }

  return this->simbodyLinkDPtr->slaveMobods[_index];
}

/////////////////////////////////////////////////
void SimbodyLink::AddSlaveWeld(SimTK::Constraint::Weld _weld)
{
  this->simbodyLinkDPtr->slaveWelds.push_back(_weld);
}
