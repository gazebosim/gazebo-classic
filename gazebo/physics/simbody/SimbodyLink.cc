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
#include "gazebo/common/Exception.hh"

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
      this->World()->GetPhysicsEngine());

  if (this->simbodyLinkDPtr->simbodyPhysics == NULL)
    gzthrow("Not using the simbody physics engine");

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

  ignition::math::Vector3d cogVec = this->inertial->CoG();

  // Set the initial pose of the body

  for (Base_V::iterator iter = this->children.begin();
       iter != this->children.end(); ++iter)
  {
    if ((*iter)->HasType(Base::COLLISION))
    {
      SimbodyCollisionPtr collision;
      collision = std::static_pointer_cast<SimbodyCollision>(*iter);

      ignition::math::Pose relativePose = collision->RelativePose();
      relativePose.pos -= cogVec;
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
      this->sdf->GetElement("gravity")->Set(this->simbodyLinkDPtr->gravityMode);
      this->simbodyLinkDPtr->simbodyPhysics->gravity.setBodyIsExcluded(
          this->simbodyLinkDPtr->simbodyPhysics->integ->updAdvancedState(),
          this->simbodyLinkDPtr->masterMobod,
          !this->simbodyLinkDPtr->gravityMode);

      // realize system after changing gravity mode
      this->simbodyLinkDPtr->simbodyPhysics->system.realize(
        this->simbodyLinkDPtr->simbodyPhysics->integ->getState(),
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
    return this->simbodyLinkDPtr->simbodyPhysics->gravity.getBodyIsExcluded(
      this->simbodyLinkDPtr->simbodyPhysics->integ->getState(),
      this->simbodyLinkDPtr->masterMobod);
  }
  else
  {
    gzlog << "GetGravityMode [" << this->simbodyLinkDPtr->gravityMode
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
//     gzthrow("requires SimbodyCollision");
//
//   ignition::math::Pose relativePose = _collision->RelativePose();
// }

//////////////////////////////////////////////////
/// changed
void SimbodyLink::OnPoseChange()
{
  Link::OnPoseChange();

  if (!this->simbodyLinkDPtr->simbodyPhysics->simbodyPhysicsInitialized)
    return;

  if (this->simbodyLinkDPtr->masterMobod.isEmptyHandle())
    return;

  // debug
  // gzerr << "original: [" << SimbodyPhysics::Transform2Pose(
  //   this->simbodyLinkDPtr->masterMobod.getBodyTransform(
  //   this->simbodyLinkDPtr->simbodyPhysics->integ->updAdvancedState()))
  //       << "]\n";

  if (!this->simbodyLinkDPtr->masterMobod.isGround())
  {
    if (this->simbodyLinkDPtr->masterMobod.getParentMobilizedBody().isGround())
    {
      /// If parent is ground:
      /// Setting 6 dof pose of a link works in simbody only if
      /// the inboard joint is a free joint to the ground for now.
      this->simbodyLinkDPtr->masterMobod.setQToFitTransform(
         this->simbodyLinkDPtr->simbodyPhysics->integ->updAdvancedState(),
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
        this->simbodyLinkDPtr->simbodyPhysics->integ->updAdvancedState()));
      ignition::math::Pose relPose = this->WorldPose() - parentPose;
      this->simbodyLinkDPtr->masterMobod.setQToFitTransform(
         this->simbodyLinkDPtr->simbodyPhysics->integ->updAdvancedState(),
         SimbodyPhysics::Pose2Transform(relPose));
      */
    }
    // realize system after updating Q's
    this->simbodyLinkDPtr->simbodyPhysics->system.realize(
        this->simbodyLinkDPtr->simbodyPhysics->integ->getState(),
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
       this->simbodyLinkDPtr->simbodyPhysics->integ->updAdvancedState());
    else
      this->simbodyLinkDPtr->masterMobod.unlock(
       this->simbodyLinkDPtr->simbodyPhysics->integ->updAdvancedState());

    // re-realize
    this->simbodyLinkDPtr->simbodyPhysics->system.realize(
      this->simbodyLinkDPtr->simbodyPhysics->integ->getAdvancedState(),
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
    this->simbodyLinkDPtr->simbodyPhysics->integ->updAdvancedState(),
    SimbodyPhysics::Vector3ToVec3(_vel));

  this->simbodyLinkDPtr->simbodyPhysics->system.realize(
    this->simbodyLinkDPtr->simbodyPhysics->integ->getAdvancedState(),
    SimTK::Stage::Velocity);
}

//////////////////////////////////////////////////
ignition::math::Vector3d SimbodyLink::WorldLinearVel(
  const ignition::math::Vector3& _offset) const
{
  SimTK::Vec3 station = SimbodyPhysics::Vector3ToVec3(_offset);
  ignition::math::Vector3d v;

  if (this->simbodyLinkDPtr->simbodyPhysics->simbodyPhysicsInitialized)
  {
    // lock physics update mutex to ensure thread safety
    boost::recursive_mutex::scoped_lock lock(
      *this->world->PhysicsEngine()->GetPhysicsUpdateMutex());

    v = SimbodyPhysics::Vec3ToVector3(
      this->simbodyLinkDPtr->masterMobod.findStationVelocityInGround(
      this->simbodyLinkDPtr->simbodyPhysics->integ->getState(), station));
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
  const ignition::math::Quaternion &_q) const
{
  ignition::math::Vector3d v;

  if (this->simbodyLinkDPtr->simbodyPhysics->simbodyPhysicsInitialized)
  {
    SimTK::Rotation rwf(SimbodyPhysics::QuadToQuad(_q));
    SimTK::Vec3 pf(SimbodyPhysics::Vector3ToVec3(_offset));
    SimTK::Vec3 pw(rwf * pf);

    // lock physics update mutex to ensure thread safety
    boost::recursive_mutex::scoped_lock lock(
      *this->world->PhysicsEngine()->GetPhysicsUpdateMutex());

    const SimTK::Rotation &rwl =
      this->simbodyLinkDPtr->masterMobod.getBodyRotation(
          this->simbodyLinkDPtr->simbodyPhysics->integ->getState());

    SimTK::Vec3 p_B(~rwl * pw);
    v = SimbodyPhysics::Vec3ToVector3(
      this->simbodyLinkDPtr->masterMobod.findStationVelocityInGround(
      this->simbodyLinkDPtr->simbodyPhysics->integ->getState(), p_B));
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

  if (this->simbodyLinkDPtr->simbodyPhysics->simbodyPhysicsInitialized)
  {
    // lock physics update mutex to ensure thread safety
    boost::recursive_mutex::scoped_lock lock(
      *this->world->PhysicsEngine()->GetPhysicsUpdateMutex());

    SimTK::Vec3 station =
      this->simbodyLinkDPtr->masterMobod.getBodyMassCenterStation(
          this->simbodyLinkDPtr->simbodyPhysics->integ->getState());

    v = SimbodyPhysics::Vec3ToVector3(
        this->simbodyLinkDPtr->masterMobod.findStationVelocityInGround(
          this->simbodyLinkDPtr->simbodyPhysics->integ->getState(), station));
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
      this->simbodyLinkDPtr->simbodyPhysics->integ->updAdvancedState(),
      SimbodyPhysics::Vector3ToVec3(_vel));

  this->simbodyLinkDPtr->simbodyPhysics->system.realize(
      this->simbodyLinkDPtr->simbodyPhysics->integ->getAdvancedState(),
      SimTK::Stage::Velocity);
}

//////////////////////////////////////////////////
ignition::math::Vector3d SimbodyLink::WorldAngularVel() const
{
  // lock physics update mutex to ensure thread safety
  boost::recursive_mutex::scoped_lock lock(
    *this->world->PhysicsEngine()->GetPhysicsUpdateMutex());

  SimTK::Vec3 w =
    this->simbodyLinkDPtr->masterMobod.getBodyAngularVelocity(
    this->simbodyLinkDPtr->simbodyPhysics->integ->getState());

  return SimbodyPhysics::Vec3ToVector3(w);
}

//////////////////////////////////////////////////
void SimbodyLink::SetForce(const ignition::math::Vector3d &_force)
{
  SimTK::Vec3 f(SimbodyPhysics::Vector3ToVec3(_force));

  this->simbodyLinkDPtr->simbodyPhysics->discreteForces.setOneBodyForce(
      this->simbodyLinkDPtr->simbodyPhysics->integ->updAdvancedState(),
      this->simbodyLinkDPtr->masterMobod, SimTK::SpatialVec(SimTK::Vec3(0), f));
}

//////////////////////////////////////////////////
ignition::math::Vector3d SimbodyLink::WorldForce() const
{
  SimTK::SpatialVec sv =
    this->simbodyLinkDPtr->simbodyPhysics->discreteForces.getOneBodyForce(
        this->simbodyLinkDPtr->simbodyPhysics->integ->getState(),
        this->simbodyLinkDPtr->masterMobod);

  // get translational component
  SimTK::Vec3 f = sv[1];

  return SimbodyPhysics::Vec3ToVector3(f);
}

//////////////////////////////////////////////////
void SimbodyLink::SetTorque(const ignition::math::Vector3d &_torque)
{
  SimTK::Vec3 t(SimbodyPhysics::Vector3ToVec3(_torque));

  this->simbodyLinkDPtr->simbodyPhysics->discreteForces.setOneBodyForce(
      this->simbodyLinkDPtr->simbodyPhysics->integ->updAdvancedState(),
      this->simbodyLinkDPtr->masterMobod, SimTK::SpatialVec(t, SimTK::Vec3(0)));
}

//////////////////////////////////////////////////
ignition::math::Vector3d SimbodyLink::WorldTorque() const
{
  SimTK::SpatialVec sv =
    this->simbodyLinkDPtr->simbodyPhysics->discreteForces.getOneBodyForce(
        this->simbodyLinkDPtr->simbodyPhysics->integ->getState(),
        this->simbodyLinkDPtr->masterMobod);

  // get rotational component
  SimTK::Vec3 t = sv[0];

  return SimbodyPhysics::Vec3ToVector3(t);
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

  this->simbodyLinkDPtr->simbodyPhysics->discreteForces.addForceToBodyPoint(
      this->simbodyLinkDPtr->simbodyPhysics->integ->updAdvancedState(),
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
    const SimTK::Real mass = this->inertial->GetMass();
    SimTK::Transform X_LI = physics::SimbodyPhysics::Pose2Transform(
        this->inertial->Pose());

    // vector from Lo to com, exp. in L
    const SimTK::Vec3 &com_L = X_LI.p();

    if (ignition::math::equal(mass, 0.0))
      return SimTK::MassProperties(mass, com_L, SimTK::UnitInertia(1, 1, 1));

    // Get mass-weighted central inertia, expressed in I frame.
    SimTK::Inertia Ic_I(this->inertial->IXX(),
        this->inertial->IYY(),
        this->inertial->IZZ(),
        this->inertial->IXY(),
        this->inertial->IXZ(),
        this->inertial->IYZ());
    // Re-express the central inertia from the I frame to the L frame.
    // Ic_L=R_LI*Ic_I*R_IL
    SimTK::Inertia Ic_L = Ic_I.reexpress(~X_LI.R());

    // Shift to L frame origin.
    SimTK::Inertia Io_L = Ic_L.shiftFromMassCenter(-com_L, mass);

    // convert to unit inertia
    return SimTK::MassProperties(mass, com_L, Io_L);
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
void SimbodyLink::SetDirtyPose(const ignition::math::Pose &_pose)
{
  this->simbodyLinkDPtr->dirtyPose = _pose;
}
