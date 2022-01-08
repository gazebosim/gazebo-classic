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
#include <boost/bind/bind.hpp>
#include <boost/thread.hpp>

#include "gazebo/common/Assert.hh"
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
  this->gravityModeDirty = false;
}

//////////////////////////////////////////////////
SimbodyLink::~SimbodyLink()
{
}

//////////////////////////////////////////////////
void SimbodyLink::Load(sdf::ElementPtr _sdf)
{
  this->simbodyPhysics = boost::dynamic_pointer_cast<SimbodyPhysics>(
      this->GetWorld()->Physics());

  if (this->simbodyPhysics == nullptr)
    gzthrow("Not using the simbody physics engine");

  if (_sdf->HasElement("must_be_base_link"))
    this->mustBeBaseLink = _sdf->Get<bool>("must_be_base_link");

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
      collision = boost::static_pointer_cast<SimbodyCollision>(*iter);

      ignition::math::Pose3d relativePose = collision->RelativePose();
      relativePose.Pos() -= cogVec;
    }
  }

  // Create a construction info object
  // Create the new rigid body

  // change link's gravity mode if requested by user
  this->gravityModeConnection = event::Events::ConnectWorldUpdateBegin(
    boost::bind(&SimbodyLink::ProcessSetGravityMode, this));

  // lock or unlock the link if requested by user
  this->staticLinkConnection = event::Events::ConnectWorldUpdateEnd(
    boost::bind(&SimbodyLink::ProcessSetLinkStatic, this));
}

//////////////////////////////////////////////////
void SimbodyLink::Fini()
{
  this->gravityModeConnection.reset();
  this->staticLinkConnection.reset();
  Link::Fini();
}

/////////////////////////////////////////////////////////////////////
void SimbodyLink::UpdateMass()
{
}

//////////////////////////////////////////////////
void SimbodyLink::SetGravityMode(bool _mode)
{
  if (!this->gravityModeDirty)
  {
    this->gravityModeDirty = true;
    this->gravityMode = _mode;
  }
  else
    gzerr << "Trying to SetGravityMode for link [" << this->GetScopedName()
          << "] before last setting is processed.\n";
}

//////////////////////////////////////////////////
void SimbodyLink::ProcessSetGravityMode()
{
  if (this->gravityModeDirty)
  {
    if (this->physicsInitialized)
    {
      this->sdf->GetElement("gravity")->Set(this->gravityMode);
      this->simbodyPhysics->gravity.setBodyIsExcluded(
        this->simbodyPhysics->integ->updAdvancedState(),
        this->masterMobod, !this->gravityMode);
      // realize system after changing gravity mode
      this->simbodyPhysics->system.realize(
        this->simbodyPhysics->integ->getState(), SimTK::Stage::Velocity);
      this->gravityModeDirty = false;
    }
    else
    {
      gzlog << "SetGravityMode [" << this->gravityMode
            << "], but physics not initialized, caching\n";
    }
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
// void SimbodyLink::AttachCollision(Collision *_collision)
// {
//   Link::AttachCollision(_collision);
//
//   SimbodyCollision *bcollision = dynamic_cast<SimbodyCollision*>(_collision);
//
//   if (_collision == nullptr)
//     gzthrow("requires SimbodyCollision");
//
//     ignition::math::Pose3d relativePose = _collision->GetRelativePose();
// }

//////////////////////////////////////////////////
/// changed
void SimbodyLink::OnPoseChange()
{
  Link::OnPoseChange();

  if (!this->simbodyPhysics->simbodyPhysicsInitialized)
    return;

  if (this->masterMobod.isEmptyHandle())
    return;

  // debug
  // gzerr << "original: [" << SimbodyPhysics::Transform2Pose(
  //   this->masterMobod.getBodyTransform(
  //   this->simbodyPhysics->integ->updAdvancedState()))
  //       << "]\n";

  if (!this->masterMobod.isGround())
  {
    if (this->masterMobod.getParentMobilizedBody().isGround())
    {
      /// If parent is ground:
      /// Setting 6 dof pose of a link works in simbody only if
      /// the inboard joint is a free joint to the ground for now.
      this->masterMobod.setQToFitTransform(
         this->simbodyPhysics->integ->updAdvancedState(),
         SimbodyPhysics::Pose2Transform(this->WorldPose()));
    }
    else
    {
      gzerr << "SetWorldPose (OnPoseChange) for child links need testing.\n";
      /// If the inboard joint is not free, simbody tries to project
      /// target pose into available DOF's.
      /// But first convert to relative pose to parent mobod.
      // ignition::math::Pose3d parentPose = SimbodyPhysics::Transform2PoseIgn(
      //   this->masterMobod.getBodyTransform(
      //   this->simbodyPhysics->integ->updAdvancedState()));
      //   ignition::math::Pose3d relPose = this->WorldPose() - parentPose;
      // this->masterMobod.setQToFitTransform(
      //    this->simbodyPhysics->integ->updAdvancedState(),
      //    SimbodyPhysics::Pose2Transform(relPose));
    }
    // realize system after updating Q's
    this->simbodyPhysics->system.realize(
      this->simbodyPhysics->integ->getState(), SimTK::Stage::Position);
  }
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
void SimbodyLink::SetLinearVel(const ignition::math::Vector3d & _vel)
{
  this->masterMobod.setUToFitLinearVelocity(
    this->simbodyPhysics->integ->updAdvancedState(),
    SimbodyPhysics::Vector3ToVec3(_vel));
  this->simbodyPhysics->system.realize(
    this->simbodyPhysics->integ->getAdvancedState(), SimTK::Stage::Velocity);
}

//////////////////////////////////////////////////
ignition::math::Vector3d SimbodyLink::WorldLinearVel(
  const ignition::math::Vector3d &_offset) const
{
  SimTK::Vec3 station = SimbodyPhysics::Vector3ToVec3(_offset);
  ignition::math::Vector3d v;

  if (this->simbodyPhysics->simbodyPhysicsInitialized)
  {
    // lock physics update mutex to ensure thread safety
    boost::recursive_mutex::scoped_lock lock(
      *this->world->Physics()->GetPhysicsUpdateMutex());
    v = SimbodyPhysics::Vec3ToVector3Ign(
      this->masterMobod.findStationVelocityInGround(
      this->simbodyPhysics->integ->getState(), station));
  }
  else
    gzwarn << "SimbodyLink::WorldLinearVel: simbody physics"
           << " not yet initialized\n";

  return v;
}

//////////////////////////////////////////////////
ignition::math::Vector3d SimbodyLink::WorldLinearVel(
  const ignition::math::Vector3d &_offset,
  const ignition::math::Quaterniond &_q) const
{
  ignition::math::Vector3d v;

  if (this->simbodyPhysics->simbodyPhysicsInitialized)
  {
    SimTK::Rotation R_WF(SimbodyPhysics::QuadToQuad(_q));
    SimTK::Vec3 p_F(SimbodyPhysics::Vector3ToVec3(_offset));
    SimTK::Vec3 p_W(R_WF * p_F);

    // lock physics update mutex to ensure thread safety
    boost::recursive_mutex::scoped_lock lock(
      *this->world->Physics()->GetPhysicsUpdateMutex());

    const SimTK::Rotation &R_WL = this->masterMobod.getBodyRotation(
      this->simbodyPhysics->integ->getState());
    SimTK::Vec3 p_B(~R_WL * p_W);
    v = SimbodyPhysics::Vec3ToVector3Ign(
      this->masterMobod.findStationVelocityInGround(
      this->simbodyPhysics->integ->getState(), p_B));
  }
  else
    gzwarn << "SimbodyLink::WorldLinearVel: simbody physics"
           << " not yet initialized\n";

  return v;
}

//////////////////////////////////////////////////
ignition::math::Vector3d SimbodyLink::WorldCoGLinearVel() const
{
  ignition::math::Vector3d v;

  if (this->simbodyPhysics->simbodyPhysicsInitialized)
  {
    // lock physics update mutex to ensure thread safety
    boost::recursive_mutex::scoped_lock lock(
      *this->world->Physics()->GetPhysicsUpdateMutex());
    SimTK::Vec3 station = this->masterMobod.getBodyMassCenterStation(
       this->simbodyPhysics->integ->getState());
    v = SimbodyPhysics::Vec3ToVector3Ign(
      this->masterMobod.findStationVelocityInGround(
      this->simbodyPhysics->integ->getState(), station));
  }
  else
    gzwarn << "SimbodyLink::WorldCoGLinearVel: simbody physics"
           << " not yet initialized\n";

  return v;
}

//////////////////////////////////////////////////
void SimbodyLink::SetAngularVel(const ignition::math::Vector3d &_vel)
{
  this->masterMobod.setUToFitAngularVelocity(
    this->simbodyPhysics->integ->updAdvancedState(),
    SimbodyPhysics::Vector3ToVec3(_vel));
  this->simbodyPhysics->system.realize(
    this->simbodyPhysics->integ->getAdvancedState(), SimTK::Stage::Velocity);
}

//////////////////////////////////////////////////
ignition::math::Vector3d SimbodyLink::WorldAngularVel() const
{
  // lock physics update mutex to ensure thread safety
  boost::recursive_mutex::scoped_lock lock(
    *this->world->Physics()->GetPhysicsUpdateMutex());
  SimTK::Vec3 w =
    this->masterMobod.getBodyAngularVelocity(
    this->simbodyPhysics->integ->getState());
  return SimbodyPhysics::Vec3ToVector3Ign(w);
}

//////////////////////////////////////////////////
void SimbodyLink::SetForce(const ignition::math::Vector3d &_force)
{
  SimTK::Vec3 f(SimbodyPhysics::Vector3ToVec3(_force));

  this->simbodyPhysics->discreteForces.setOneBodyForce(
    this->simbodyPhysics->integ->updAdvancedState(),
    this->masterMobod, SimTK::SpatialVec(SimTK::Vec3(0), f));
}

//////////////////////////////////////////////////
ignition::math::Vector3d SimbodyLink::WorldForce() const
{
  SimTK::SpatialVec sv = this->simbodyPhysics->discreteForces.getOneBodyForce(
    this->simbodyPhysics->integ->getState(), this->masterMobod);

  // get translational component
  SimTK::Vec3 f = sv[1];

  return SimbodyPhysics::Vec3ToVector3Ign(f);
}

//////////////////////////////////////////////////
void SimbodyLink::SetTorque(const ignition::math::Vector3d &_torque)
{
  SimTK::Vec3 t(SimbodyPhysics::Vector3ToVec3(_torque));

  this->simbodyPhysics->discreteForces.setOneBodyForce(
    this->simbodyPhysics->integ->updAdvancedState(),
    this->masterMobod, SimTK::SpatialVec(t, SimTK::Vec3(0)));
}

//////////////////////////////////////////////////
ignition::math::Vector3d SimbodyLink::WorldTorque() const
{
  SimTK::SpatialVec sv = this->simbodyPhysics->discreteForces.getOneBodyForce(
    this->simbodyPhysics->integ->getState(), this->masterMobod);

  // get rotational component
  SimTK::Vec3 t = sv[0];

  return SimbodyPhysics::Vec3ToVector3Ign(t);
}

//////////////////////////////////////////////////
void SimbodyLink::SetLinearDamping(double /*_damping*/)
{
  gzerr << "Not implemented.\n";
}

//////////////////////////////////////////////////
void SimbodyLink::SetAngularDamping(double /*_damping*/)
{
  gzerr << "Not implemented.\n";
}

/////////////////////////////////////////////////
void SimbodyLink::AddForce(const ignition::math::Vector3d &_force)
{
  SimTK::Vec3 f(SimbodyPhysics::Vector3ToVec3(_force));

  this->simbodyPhysics->discreteForces.addForceToBodyPoint(
    this->simbodyPhysics->integ->updAdvancedState(),
    this->masterMobod,
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
SimTK::MassProperties SimbodyLink::GetMassProperties() const
{
  gzlog << "SimbodyLink::GetMassProperties for ["
        << this->GetScopedName() << "]\n";

  if (!this->IsStatic())
  {
    const SimTK::Real mass = this->inertial->Mass();
    SimTK::Transform X_LI = physics::SimbodyPhysics::Pose2Transform(
      this->inertial->Pose());
    const SimTK::Vec3 &com_L = X_LI.p();  // vector from Lo to com, exp. in L

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
    GZ_ASSERT(_numFragments > 0,
              "_numFragments must be at least 1 for the master");
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
void SimbodyLink::SetDirtyPose(const ignition::math::Pose3d &_pose)
{
  this->dirtyPose = _pose;
}
