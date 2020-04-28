/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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

#include <functional>
#include <vector>

#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>

// OpenDE private definitions; unfortunately, we need them
#include "joints/contact.h"

#include "gazebo/common/Assert.hh"
#include "gazebo/transport/transport.hh"

#include "plugins/SimpleTrackedVehiclePlugin.hh"

namespace std {
template<class T>
class hash<boost::shared_ptr<T>> {
  public: size_t operator()(const boost::shared_ptr<T>& key) const {
    return (size_t)key.get();
  }
};
}

namespace gazebo
{
using namespace std;
using namespace physics;
/// \brief This is a temporary workaround to keep ABI compatibility in
/// Gazebo 9. It should be deleted starting with Gazebo 10.
unordered_map<LinkPtr, unordered_map<Tracks, Link_V> > globalTracks;
}

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(SimpleTrackedVehiclePlugin)

SimpleTrackedVehiclePlugin::~SimpleTrackedVehiclePlugin()
{
  if (this->body != nullptr)
  {
    if (globalTracks.find(this->body) != globalTracks.end())
    {
      globalTracks.erase(this->body);
    }
  }
}

void SimpleTrackedVehiclePlugin::Load(physics::ModelPtr _model,
                                      sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model, "SimpleTrackedVehiclePlugin: _model pointer is NULL");
  GZ_ASSERT(_sdf, "SimpleTrackedVehiclePlugin: _sdf pointer is NULL");

  if (_model->GetWorld()->Physics()->GetType() != "ode")
  {
    gzerr << "Tracked vehicle simulation works only with ODE." << std::endl;
    throw std::runtime_error("SimpleTrackedVehiclePlugin: Load() failed.");
  }

  TrackedVehiclePlugin::Load(_model, _sdf);

  if (!_sdf->HasElement("body"))
  {
    gzerr << "SimpleTrackedVehiclePlugin: <body> tag missing." << std::endl;
    throw std::runtime_error("SimpleTrackedVehiclePlugin: Load() failed.");
  }

  if (!_sdf->HasElement("left_track"))
  {
    gzerr << "SimpleTrackedVehiclePlugin: <left_track> tag missing."
          << std::endl;
    throw std::runtime_error("SimpleTrackedVehiclePlugin: Load() failed.");
  }

  if (!_sdf->HasElement("right_track"))
  {
    gzerr << "SimpleTrackedVehiclePlugin: <right_track> tag missing."
          << std::endl;
    throw std::runtime_error("SimpleTrackedVehiclePlugin: Load() failed.");
  }

  this->body = _model->GetLink(
      _sdf->GetElement("body")->Get<std::string>());
  if (this->body == nullptr)
  {
    gzerr << "SimpleTrackedVehiclePlugin: <body> link does not exist."
          << std::endl;
    throw std::runtime_error("SimpleTrackedVehiclePlugin: Load() failed.");
  }
  else
  {
    gzmsg << "SimpleTrackedVehiclePlugin: Successfully added robot body link "
          << this->body->GetName() << std::endl;
  }

  globalTracks.emplace(this->body,
      std::unordered_map<Tracks, physics::Link_V>());
  auto& gtracks = globalTracks.at(this->body);

  this->tracks[Tracks::LEFT] = _model->GetLink(
      _sdf->GetElement("left_track")->Get<std::string>());
  gtracks[Tracks::LEFT].push_back(this->tracks[Tracks::LEFT]);
  if (gtracks[Tracks::LEFT].at(0) == nullptr)
  {
    gzerr << "SimpleTrackedVehiclePlugin: <left_track> link does not exist."
          << std::endl;
    throw std::runtime_error("SimpleTrackedVehiclePlugin: Load() failed.");
  }
  else
  {
    gzmsg << "SimpleTrackedVehiclePlugin: Successfully added left track link "
          << gtracks[Tracks::LEFT].at(0)->GetName() << std::endl;
  }

  this->tracks[Tracks::RIGHT] = _model->GetLink(
      _sdf->GetElement("right_track")->Get<std::string>());
  gtracks[Tracks::RIGHT].push_back(this->tracks[Tracks::RIGHT]);
  if (gtracks[Tracks::RIGHT].at(0) == nullptr)
  {
    gzerr << "SimpleTrackedVehiclePlugin: <right_track> link does not exist."
          << std::endl;
    throw std::runtime_error("SimpleTrackedVehiclePlugin: Load() failed.");
  }
  else
  {
    gzmsg << "SimpleTrackedVehiclePlugin: Successfully added right track link "
          << gtracks[Tracks::RIGHT].at(0)->GetName() << std::endl;
  }

  if (_sdf->HasElement("left_flipper"))
  {
    auto flipper = _sdf->GetElement("left_flipper");
    while (flipper)
    {
      const auto flipperName = flipper->Get<std::string>();
      const auto flipperLink = _model->GetLink(flipperName);
      if (flipperLink == nullptr)
      {
        gzerr << "SimpleTrackedVehiclePlugin: <left_flipper> link '"
              << flipperName << "' does not exist." << std::endl;
      }
      else
      {
        gtracks[Tracks::LEFT].push_back(flipperLink);
        gzmsg << "SimpleTrackedVehiclePlugin: Successfully added left flipper "
                 "link '" << flipperName << "'" << std::endl;
      }
      flipper = flipper->GetNextElement("left_flipper");
    }
  }

  if (_sdf->HasElement("right_flipper"))
  {
    auto flipper = _sdf->GetElement("right_flipper");
    while (flipper)
    {
      const auto flipperName = flipper->Get<std::string>();
      const auto flipperLink = _model->GetLink(flipperName);
      if (flipperLink == nullptr)
      {
        gzerr << "SimpleTrackedVehiclePlugin: <right_flipper> link '"
              << flipperName << "' does not exist." << std::endl;
      }
      else
      {
        gtracks[Tracks::RIGHT].push_back(flipperLink);
        gzmsg << "SimpleTrackedVehiclePlugin: Successfully added right flipper "
                 "link '" << flipperName << "'" << std::endl;
      }
      flipper = flipper->GetNextElement("right_flipper");
    }
  }

  this->LoadParam(_sdf, "collide_without_contact_bitmask",
                  this->collideWithoutContactBitmask, 1u);
}

void SimpleTrackedVehiclePlugin::Init()
{
  TrackedVehiclePlugin::Init();

  physics::ModelPtr model = this->body->GetModel();

  this->contactManager = model->GetWorld()->Physics()->GetContactManager();
  // otherwise contact manager would not publish any contacts (since we are not
  // a real contact subscriber)
  this->contactManager->SetNeverDropContacts(true);

  // set correct categories and collide bitmasks
  this->SetGeomCategories();

  // set the desired friction to tracks (override the values set in the
  // SDF model)
  this->UpdateTrackSurface();

  // initialize Gazebo node, subscribers and publishers and event connections
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(model->GetWorld()->Name());

  this->beforePhysicsUpdateConnection =
      event::Events::ConnectBeforePhysicsUpdate(
          std::bind(&SimpleTrackedVehiclePlugin::DriveTracks, this,
                    std::placeholders::_1));
}

void SimpleTrackedVehiclePlugin::Reset()
{
  TrackedVehiclePlugin::Reset();
}

void SimpleTrackedVehiclePlugin::SetTrackVelocityImpl(double _left,
                                                      double _right)
{
  this->trackVelocity[Tracks::LEFT] = _left;
  this->trackVelocity[Tracks::RIGHT] = _right;
}

void SimpleTrackedVehiclePlugin::UpdateTrackSurface()
{
  auto& gtracks = globalTracks.at(this->body);
  for (auto trackSide : gtracks)
  {
    for (auto track : trackSide.second)
    {
      this->SetLinkMu(track);
    }
  }
}

void SimpleTrackedVehiclePlugin::SetGeomCategories()
{
  auto linksToProcess = this->body->GetModel()->GetLinks();

  // set ROBOT_CATEGORY to the whole body and all subparts
  physics::LinkPtr link;
  while (!linksToProcess.empty())
  {
    link = linksToProcess.back();
    linksToProcess.pop_back();

    auto childLinks = link->GetChildJointsLinks();
    linksToProcess.insert(linksToProcess.end(), childLinks.begin(),
                          childLinks.end());

    for (auto const &collision : link->GetCollisions())
    {
      collision->SetCategoryBits(ROBOT_CATEGORY);
      collision->SetCollideBits(GZ_FIXED_COLLIDE);

      GZ_ASSERT(collision->GetSurface() != nullptr,
                "Collision surface is nullptr");
      collision->GetSurface()->collideWithoutContactBitmask =
        this->collideWithoutContactBitmask;
    }
  }

  auto& gtracks = globalTracks.at(this->body);
  for (auto trackSide : gtracks)
  {
    for (auto trackLink : trackSide.second)
    {
      auto bits = ROBOT_CATEGORY | BELT_CATEGORY;
      if (trackSide.first == Tracks::LEFT)
        bits |= LEFT_CATEGORY;

      for (auto const &collision : trackLink->GetCollisions())
      {
        collision->SetCategoryBits(bits);
      }
    }
  }
}

size_t SimpleTrackedVehiclePlugin::GetNumTracks(const Tracks side) const
{
  auto& gtracks = globalTracks.at(this->body);
  return gtracks[side].size();
}

void SimpleTrackedVehiclePlugin::DriveTracks(
    const common::UpdateInfo &/*_unused*/)
{
  if (this->contactManager->GetContactCount() == 0)
    return;

  /////////////////////////////////////////////
  // Calculate the desired center of rotation
  /////////////////////////////////////////////

  const auto leftBeltSpeed = -this->trackVelocity[Tracks::LEFT];
  const auto rightBeltSpeed = -this->trackVelocity[Tracks::RIGHT];

  // the desired linear and angular speeds (set by desired track velocities)
  const auto linearSpeed = (leftBeltSpeed + rightBeltSpeed) / 2;
  const auto angularSpeed = -(leftBeltSpeed - rightBeltSpeed) *
    this->GetSteeringEfficiency() / this->GetTracksSeparation();

  // radius of the turn the robot is doing
  const auto desiredRotationRadiusSigned =
                               (fabs(angularSpeed) < 0.1) ?
                               // is driving straight
                               dInfinity :
                               (
                                 (fabs(linearSpeed) < 0.1) ?
                                 // is rotating about a single point
                                 0 :
                                 // general movement
                                 linearSpeed / angularSpeed);

  const auto bodyPose = this->body->WorldPose();
  const auto bodyYAxisGlobal =
    bodyPose.Rot().RotateVector(ignition::math::Vector3d(0, 1, 0));
  const auto centerOfRotation =
    (bodyYAxisGlobal * desiredRotationRadiusSigned) + bodyPose.Pos();

  ////////////////////////////////////////////////////////////////////////
  // For each contact, compute the friction force direction and speed of
  // surface movement.
  ////////////////////////////////////////////////////////////////////////
  size_t i = 0;
  const auto contacts = this->contactManager->GetContacts();
  const auto model = this->body->GetModel();

  for (auto contact : contacts)
  {
    // Beware! There may be invalid contacts beyond GetContactCount()...
    if (i == this->contactManager->GetContactCount())
      break;

    ++i;

    if (contact->collision1->GetSurface()->collideWithoutContact ||
      contact->collision2->GetSurface()->collideWithoutContact)
      continue;

    if (!contact->collision1->GetLink()->GetEnabled() ||
      !contact->collision2->GetLink()->GetEnabled())
      continue;

    if (contact->collision1->IsStatic() && contact->collision2->IsStatic())
    {
      // we're not interested in static model collisions
      // (they do not have any ODE bodies).
      continue;
    }

    if (model != contact->collision1->GetLink()->GetModel() &&
        model != contact->collision2->GetLink()->GetModel())
    {
      // Verify one of the collisions' bodies is a track of this vehicle
      continue;
    }

    dBodyID body1 = dynamic_cast<physics::ODELink&>(
      *contact->collision1->GetLink()).GetODEId();
    dBodyID body2 = dynamic_cast<physics::ODELink& >(
      *contact->collision2->GetLink()).GetODEId();

    dGeomID geom1 = dynamic_cast<physics::ODECollision& >(
      *contact->collision1).GetCollisionId();
    dGeomID geom2 = dynamic_cast<physics::ODECollision& >(
      *contact->collision2).GetCollisionId();

    bool bodiesSwapped = false;
    if (body1 == 0)
    {
      std::swap(body1, body2);
      std::swap(geom1, geom2);

      // we'll take care of the normal flipping later
      bodiesSwapped = true;
    }

    // determine if track is the first or second collision element
    const bool isGeom1Track = (dGeomGetCategoryBits(geom1) & BELT_CATEGORY) > 0;
    const bool isGeom2Track = (dGeomGetCategoryBits(geom2) & BELT_CATEGORY) > 0;

    if (!isGeom1Track && !isGeom2Track)
      continue;

    // speed and geometry of the track in collision
    const auto trackGeom = (isGeom1Track ? geom1 : geom2);
    // the != means XOR here; we basically want to get the collision belonging
    // to the track, but we might have swapped the ODE bodies in between,
    // so we have to account for it
    const physics::Collision* trackCollision =
      ((isGeom1Track != bodiesSwapped) ? contact->collision1
                                       : contact->collision2);
    const dReal beltSpeed =
      (dGeomGetCategoryBits(trackGeom) & LEFT_CATEGORY) != 0 ?
      leftBeltSpeed : rightBeltSpeed;

    // remember if we've found at least one contact joint (we should!)
    bool foundContact = false;
    for (auto contactIterator = ContactIterator::begin(body1, geom1, geom2);
         contactIterator != ContactIterator::end();
         ++contactIterator)
    {
      dContact* odeContact = contactIterator.getPointer();

      // now we're sure it is a contact between our two geometries
      foundContact = true;

      const ignition::math::Vector3d contactWorldPosition(
        odeContact->geom.pos[0],
        odeContact->geom.pos[1],
        odeContact->geom.pos[2]);

      ignition::math::Vector3d contactNormal(
        odeContact->geom.normal[0],
        odeContact->geom.normal[1],
        odeContact->geom.normal[2]);

      // We always want contactNormal to point "inside" the track.
      // The dot product is 1 for co-directional vectors and -1 for
      // opposite-pointing vectors.
      // The contact can be flipped either by swapping body1 and body2 above,
      // or by having some flipped faces on collision meshes.
      const double normalToTrackCenterDot =
        contactNormal.Dot(
          trackCollision->WorldPose().Pos() - contactWorldPosition);
      if (normalToTrackCenterDot < 0)
      {
        contactNormal = -contactNormal;
      }

      // vector tangent to the belt pointing in the belt's movement direction
      auto beltDirection(contactNormal.Cross(bodyYAxisGlobal));

      if (beltSpeed > 0)
        beltDirection = -beltDirection;

      const auto frictionDirection =
        this->ComputeFrictionDirection(linearSpeed,
                                       angularSpeed,
                                       desiredRotationRadiusSigned == dInfinity,
                                       bodyPose,
                                       bodyYAxisGlobal,
                                       centerOfRotation,
                                       contactWorldPosition,
                                       contactNormal,
                                       beltDirection);

      odeContact->fdir1[0] = frictionDirection.X();
      odeContact->fdir1[1] = frictionDirection.Y();
      odeContact->fdir1[2] = frictionDirection.Z();

      // use friction direction and motion1 to simulate the track movement
      odeContact->surface.mode |= dContactFDir1 | dContactMotion1;

      odeContact->surface.motion1 = this->ComputeSurfaceMotion(
        beltSpeed, beltDirection, frictionDirection);
    }

    if (!foundContact)
    {
      gzwarn << "No ODE contact joint found for contact " <<
             contact->DebugString() << std::endl;
      continue;
    }
  }
}

ignition::math::Vector3d SimpleTrackedVehiclePlugin::ComputeFrictionDirection(
  const double _linearSpeed, const double _angularSpeed,
  const bool _drivingStraight, const ignition::math::Pose3d &_bodyPose,
  const ignition::math::Vector3d &_bodyYAxisGlobal,
  const ignition::math::Vector3d &_centerOfRotation,
  const ignition::math::Vector3d &_contactWorldPosition,
  const ignition::math::Vector3d &_contactNormal,
  const ignition::math::Vector3d &_beltDirection) const
{
  ignition::math::Vector3d frictionDirection;

  if (!_drivingStraight)
  {
    // non-straight drive

    // vector pointing from the center of rotation to the contact point
    const auto COR2Contact =
      (_contactWorldPosition - _centerOfRotation).Normalize();

    // the friction force should be perpendicular to COR2Contact
    frictionDirection = _contactNormal.Cross(COR2Contact);

    // position of the contact point relative to vehicle body
    const auto contactInVehiclePos =
        _bodyPose.Rot().RotateVectorReverse(
          _contactWorldPosition - _bodyPose.Pos());

    const int linearSpeedSignum =
        (fabs(_linearSpeed) > 0.1) ? ignition::math::signum(_linearSpeed) : 1;

    // contactInVehiclePos.Dot(ignition::math::Vector3d(1, 0, 0)) > 0 means
    // the contact is "in front" of the line on which COR moves
    if ((ignition::math::signum(_angularSpeed) *
      ignition::math::signum(_bodyYAxisGlobal.Dot(frictionDirection))) !=
      (linearSpeedSignum *
        ignition::math::signum(contactInVehiclePos.Dot(
          ignition::math::Vector3d(1, 0, 0)))))
    {
      frictionDirection = -frictionDirection;
    }

    if (_linearSpeed < 0)
      frictionDirection = - frictionDirection;
  }
  else
  {
    // straight drive
    frictionDirection = _contactNormal.Cross(_bodyYAxisGlobal);

    if (frictionDirection.Dot(_beltDirection) < 0)
      frictionDirection = -frictionDirection;
  }

  return frictionDirection;
}

double SimpleTrackedVehiclePlugin::ComputeSurfaceMotion(const double _beltSpeed,
    const ignition::math::Vector3d &_beltDirection,
    const ignition::math::Vector3d &_frictionDirection) const
{
  // the dot product <beltDirection,fdir1> is the cosine of the angle they
  // form (because both are unit vectors)
  // the motion is in the opposite direction than the desired motion of the body
  return -_beltDirection.Dot(_frictionDirection) * fabs(_beltSpeed);
}

SimpleTrackedVehiclePlugin::ContactIterator
SimpleTrackedVehiclePlugin::ContactIterator::operator++()
{
  // initialized && null contact means we've reached the end of the iterator
  if (this->initialized && this->currentContact == nullptr)
  {
    return *this;
  }

  // I haven't found a nice way to get ODE ID of the collision joint,
  // so we need to iterate over all joints connecting the two colliding
  // bodies and try to find the one we're interested in.
  // This should not be a performance issue, since bodies connected by other
  // joint types do not collide by default.

  // remember if we've found at least one contact joint (we should!)
  bool found = false;
  for (; this->jointIndex < static_cast<size_t>(dBodyGetNumJoints(this->body));
         this->jointIndex++)
  {
    const auto joint = dBodyGetJoint(this->body,
                                     static_cast<int>(this->jointIndex));

    // only interested in contact joints
    if (dJointGetType(joint) != dJointTypeContact)
      continue;

    // HACK here we unfortunately have to access private ODE data
    // It must really be static_cast here; if dynamic_cast is used, the runtime
    // cannot find RTTI for dxJointContact and its predecessors.
    dContact* odeContact = &(static_cast<dxJointContact*>(joint)->contact);

    if (!(
            odeContact->geom.g1 == this->geom1 &&
            odeContact->geom.g2 == this->geom2)
        &&
        !(
            odeContact->geom.g1 == this->geom2 &&
            odeContact->geom.g2 == this->geom1))
    {
      // not a contact between our two geometries
      continue;
    }

    // we found a contact we're interested in

    found = true;
    this->initialized = true;

    // we can be pretty sure the contact instance won't get deleted until this
    // code finishes, since we are in a pause between contact generation and
    // physics update
    this->currentContact = odeContact;

    // needed since we break out of the for-loop
    this->jointIndex++;
    break;
  }

  if (!found)
  {
    // we've reached the end of the iterator
    this->currentContact = nullptr;
    this->initialized = true;
  }

  this->initialized = true;
  return *this;
}

SimpleTrackedVehiclePlugin::ContactIterator::ContactIterator()
    : currentContact(nullptr), jointIndex(0), body(nullptr), geom1(nullptr),
      geom2(nullptr), initialized(false)
{
}

SimpleTrackedVehiclePlugin::ContactIterator::ContactIterator(
    bool _initialized) : currentContact(nullptr), jointIndex(0), body(nullptr),
                         geom1(nullptr), geom2(nullptr),
                         initialized(_initialized)
{
}

SimpleTrackedVehiclePlugin::ContactIterator::ContactIterator(
    const SimpleTrackedVehiclePlugin::ContactIterator &_rhs)
{
  this->currentContact = _rhs.currentContact;
  this->initialized = _rhs.initialized;
  this->jointIndex = _rhs.jointIndex;
  this->body = _rhs.body;
  this->geom1 = _rhs.geom1;
  this->geom2 = _rhs.geom2;
}

SimpleTrackedVehiclePlugin::ContactIterator::ContactIterator(
    dBodyID _body, dGeomID _geom1, dGeomID _geom2) :
    currentContact(nullptr), jointIndex(0), body(_body),
    geom1(_geom1), geom2(_geom2), initialized(false)
{
}

SimpleTrackedVehiclePlugin::ContactIterator
SimpleTrackedVehiclePlugin::ContactIterator::begin(
    dBodyID _body, dGeomID _geom1, dGeomID _geom2)
{
  return ContactIterator(_body, _geom1, _geom2);
}

SimpleTrackedVehiclePlugin::ContactIterator
SimpleTrackedVehiclePlugin::ContactIterator::end()
{
  return ContactIterator(true);
}

bool SimpleTrackedVehiclePlugin::ContactIterator::operator==(
    const SimpleTrackedVehiclePlugin::ContactIterator &_rhs)
{
  if (this->currentContact == nullptr && !this->initialized)
    ++(*this);

  return this->currentContact == _rhs.currentContact &&
         this->initialized == _rhs.initialized;
}

SimpleTrackedVehiclePlugin::ContactIterator&
SimpleTrackedVehiclePlugin::ContactIterator::operator=(
    const SimpleTrackedVehiclePlugin::ContactIterator &_rhs)
{
  this->currentContact = _rhs.currentContact;
  this->initialized = _rhs.initialized;
  this->jointIndex = _rhs.jointIndex;
  this->body = _rhs.body;
  this->geom1 = _rhs.geom1;
  this->geom2 = _rhs.geom2;

  return *this;
}

SimpleTrackedVehiclePlugin::ContactIterator
SimpleTrackedVehiclePlugin::ContactIterator::operator++(int /*_unused*/)
{
  ContactIterator i = *this;
  ++(*this);
  return i;
}

SimpleTrackedVehiclePlugin::ContactIterator::reference
SimpleTrackedVehiclePlugin::ContactIterator::operator*()
{
  if (!this->initialized)
    ++(*this);

  return *this->currentContact;
}

SimpleTrackedVehiclePlugin::ContactIterator::pointer
SimpleTrackedVehiclePlugin::ContactIterator::operator->()
{
  if (!this->initialized)
    ++(*this);

  return this->currentContact;
}

SimpleTrackedVehiclePlugin::ContactIterator::pointer
SimpleTrackedVehiclePlugin::ContactIterator::getPointer()
{
  if (!this->initialized)
    ++(*this);

  return this->currentContact;
}

bool SimpleTrackedVehiclePlugin::ContactIterator::operator!=(
    const SimpleTrackedVehiclePlugin::ContactIterator &_rhs)
{
  return !SimpleTrackedVehiclePlugin::ContactIterator::operator==(_rhs);
}
