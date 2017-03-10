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

#include "gazebo/common/Assert.hh"
#include "gazebo/transport/transport.hh"

#include "plugins/SimpleTrackedVehiclePlugin.hh"


using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(SimpleTrackedVehiclePlugin)

SimpleTrackedVehiclePlugin::SimpleTrackedVehiclePlugin()
{
}

SimpleTrackedVehiclePlugin::~SimpleTrackedVehiclePlugin()
{
}

void SimpleTrackedVehiclePlugin::Load(physics::ModelPtr _model,
                                sdf::ElementPtr _sdf) {
  if (_model->GetWorld()->Physics()->GetType().compare("ode") != 0) {
    gzerr << "Tracked vehicle simulation works only with ODE." << std::endl;
    return;
  }

  TrackedVehiclePlugin::Load(_model, _sdf);

  GZ_ASSERT(_model, "SimpleTrackedVehiclePlugin _model pointer is NULL");
  GZ_ASSERT(_sdf, "SimpleTrackedVehiclePlugin _sdf pointer is NULL");

  GZ_ASSERT(_sdf->HasElement("body"), "SimpleTrackedVehiclePlugin "
      "<body> tag missing.");

  GZ_ASSERT(_sdf->HasElement("left_track"), "SimpleTrackedVehiclePlugin "
      "<left_track> tag missing.");

  GZ_ASSERT(_sdf->HasElement("right_track"), "SimpleTrackedVehiclePlugin "
      "<right_track> tag missing.");

  this->body = _model->GetLink(
      _sdf->GetElement("body")->Get<std::string>());
  GZ_ASSERT(this->body, "SimpleTrackedVehiclePlugin "
      "<body> link does not exist.");

  this->leftTrack = _model->GetLink(
      _sdf->GetElement("left_track")->Get<std::string>());
  GZ_ASSERT(this->leftTrack, "SimpleTrackedVehiclePlugin "
      "<left_track> link does not exist.");

  this->rightTrack = _model->GetLink(
      _sdf->GetElement("right_track")->Get<std::string>());
  GZ_ASSERT(this->rightTrack, "SimpleTrackedVehiclePlugin "
      "<right_track> link does not exist.");

  this->LoadParam(_sdf, "collide_without_contact_bitmask",
                  this->collideWithoutContactBitmask, 1u);
}

void SimpleTrackedVehiclePlugin::Init()
{
  TrackedVehiclePlugin::Init();

  physics::ModelPtr model = this->body->GetModel();

  this->contactManager = model->GetWorld()->Physics()->GetContactManager();

  // set correct categories and collide bitmasks
  this->setGeomCategories();
  for (auto link : model->GetLinks()) {
    for (auto collision : link->GetCollisions()) {
      collision->GetSurface()->collideWithoutContactBitmask =
          this->collideWithoutContactBitmask;
    }
  }

  // initialize Gazebo node, subscribers and publishers and event connections
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(model->GetWorld()->Name());

  // HACK Contact manager would not publish any contacts unless there is at
  // least one filter or someone subscribes to the ~/physics/contacts gazebo
  // topic. We do not handle the received contacts in any way, because we need
  // to process them earlier then the message is published (which is done in
  // driveTracks()).
  this->contactsSubscriber = this->node->Subscribe(
      this->GetRobotNamespace() + "/physics/contacts",
      &SimpleTrackedVehiclePlugin::ignoreContacts, this);

  this->beforePhysicsUpdateConnection =
      event::Events::ConnectBeforePhysicsUpdate(
          std::bind(&SimpleTrackedVehiclePlugin::driveTracks, this,
                    std::placeholders::_1));
}

void SimpleTrackedVehiclePlugin::Reset()
{
  this->leftTrackVelocity = this->rightTrackVelocity = 0;

  TrackedVehiclePlugin::Reset();
}

void SimpleTrackedVehiclePlugin::SetTrackVelocity(double left, double right)
{
  this->leftTrackVelocity = -left;
  this->rightTrackVelocity = -right;
}

void SimpleTrackedVehiclePlugin::setGeomCategories() {
  std::vector<physics::LinkPtr> linksToProcess(
      this->body->GetModel()->GetLinks());

  physics::LinkPtr link;
  while (!linksToProcess.empty()) {
    link = linksToProcess.back();
    linksToProcess.pop_back();

    std::vector<physics::LinkPtr> childLinks(link->GetChildJointsLinks());
    linksToProcess.insert(linksToProcess.end(), childLinks.begin(),
                          childLinks.end());

    for (auto collision : link->GetCollisions()) {
      collision->SetCategoryBits(ROBOT_CATEGORY);
      collision->SetCollideBits(GZ_FIXED_COLLIDE);
    }
  }

  for (physics::CollisionPtr collision : this->leftTrack->GetCollisions()) {
    collision->SetCategoryBits(ROBOT_CATEGORY | BELT_CATEGORY | LEFT_CATEGORY);
  }
  for (physics::CollisionPtr collision : this->rightTrack->GetCollisions()) {
    collision->SetCategoryBits(ROBOT_CATEGORY | BELT_CATEGORY);
  }

}

void SimpleTrackedVehiclePlugin::driveTracks(const common::UpdateInfo&)
{
  if (this->contactManager->GetContactCount() == 0)
  {
    return;
  }

  /////////////////////////////////////////////
  // Calculate the desired center of rotation
  /////////////////////////////////////////////

  const double leftBeltSpeed = this->leftTrackVelocity;
  const double rightBeltSpeed = this->rightTrackVelocity;

  // the desired linear and angular speeds (set by desired track velocities)
  const double linearSpeed = (leftBeltSpeed + rightBeltSpeed) / 2;
  // for whatever reason, the angular speed needs to be negated here
  const double angularSpeed = -(leftBeltSpeed - rightBeltSpeed) *
      this->GetSteeringEfficiency() / this->GetTracksSeparation();

  // radius of the turn the robot is doing
  const double desiredRotationRadiusSigned =
      (fabs(angularSpeed) < 0.1) ?
        dInfinity : // is driving straight
        (
          (fabs(linearSpeed) < 0.1) ?
            0 : // is rotating about a single point
            linearSpeed / angularSpeed // general movement
        );

  const ignition::math::Pose3d bodyPose = this->body->WorldPose();
  const ignition::math::Vector3d bodyYAxisGlobal =
      bodyPose.Rot().RotateVector(ignition::math::Vector3d(0, 1, 0));
  const ignition::math::Vector3d centerOfRotation =
      (bodyYAxisGlobal * desiredRotationRadiusSigned) + bodyPose.Pos();

  ////////////////////////////////////////////////////////////////////////
  // For each contact, compute the friction force direction and speed of
  // surface movement.
  ////////////////////////////////////////////////////////////////////////
  size_t i = 0;

  const std::vector<physics::Contact *> contacts =
      this->contactManager->GetContacts();

  for (physics::Contact *contact : contacts) {
    // Beware! There may be invalid contacts beyond GetContactCount()...
    if (i == this->contactManager->GetContactCount())
      break;

    ++i;

    if (contact->collision1->GetSurface()->collideWithoutContact ||
        contact->collision1->GetSurface()->collideWithoutContact) {
      continue;
    }

    if (!contact->collision1->GetLink()->GetEnabled() ||
        !contact->collision2->GetLink()->GetEnabled()) {
      continue;
    }

    if (contact->collision1->IsStatic() && contact->collision2->IsStatic()) {
      // we're not interested in static model collisions
      // (they do not have any ODE bodies).
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

    if (body1 == 0) {
      std::swap(body1, body2);
      std::swap(geom1, geom2);
    }

    // determine if track is the first or second collision element
    const bool isGeom1Track = (dGeomGetCategoryBits(geom1) & BELT_CATEGORY) > 0;
    const bool isGeom2Track = (dGeomGetCategoryBits(geom2) & BELT_CATEGORY) > 0;

    if (!isGeom1Track && !isGeom2Track) {
      continue;
    }

    // speed and geometry of the track in collision
    const dGeomID trackGeom = (isGeom1Track ? geom1 : geom2);
    const dReal beltSpeed = (dGeomGetCategoryBits(trackGeom) & LEFT_CATEGORY) ?
                            leftBeltSpeed : rightBeltSpeed;

    // remember if we've found at least one contact joint (we should!)
    bool foundContact = false;
    for (dContact_iterator contactIterator =
            dContact_iterator::begin(body1, geom1, geom2);
         contactIterator != dContact_iterator::end();
         ++contactIterator) {

      dContact* odeContact = *contactIterator;

      // now we're sure it is a contact between our two geometries
      foundContact = true;

      const ignition::math::Vector3d contactNormal(odeContact->geom.normal[0],
        odeContact->geom.normal[1], odeContact->geom.normal[2]);

      // vector tangent to the belt pointing in the belt's movement direction
      ignition::math::Vector3d beltDirection(
          contactNormal.Cross(bodyYAxisGlobal));

      if (beltSpeed > 0) {
        beltDirection = -beltDirection;
      }

      const ignition::math::Vector3d frictionDirection =
          this->computeFrictionDirection(linearSpeed, angularSpeed,
            desiredRotationRadiusSigned == dInfinity, bodyPose, bodyYAxisGlobal,
            centerOfRotation, odeContact, beltDirection);
      odeContact->fdir1[0] = frictionDirection.X();
      odeContact->fdir1[1] = frictionDirection.Y();
      odeContact->fdir1[2] = frictionDirection.Z();

      // use friction direction and motion1 to simulate the track movement
      odeContact->surface.mode |= dContactFDir1 | dContactMotion1;

      // the dot product <beltDirection,fdir1> is the cosine of the angle they
      // form (because both are unit vectors)
      odeContact->surface.motion1 = this->computeSurfaceMotion(
          beltSpeed, beltDirection, frictionDirection);
    }

    if (!foundContact) {
      gzwarn << "No ODE contact joint found for contact " <<
        contact->DebugString() << std::endl;
      continue;
    }
  }
}

ignition::math::Vector3d SimpleTrackedVehiclePlugin::computeFrictionDirection(
    const double linearSpeed, const double angularSpeed,
    const bool drivingStraight, const ignition::math::Pose3d &bodyPose,
    const ignition::math::Vector3d &bodyYAxisGlobal,
    const ignition::math::Vector3d &centerOfRotation,
    const dContact* odeContact,
    const ignition::math::Vector3d &beltDirection) const {

  ignition::math::Vector3d frictionDirection;

  const ignition::math::Vector3d contactNormal(odeContact->geom.normal[0],
    odeContact->geom.normal[1], odeContact->geom.normal[2]);

  if (!drivingStraight) { // non-straight drive

    const ignition::math::Vector3d contactPos(odeContact->geom.pos[0],
      odeContact->geom.pos[1], odeContact->geom.pos[2]);

    // vector pointing from the center of rotation to the contact point
    const ignition::math::Vector3d COR2Contact =
        (contactPos - centerOfRotation).Normalize();

    // the friction force should be perpendicular to COR2Contact
    frictionDirection = contactNormal.Cross(COR2Contact);

    // position of the contact point relative to vehicle body
    const ignition::math::Vector3d contactInVehiclePos(
        bodyPose.Rot().RotateVectorReverse(contactPos - bodyPose.Pos()));

    const dReal linearSpeedSignum =
        (fabs(linearSpeed) > 0.1) ? this->sgn(linearSpeed) : 1;

    // contactInVehiclePos.Dot(ignition::math::Vector3d(1, 0, 0)) > 0 means
    // the contact is "in front" of the line on which COR moves
    if ((int)(this->sgn(angularSpeed) *
            this->sgn(bodyYAxisGlobal.Dot(frictionDirection))) !=
        (int)((linearSpeedSignum) * this->sgn(contactInVehiclePos.Dot(
            ignition::math::Vector3d(1, 0, 0))))) {
      frictionDirection = -frictionDirection;
    }

    if (linearSpeed < 0) {
      frictionDirection = - frictionDirection;
    }

  } else { // straight drive

    frictionDirection = contactNormal.Cross(bodyYAxisGlobal);

    if (frictionDirection.Dot(beltDirection) < 0) {
      frictionDirection = -frictionDirection;
    }
  }
  return frictionDirection;
}

double SimpleTrackedVehiclePlugin::computeSurfaceMotion(const double beltSpeed,
    const ignition::math::Vector3d &beltDirection,
    const ignition::math::Vector3d &frictionDirection) const {

  return -beltDirection.Dot(frictionDirection) * fabs(beltSpeed);

}

// dContactIterator main code

SimpleTrackedVehiclePlugin::dContact_iterator::self_type
SimpleTrackedVehiclePlugin::dContact_iterator::operator++()
{
  // initialized && null contact means we've reached the end of the iterator
  if (this->initialized && this->currentContact == nullptr) {
    return *this;
  }

  // I haven't found a nice way to get ODE ID of the collision joint,
  // so we need to iterate over all joints connecting the two colliding
  // bodies and try to find the one we're interested in.
  // This should not be a performance issue, since bodies connected by other
  // joint types do not collide by default.

  // remember if we've found at least one contact joint (we should!)
  bool found = false;
  for (; this->jointIndex < (size_t)dBodyGetNumJoints(this->body);
         this->jointIndex++) {
    const dJointID joint = dBodyGetJoint(this->body, (int)this->jointIndex);

    // only interested in contact joints
    if (dJointGetType(joint) != dJointTypeContact) {
      continue;
    }

    // HACK uncomment if dxJointContact->contact offset changes in the bundled
    // ODE to find out its new address.
    // Here we access ODE's private class dxJointContact, so it's needed to add
    // deps/opende/src temporarily to include dirs.
    // The line to uncomment follows.
    //dContact* odeContact = & static_cast<dxJointContact*>(contactJoint)->contact;

    // offset of "contact" member inside dxJointContact has been 0xE8 in the
    // ODE bundled with Gazebo 8.0
    dContact* odeContact = (dContact*) ((uint8_t*) joint + 0xE8);
    if (!(
            odeContact->geom.g1 == this->geom1 &&
            odeContact->geom.g2 == this->geom2
         ) &&
        !(
            odeContact->geom.g1 == this->geom2 &&
            odeContact->geom.g2 == this->geom1
         )
        ) {
      continue; // not a contact between our two geometries
    }

    // we found a contact we're interested in

    found = true;
    this->initialized = true;

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wreturn-stack-address"
    this->currentContact = &odeContact;
#pragma clang diagnostic pop

    this->jointIndex++; // needed since we break out of the for-loop
    break;
  }

  if (!found) {
    // we've reached the end of the iterator
    this->currentContact = nullptr;
    this->initialized = true;
  }

  this->initialized = true;
  return *this;
}

// Implementation of boring parts of the dContactIterator

SimpleTrackedVehiclePlugin::dContact_iterator::dContact_iterator() :
    currentContact(nullptr), jointIndex(0), body(0), geom1(0), geom2(0),
    initialized(false)
{
}

SimpleTrackedVehiclePlugin::dContact_iterator::dContact_iterator(
    bool _initialized) : currentContact(nullptr), initialized(_initialized)
{
}

SimpleTrackedVehiclePlugin::dContact_iterator::dContact_iterator(
    const SimpleTrackedVehiclePlugin::dContact_iterator::self_type &rhs)
{
  this->currentContact = rhs.currentContact;
  this->initialized = rhs.initialized;
  this->jointIndex = rhs.jointIndex;
  this->body = rhs.body;
  this->geom1 = rhs.geom1;
  this->geom2 = rhs.geom2;
}

SimpleTrackedVehiclePlugin::dContact_iterator::dContact_iterator(
    dBodyID _body, dGeomID _geom1, dGeomID _geom2) :
    currentContact(nullptr), jointIndex(0), body(_body),
    geom1(_geom1), geom2(_geom2), initialized(false)
{
}

SimpleTrackedVehiclePlugin::dContact_iterator::self_type
SimpleTrackedVehiclePlugin::dContact_iterator::begin(
    dBodyID _body, dGeomID _geom1, dGeomID _geom2)
{
  return dContact_iterator(_body, _geom1, _geom2);
}

SimpleTrackedVehiclePlugin::dContact_iterator::self_type
SimpleTrackedVehiclePlugin::dContact_iterator::end()
{
  return dContact_iterator(true);
}

bool SimpleTrackedVehiclePlugin::dContact_iterator::operator==(
    const SimpleTrackedVehiclePlugin::dContact_iterator::self_type &rhs)
{
  if (this->currentContact == nullptr && !this->initialized)
  {
    ++(*this);
  }
  return this->currentContact == rhs.currentContact &&
         this->initialized == rhs.initialized;
}

void SimpleTrackedVehiclePlugin::dContact_iterator::operator=(
    const SimpleTrackedVehiclePlugin::dContact_iterator::self_type &rhs)
{
  this->currentContact = rhs.currentContact;
  this->initialized = rhs.initialized;
  this->jointIndex = rhs.jointIndex;
  this->body = rhs.body;
  this->geom1 = rhs.geom1;
  this->geom2 = rhs.geom2;
}

SimpleTrackedVehiclePlugin::dContact_iterator::self_type
SimpleTrackedVehiclePlugin::dContact_iterator::operator++(int)
{
  self_type i = *this;
  ++(*this);
  return i;
}

dContact *&SimpleTrackedVehiclePlugin::dContact_iterator::operator*()
{
  if (!this->initialized)
  {
    ++(*this);
  }
  return *this->currentContact;
}

dContact** SimpleTrackedVehiclePlugin::dContact_iterator::operator->()
{
  if (!this->initialized)
  {
    ++(*this);
  }
  return this->currentContact;
}

bool SimpleTrackedVehiclePlugin::dContact_iterator::operator!=(
    const SimpleTrackedVehiclePlugin::dContact_iterator::self_type &rhs)
{
  return !SimpleTrackedVehiclePlugin::dContact_iterator::operator==(rhs);
}