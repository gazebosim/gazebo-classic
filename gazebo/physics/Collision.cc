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
#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
  #include <Winsock2.h>
#endif

#include <sstream>

#include "gazebo/msgs/msgs.hh"
#include "gazebo/msgs/MessageTypes.hh"

#include "gazebo/common/Events.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/transport/TransportIface.hh"

#include "gazebo/transport/Publisher.hh"

#include "gazebo/physics/World.hh"
#include "gazebo/physics/ContactManager.hh"
#include "gazebo/physics/PhysicsIface.hh"
#include "gazebo/physics/Contact.hh"
#include "gazebo/physics/Shape.hh"
#include "gazebo/physics/SurfaceParams.hh"
#include "gazebo/physics/Model.hh"
#include "gazebo/physics/Link.hh"
#include "gazebo/physics/CollisionPrivate.hh"
#include "gazebo/physics/Collision.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
Collision::Collision(LinkPtr _link)
: Entity(*new CollisionPrivate, _link),
  collDPtr(static_cast<CollisionPrivate*>(this->entityDPtr))
{
  this->ConstructionHelper(_link);
}

//////////////////////////////////////////////////
Collision::Collision(CollisionPrivate &_dataPtr, LinkPtr _link)
: Entity(_dataPtr, _link),
  collDPtr(static_cast<CollisionPrivate*>(this->entityDPtr))
{
  this->ConstructionHelper(_link);
}

/////////////////////////////////////////////////
void Collision::ConstructionHelper(LinkPtr _link)
{
  this->AddType(Base::COLLISION);

  this->collDPtr->link = _link;

  this->collDPtr->placeable = false;

  sdf::initFile("collision.sdf", this->collDPtr->sdf);

  this->collDPtr->collisionVisualId = physics::getUniqueId();
}

//////////////////////////////////////////////////
Collision::~Collision()
{
}

//////////////////////////////////////////////////
void Collision::Fini()
{
  if (this->collDPtr->requestPub)
  {
    msgs::Request *msg = msgs::CreateRequest("entity_delete",
        this->ScopedName()+"__COLLISION_VISUAL__");
    this->collDPtr->requestPub->Publish(*msg, true);
  }

  Entity::Fini();
  this->collDPtr->link.reset();
  this->collDPtr->shape.reset();
  this->collDPtr->surface.reset();
}

//////////////////////////////////////////////////
void Collision::Load(sdf::ElementPtr _sdf)
{
  Entity::Load(_sdf);

  this->collDPtr->maxContacts = _sdf->Get<unsigned int>("max_contacts");
  this->SetMaxContacts(this->collDPtr->maxContacts);

  if (this->collDPtr->sdf->HasElement("laser_retro"))
    this->SetLaserRetro(this->collDPtr->sdf->Get<double>("laser_retro"));

  this->SetRelativePose(
      this->collDPtr->sdf->Get<ignition::math::Pose3d>("pose"));

  this->collDPtr->surface->Load(this->collDPtr->sdf->GetElement("surface"));

  if (this->collDPtr->shape)
  {
    this->collDPtr->shape->Load(
        this->collDPtr->sdf->GetElement("geometry")->GetFirstElement());
  }
  else
    gzwarn << "No shape has been specified. Error!!!\n";
}

//////////////////////////////////////////////////
void Collision::Init()
{
  this->collDPtr->shape->Init();

  this->SetRelativePose(
    this->collDPtr->sdf->Get<ignition::math::Pose3d>("pose"));
}

//////////////////////////////////////////////////
void Collision::SetCollision(const bool _placeable)
{
  this->collDPtr->placeable = _placeable;

  if (this->IsStatic())
  {
    this->SetCategoryBits(GZ_FIXED_COLLIDE);
    this->SetCollideBits(~GZ_FIXED_COLLIDE);
  }
  else
  {
    // collide with all
    this->SetCategoryBits(GZ_ALL_COLLIDE);
    this->SetCollideBits(GZ_ALL_COLLIDE);
  }
}

//////////////////////////////////////////////////
bool Collision::IsPlaceable() const
{
  return this->collDPtr->placeable;
}


//////////////////////////////////////////////////
void Collision::SetLaserRetro(const float _retro)
{
  this->collDPtr->sdf->GetElement("laser_retro")->Set(_retro);
  this->collDPtr->laserRetro = _retro;
}

//////////////////////////////////////////////////
float Collision::GetLaserRetro() const
{
  return this->LaserRetro();
}

//////////////////////////////////////////////////
float Collision::LaserRetro() const
{
  return this->collDPtr->laserRetro;
}

//////////////////////////////////////////////////
LinkPtr Collision::GetLink() const
{
  return this->Link();
}

//////////////////////////////////////////////////
LinkPtr Collision::Link() const
{
  return this->collDPtr->link;
}

//////////////////////////////////////////////////
ModelPtr Collision::GetModel() const
{
  return ModelPtr(this->ParentModel());
}

//////////////////////////////////////////////////
Model *Collision::ParentModel() const
{
  return this->collDPtr->link->ParentModel();
}

//////////////////////////////////////////////////
unsigned int Collision::GetShapeType() const
{
  return this->ShapeType();
}

//////////////////////////////////////////////////
unsigned int Collision::ShapeType() const
{
  return this->collDPtr->shape->Type();
}

//////////////////////////////////////////////////
void Collision::SetShape(ShapePtr _shape)
{
  this->collDPtr->shape = _shape;
}

//////////////////////////////////////////////////
ShapePtr Collision::GetShape() const
{
  return this->Shape();
}

//////////////////////////////////////////////////
ShapePtr Collision::Shape() const
{

  return this->collDPtr->shape;
}

//////////////////////////////////////////////////
void Collision::SetScale(const math::Vector3 &_scale)
{
  this->collDPtr->shape->SetScale(_scale.Ign());
}

//////////////////////////////////////////////////
math::Vector3 Collision::GetRelativeLinearVel() const
{
  return this->RelativeLinearVel();
}

//////////////////////////////////////////////////
ignition::math::Vector3d Collision::RelativeLinearVel() const
{
  if (this->collDPtr->link)
    return this->collDPtr->link->RelativeLinearVel();
  else
    return ignition::math::Vector3d::Zero;
}

//////////////////////////////////////////////////
math::Vector3 Collision::GetWorldLinearVel() const
{
  return this->WorldLinearVel();
}

//////////////////////////////////////////////////
ignition::math::Vector3d Collision::WorldLinearVel() const
{
  if (this->collDPtr->link)
    return this->collDPtr->link->WorldLinearVel();
  else
    return ignition::math::Vector3d::Zero;
}

//////////////////////////////////////////////////
math::Vector3 Collision::GetRelativeAngularVel() const
{
  return this->RelativeAngularVel();
}

//////////////////////////////////////////////////
ignition::math::Vector3d Collision::RelativeAngularVel() const
{
  if (this->collDPtr->link)
    return this->collDPtr->link->RelativeAngularVel();
  else
    return ignition::math::Vector3d::Zero;
}

//////////////////////////////////////////////////
math::Vector3 Collision::GetWorldAngularVel() const
{
  return this->WorldAngularVel();
}

//////////////////////////////////////////////////
ignition::math::Vector3d Collision::WorldAngularVel() const
{
  if (this->collDPtr->link)
    return this->collDPtr->link->WorldAngularVel();
  else
    return ignition::math::Vector3d::Zero;
}

//////////////////////////////////////////////////
math::Vector3 Collision::GetRelativeLinearAccel() const
{
  return this->RelativeLinearAccel();
}

//////////////////////////////////////////////////
ignition::math::Vector3d Collision::RelativeLinearAccel() const
{
  if (this->collDPtr->link)
    return this->collDPtr->link->RelativeLinearAccel();
  else
    return ignition::math::Vector3d::Zero;
}

//////////////////////////////////////////////////
math::Vector3 Collision::GetWorldLinearAccel() const
{
  return this->WorldLinearAccel();
}

//////////////////////////////////////////////////
ignition::math::Vector3d Collision::WorldLinearAccel() const
{
  if (this->collDPtr->link)
    return this->collDPtr->link->WorldLinearAccel();
  else
    return ignition::math::Vector3d::Zero;
}

//////////////////////////////////////////////////
math::Vector3 Collision::GetRelativeAngularAccel() const
{
  return this->RelativeAngularAccel();
}

//////////////////////////////////////////////////
ignition::math::Vector3d Collision::RelativeAngularAccel() const
{
  if (this->collDPtr->link)
    return this->collDPtr->link->RelativeAngularAccel();
  else
    return ignition::math::Vector3d::Zero;
}

//////////////////////////////////////////////////
math::Vector3 Collision::GetWorldAngularAccel() const
{
  return this->WorldAngularAccel();
}

//////////////////////////////////////////////////
ignition::math::Vector3d Collision::WorldAngularAccel() const
{
  if (this->collDPtr->link)
    return this->collDPtr->link->WorldAngularAccel();
  else
    return ignition::math::Vector3d::Zero;
}

//////////////////////////////////////////////////
void Collision::UpdateParameters(sdf::ElementPtr _sdf)
{
  Entity::UpdateParameters(_sdf);
}

//////////////////////////////////////////////////
void Collision::FillMsg(msgs::Collision &_msg)
{
  msgs::Set(_msg.mutable_pose(), this->RelativePose());
  _msg.set_id(this->Id());
  _msg.set_name(this->ScopedName());
  _msg.set_laser_retro(this->LaserRetro());

  this->collDPtr->shape->FillMsg(*_msg.mutable_geometry());
  this->collDPtr->surface->FillMsg(*_msg.mutable_surface());

  msgs::Set(this->collDPtr->visualMsg->mutable_pose(), this->RelativePose());

  if (!this->HasType(physics::Base::SENSOR_COLLISION))
  {
    _msg.add_visual()->CopyFrom(*this->collDPtr->visualMsg);
    // TODO remove the need to create the special collision visual msg and
    // let the gui handle this.
    _msg.add_visual()->CopyFrom(this->CreateCollisionVisual());
  }
}

//////////////////////////////////////////////////
void Collision::ProcessMsg(const msgs::Collision &_msg)
{
  if (_msg.id() != this->Id())
  {
    gzerr << "Incorrect ID\n";
    return;
  }

  this->SetName(_msg.name());
  if (_msg.has_laser_retro())
    this->SetLaserRetro(_msg.laser_retro());

  if (_msg.has_pose())
  {
    this->collDPtr->link->SetEnabled(true);
    this->SetRelativePose(msgs::ConvertIgn(_msg.pose()));
  }

  if (_msg.has_geometry())
  {
    this->collDPtr->link->SetEnabled(true);
    this->collDPtr->shape->ProcessMsg(_msg.geometry());
  }

  if (_msg.has_surface())
  {
    this->collDPtr->link->SetEnabled(true);
    this->collDPtr->surface->ProcessMsg(_msg.surface());
  }
}

/////////////////////////////////////////////////
msgs::Visual Collision::CreateCollisionVisual()
{
  msgs::Visual msg;
  msg.set_name(this->ScopedName()+"__COLLISION_VISUAL__");

  // Put in a unique ID because this is a special visual.
  msg.set_id(this->collDPtr->collisionVisualId);
  msg.set_parent_name(this->collDPtr->parent->ScopedName());
  msg.set_parent_id(this->collDPtr->parent->Id());
  msg.set_is_static(this->IsStatic());
  msg.set_cast_shadows(false);
  msg.set_type(msgs::Visual::COLLISION);
  msgs::Set(msg.mutable_pose(), this->RelativePose());
  msg.mutable_material()->mutable_script()->add_uri(
      "file://media/materials/scripts/gazebo.material");
  msg.mutable_material()->mutable_script()->set_name(
      "Gazebo/OrangeTransparent");
  msgs::Geometry *geom = msg.mutable_geometry();
  geom->CopyFrom(msgs::GeometryFromSDF(
        this->collDPtr->sdf->GetElement("geometry")));

  return msg;
}

/////////////////////////////////////////////////
CollisionState Collision::GetState()
{
  return this->collDPtr->state;
}

/////////////////////////////////////////////////
CollisionState Collision::State() const
{
  return this->collDPtr->state;
}

/////////////////////////////////////////////////
void Collision::SetState(const CollisionState &_state)
{
  this->SetRelativePose(_state.Pose());
}

/////////////////////////////////////////////////
void Collision::SetMaxContacts(const unsigned int _maxContacts)
{
  this->collDPtr->maxContacts = _maxContacts;
  this->collDPtr->sdf->GetElement(
      "max_contacts")->GetValue()->Set(_maxContacts);
}

/////////////////////////////////////////////////
unsigned int Collision::GetMaxContacts()
{
  return this->MaxContacts();
}

/////////////////////////////////////////////////
unsigned int Collision::MaxContacts() const
{
  return this->collDPtr->maxContacts;
}

/////////////////////////////////////////////////
const ignition::math::Pose3d &Collision::WorldPose() const
{
  // If true, compute a new world pose value.
  //
  if (this->collDPtr->worldPoseDirty)
  {
    this->collDPtr->worldPose = this->InitialRelativePose() +
                      this->collDPtr->link->WorldPose();
    this->collDPtr->worldPoseDirty = false;
  }

  return this->collDPtr->worldPose;
}

/////////////////////////////////////////////////
void Collision::SetWorldPoseDirty()
{
  // Tell the collision object that the next call to ::WorldPose should
  // compute a new worldPose value.
  this->collDPtr->worldPoseDirty = true;
}

/////////////////////////////////////////////////
SurfaceParamsPtr Collision::Surface() const
{
  return this->collDPtr->surface;
}
