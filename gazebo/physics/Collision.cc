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
#include "gazebo/physics/Collision.hh"

using namespace gazebo;
using namespace physics;

// Class used to initialize an sdf element pointer from "collision.sdf".
// This is then used in the Collision constructor to improve performance.
class SDFCollisionInitializer
{
  // Constructor that create the collision sdf element
  public: SDFCollisionInitializer()
  {
    sdf::initFile("collision.sdf", collisionSDF);
  }

  // Pointer the collision sdf element
  public: sdf::ElementPtr collisionSDF = std::make_shared<sdf::Element>();
};
static SDFCollisionInitializer g_SDFInit;


//////////////////////////////////////////////////
Collision::Collision(LinkPtr _link)
: Entity(_link), maxContacts(1), worldPoseDirty(true)
{
  this->AddType(Base::COLLISION);

  this->link = _link;

  this->placeable = false;

  this->sdf->Copy(g_SDFInit.collisionSDF);

  this->collisionVisualId = physics::getUniqueId();
}

//////////////////////////////////////////////////
Collision::~Collision()
{
  this->Fini();
}

//////////////////////////////////////////////////
void Collision::Fini()
{
  if (this->requestPub)
  {
    msgs::Request *msg = msgs::CreateRequest("entity_delete",
        this->GetScopedName()+"__COLLISION_VISUAL__");
    this->requestPub->Publish(*msg, true);
    delete msg;
  }

  this->link.reset();
  this->shape.reset();
  this->surface.reset();

  Entity::Fini();
}

//////////////////////////////////////////////////
void Collision::Load(sdf::ElementPtr _sdf)
{
  auto *linkDom = this->GetLink()->GetSDFDom();
  if (nullptr != linkDom)
  {
    this->collisionSDFDom =
      linkDom->CollisionByName(_sdf->Get<std::string>("name"));
  }

  Entity::Load(_sdf);

  this->maxContacts = _sdf->Get<unsigned int>("max_contacts");
  this->SetMaxContacts(this->maxContacts);

  if (this->sdf->HasElement("laser_retro"))
    this->SetLaserRetro(this->sdf->Get<double>("laser_retro"));

  this->SetRelativePose(this->SDFPoseRelativeToParent());
  this->surface->Load(this->sdf->GetElement("surface"));

  if (this->shape)
    this->shape->Load(this->sdf->GetElement("geometry")->GetFirstElement());
  else
    gzwarn << "No shape has been specified. Error!!!\n";
}

//////////////////////////////////////////////////
void Collision::Init()
{
  this->shape->Init();

  this->SetRelativePose(this->SDFPoseRelativeToParent());
}

//////////////////////////////////////////////////
void Collision::SetCollision(bool _placeable)
{
  this->SetPlaceable(_placeable);

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
void Collision::SetPlaceable(const bool _placeable)
{
  this->placeable = _placeable;
}

//////////////////////////////////////////////////
bool Collision::IsPlaceable() const
{
  return this->placeable;
}


//////////////////////////////////////////////////
void Collision::SetLaserRetro(float _retro)
{
  this->sdf->GetElement("laser_retro")->Set(_retro);
  this->laserRetro = _retro;
}

//////////////////////////////////////////////////
float Collision::GetLaserRetro() const
{
  return this->laserRetro;
}

//////////////////////////////////////////////////
LinkPtr Collision::GetLink() const
{
  return this->link;
}

//////////////////////////////////////////////////
ModelPtr Collision::GetModel() const
{
  return this->link->GetModel();
}

//////////////////////////////////////////////////
unsigned int Collision::GetShapeType() const
{
  return this->shape->GetType();
}

//////////////////////////////////////////////////
void Collision::SetShape(ShapePtr _shape)
{
  this->shape = _shape;
}

//////////////////////////////////////////////////
ShapePtr Collision::GetShape() const
{
  return this->shape;
}

//////////////////////////////////////////////////
void Collision::SetScale(const ignition::math::Vector3d &_scale)
{
  this->shape->SetScale(_scale);
}

//////////////////////////////////////////////////
ignition::math::Vector3d Collision::RelativeLinearVel() const
{
  if (this->link)
    return this->link->RelativeLinearVel();
  else
    return ignition::math::Vector3d::Zero;
}

//////////////////////////////////////////////////
ignition::math::Vector3d Collision::WorldLinearVel() const
{
  if (this->link)
    return this->link->WorldLinearVel();
  else
    return ignition::math::Vector3d::Zero;
}

//////////////////////////////////////////////////
ignition::math::Vector3d Collision::RelativeAngularVel() const
{
  if (this->link)
    return this->link->RelativeAngularVel();
  else
    return ignition::math::Vector3d::Zero;
}

//////////////////////////////////////////////////
ignition::math::Vector3d Collision::WorldAngularVel() const
{
  if (this->link)
    return this->link->WorldAngularVel();
  else
    return ignition::math::Vector3d::Zero;
}

//////////////////////////////////////////////////
ignition::math::Vector3d Collision::RelativeLinearAccel() const
{
  if (this->link)
    return this->link->RelativeLinearAccel();
  else
    return ignition::math::Vector3d::Zero;
}

//////////////////////////////////////////////////
ignition::math::Vector3d Collision::WorldLinearAccel() const
{
  if (this->link)
    return this->link->WorldLinearAccel();
  else
    return ignition::math::Vector3d::Zero;
}

//////////////////////////////////////////////////
ignition::math::Vector3d Collision::RelativeAngularAccel() const
{
  if (this->link)
    return this->link->RelativeAngularAccel();
  else
    return ignition::math::Vector3d::Zero;
}

//////////////////////////////////////////////////
ignition::math::Vector3d Collision::WorldAngularAccel() const
{
  if (this->link)
    return this->link->WorldAngularAccel();
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
  _msg.set_id(this->GetId());
  _msg.set_name(this->GetScopedName());
  _msg.set_laser_retro(this->GetLaserRetro());

  this->shape->FillMsg(*_msg.mutable_geometry());
  this->surface->FillMsg(*_msg.mutable_surface());

  msgs::Set(this->visualMsg->mutable_pose(), this->RelativePose());

  if (!this->HasType(physics::Base::SENSOR_COLLISION))
  {
    _msg.add_visual()->CopyFrom(*this->visualMsg);
    // TODO remove the need to create the special collision visual msg and
    // let the gui handle this.
    _msg.add_visual()->CopyFrom(this->CreateCollisionVisual());
  }
}

//////////////////////////////////////////////////
void Collision::ProcessMsg(const msgs::Collision &_msg)
{
  if (_msg.id() != this->GetId())
  {
    gzerr << "Incorrect ID\n";
    return;
  }

  this->SetName(_msg.name());
  if (_msg.has_laser_retro())
    this->SetLaserRetro(_msg.laser_retro());

  if (_msg.has_pose())
  {
    this->link->SetEnabled(true);
    this->SetRelativePose(msgs::ConvertIgn(_msg.pose()));
  }

  if (_msg.has_geometry())
  {
    this->link->SetEnabled(true);
    this->shape->ProcessMsg(_msg.geometry());
  }

  if (_msg.has_surface())
  {
    this->link->SetEnabled(true);
    this->surface->ProcessMsg(_msg.surface());
  }
}

/////////////////////////////////////////////////
msgs::Visual Collision::CreateCollisionVisual()
{
  msgs::Visual msg;
  msg.set_name(this->GetScopedName()+"__COLLISION_VISUAL__");

  // Put in a unique ID because this is a special visual.
  msg.set_id(this->collisionVisualId);
  msg.set_parent_name(this->parent->GetScopedName());
  msg.set_parent_id(this->parent->GetId());
  msg.set_is_static(this->IsStatic());
  msg.set_cast_shadows(false);
  msg.set_type(msgs::Visual::COLLISION);
  msgs::Set(msg.mutable_pose(), this->RelativePose());
  msg.mutable_material()->mutable_script()->add_uri(
      "file://media/materials/scripts/gazebo.material");
  msg.mutable_material()->mutable_script()->set_name(
      "Gazebo/OrangeTransparent");
  msgs::Geometry *geom = msg.mutable_geometry();
  geom->CopyFrom(msgs::GeometryFromSDF(this->sdf->GetElement("geometry")));

  return msg;
}

/////////////////////////////////////////////////
CollisionState Collision::GetState()
{
  return this->state;
}

/////////////////////////////////////////////////
void Collision::SetState(const CollisionState &_state)
{
  this->SetRelativePose(_state.Pose());
}

/////////////////////////////////////////////////
void Collision::SetMaxContacts(unsigned int _maxContacts)
{
  this->maxContacts = _maxContacts;
  this->sdf->GetElement("max_contacts")->GetValue()->Set(_maxContacts);
}

/////////////////////////////////////////////////
unsigned int Collision::GetMaxContacts()
{
  return this->maxContacts;
}

/////////////////////////////////////////////////
const ignition::math::Pose3d &Collision::WorldPose() const
{
  // If true, compute a new world pose value.
  //
  if (this->worldPoseDirty)
  {
    this->worldPose = this->InitialRelativePose() + this->link->WorldPose();
    this->worldPoseDirty = false;
  }

  return this->worldPose;
}

/////////////////////////////////////////////////
void Collision::SetWorldPoseDirty()
{
  // Tell the collision object that the next call to ::GetWorldPose should
  // compute a new worldPose value.
  this->worldPoseDirty = true;
}

std::optional<sdf::SemanticPose> Collision::SDFSemanticPose() const
{
  if (nullptr != this->collisionSDFDom)
  {
    return this->collisionSDFDom->SemanticPose();
  }
  return std::nullopt;
}
