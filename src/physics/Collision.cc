/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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
/* Desc: Collision class
 * Author: Nate Koenig
 * Date: 13 Feb 2006
 */

#include <sstream>

#include "msgs/msgs.h"
#include "msgs/MessageTypes.hh"

#include "common/Events.hh"
#include "common/Console.hh"

#include "transport/Publisher.hh"

#include "physics/Contact.hh"
#include "physics/Shape.hh"
#include "physics/BoxShape.hh"
#include "physics/CylinderShape.hh"
#include "physics/TrimeshShape.hh"
#include "physics/SphereShape.hh"
#include "physics/SurfaceParams.hh"
#include "physics/Model.hh"
#include "physics/Link.hh"
#include "physics/Collision.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
Collision::Collision(LinkPtr _link)
    : Entity(_link)
{
  this->AddType(Base::COLLISION);

  this->link = _link;

  this->transparency = 0;
  this->contactsEnabled = false;

  this->surface.reset(new SurfaceParams());

  this->connections.push_back(
      event::Events::ConnectShowBoundingBoxes(
        boost::bind(&Collision::ShowBoundingBox, this, _1)));
  this->connections.push_back(
      this->link->ConnectEnabled(
        boost::bind(&Collision::EnabledCB, this, _1)));
}

//////////////////////////////////////////////////
Collision::~Collision()
{
  if (!this->bbVisual.empty())
  {
    msgs::Visual msg;
    msg.set_name(this->bbVisual);
    msg.set_delete_me(true);
    this->visPub->Publish(msg);
  }
}

//////////////////////////////////////////////////
void Collision::Fini()
{
  msgs::Request *msg = msgs::CreateRequest("entity_delete",
      this->GetScopedName()+"__COLLISION_VISUAL__");
  this->requestPub->Publish(*msg, true);

  Entity::Fini();
  this->link.reset();
  this->shape.reset();
  this->surface.reset();
  this->connections.clear();
}



//////////////////////////////////////////////////
// First step in the loading process
void Collision::Load(sdf::ElementPtr &_sdf)
{
  Entity::Load(_sdf);

  if (this->sdf->HasElement("surface"))
    this->surface->Load(this->sdf->GetElement("surface"));

  if (this->shape)
    this->shape->Load(this->sdf->GetElement("geometry")->GetFirstElement());
  else
    gzwarn << "No shape has been specified. Error!!!\n";

  if (this->shape->GetType() != MULTIRAY_SHAPE &&
      this->shape->GetType() != RAY_SHAPE)
  {
    // this->visPub->Publish(this->CreateCollisionVisual());
  }
}

void Collision::Init()
{
  if (this->sdf->HasElement("origin"))
      this->SetRelativePose(
        this->sdf->GetElement("origin")->GetValuePose("pose"));
  else
    this->SetRelativePose(
        math::Pose(math::Vector3(0, 0, 0), math::Quaternion(0, 0, 0)));
  this->shape->Init();
}

//////////////////////////////////////////////////
// Create the bounding box for the collision
void Collision::CreateBoundingBox()
{
  // Create the bounding box
  if (this->GetShapeType() != PLANE_SHAPE && this->GetShapeType() != MAP_SHAPE)
  {
    math::Box box;

    box = this->GetBoundingBox();

    std::ostringstream visname;
    visname << this->GetScopedName() << "::BBVISUAL";

    msgs::Visual msg;
    msg.mutable_geometry()->set_type(msgs::Geometry::BOX);
    msg.set_parent_name(this->GetScopedName());
    msg.set_name(this->GetScopedName() + "_BBVISUAL");
    msg.set_cast_shadows(false);

    if (this->IsStatic())
      msg.mutable_material()->set_script("Gazebo/YellowTransparent");
    else
      msg.mutable_material()->set_script("Gazebo/GreenTransparent");

    msgs::Set(msg.mutable_geometry()->mutable_box()->mutable_size(),
               (box.max - box.min) * 1.05);
    msgs::Set(msg.mutable_pose()->mutable_position(),
              math::Vector3(0, 0, 0.0));
    msgs::Set(msg.mutable_pose()->mutable_orientation(),
              math::Quaternion(1, 0, 0, 0));
    msg.set_transparency(.5);
  }
}

//////////////////////////////////////////////////
void Collision::SetCollision(bool _placeable)
{
  this->placeable = _placeable;

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
  return this->placeable;
}


//////////////////////////////////////////////////
void Collision::SetLaserRetro(float _retro)
{
  this->sdf->GetAttribute("laser_retro")->Set(_retro);
}

//////////////////////////////////////////////////
float Collision::GetLaserRetro() const
{
  return this->sdf->GetValueDouble("laser_retro");
}


//////////////////////////////////////////////////
void Collision::ShowBoundingBox(const bool &_show)
{
  if (!this->bbVisual.empty())
  {
    msgs::Visual msg;
    msg.set_name(this->bbVisual);
    msg.set_visible(_show);
    msg.set_delete_me(true);
    // this->visPub->Publish(msg);
  }
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
// Get the shape type
unsigned int Collision::GetShapeType()
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
// Turn contact recording on or off
void Collision::SetContactsEnabled(bool _enable)
{
  this->contactsEnabled = _enable;
}

//////////////////////////////////////////////////
// Return true of contact recording is on
bool Collision::GetContactsEnabled() const
{
  return this->contactsEnabled;
}

//////////////////////////////////////////////////
void Collision::AddContact(const Contact &_contact)
{
  if (!this->GetContactsEnabled() ||
      this->GetShapeType() == RAY_SHAPE ||
      this->GetShapeType() == PLANE_SHAPE)
    return;

  this->contact(this->GetName(), _contact);
}

//////////////////////////////////////////////////
void Collision::EnabledCB(bool _enabled)
{
  msgs::Visual msg;
  msg.set_name(this->bbVisual);

  if (_enabled)
    msg.mutable_material()->set_script("Gazebo/GreenTransparent");
  else
    msg.mutable_material()->set_script("Gazebo/RedTransparent");

  // this->visPub->Publish(msg);
}

//////////////////////////////////////////////////
math::Vector3 Collision::GetRelativeLinearVel() const
{
  if (this->link)
    return this->link->GetRelativeLinearVel();
  else
    return math::Vector3();
}

//////////////////////////////////////////////////
math::Vector3 Collision::GetWorldLinearVel() const
{
  if (this->link)
    return this->link->GetWorldLinearVel();
  else
    return math::Vector3();
}

//////////////////////////////////////////////////
math::Vector3 Collision::GetRelativeAngularVel() const
{
  if (this->link)
    return this->link->GetRelativeAngularVel();
  else
    return math::Vector3();
}

//////////////////////////////////////////////////
math::Vector3 Collision::GetWorldAngularVel() const
{
  if (this->link)
    return this->link->GetWorldAngularVel();
  else
    return math::Vector3();
}

//////////////////////////////////////////////////
math::Vector3 Collision::GetRelativeLinearAccel() const
{
  if (this->link)
    return this->link->GetRelativeLinearAccel();
  else
    return math::Vector3();
}

//////////////////////////////////////////////////
math::Vector3 Collision::GetWorldLinearAccel() const
{
  if (this->link)
    return this->link->GetWorldLinearAccel();
  else
    return math::Vector3();
}

//////////////////////////////////////////////////
math::Vector3 Collision::GetRelativeAngularAccel() const
{
  if (this->link)
    return this->link->GetRelativeAngularAccel();
  else
    return math::Vector3();
}

//////////////////////////////////////////////////
math::Vector3 Collision::GetWorldAngularAccel() const
{
  if (this->link)
    return this->link->GetWorldAngularAccel();
  else
    return math::Vector3();
}

//////////////////////////////////////////////////
void Collision::UpdateParameters(sdf::ElementPtr &_sdf)
{
  Entity::UpdateParameters(_sdf);
}

//////////////////////////////////////////////////
void Collision::FillCollisionMsg(msgs::Collision &_msg)
{
  msgs::Set(_msg.mutable_pose(), this->GetRelativePose());
  _msg.set_id(this->GetId());
  _msg.set_name(this->GetName());
  _msg.set_laser_retro(this->GetLaserRetro());
  this->shape->FillShapeMsg(*_msg.mutable_geometry());
  this->surface->FillSurfaceMsg(*_msg.mutable_surface());

  _msg.add_visual()->CopyFrom(*this->visualMsg);
  _msg.add_visual()->CopyFrom(this->CreateCollisionVisual());
}

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
    this->SetRelativePose(msgs::Convert(_msg.pose()));
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

msgs::Visual Collision::CreateCollisionVisual()
{
  msgs::Visual msg;
  msg.set_name(this->GetScopedName()+"__COLLISION_VISUAL__");
  msg.set_parent_name(this->GetScopedName());
  msg.set_is_static(this->IsStatic());
  msg.set_cast_shadows(false);
  msg.mutable_material()->set_script("Gazebo/OrangeTransparent");
  msgs::Geometry *geom = msg.mutable_geometry();

  if (this->shape->HasType(BOX_SHAPE))
  {
    BoxShape *box = static_cast<BoxShape*>(this->shape.get());
    geom->set_type(msgs::Geometry::BOX);
    math::Vector3 size = box->GetSize();
    msgs::Set(geom->mutable_box()->mutable_size(), size);
  }
  else if (this->shape->HasType(CYLINDER_SHAPE))
  {
    CylinderShape *cyl = static_cast<CylinderShape*>(this->shape.get());
    msg.mutable_geometry()->set_type(msgs::Geometry::CYLINDER);
    geom->mutable_cylinder()->set_radius(cyl->GetRadius());
    geom->mutable_cylinder()->set_length(cyl->GetLength());
  }

  else if (this->shape->HasType(SPHERE_SHAPE))
  {
    SphereShape *sph = static_cast<SphereShape*>(this->shape.get());
    msg.mutable_geometry()->set_type(msgs::Geometry::SPHERE);
    geom->mutable_sphere()->set_radius(sph->GetRadius());
  }

  else if (this->shape->HasType(HEIGHTMAP_SHAPE))
  {
    msg.mutable_geometry()->set_type(msgs::Geometry::HEIGHTMAP);
  }

  else if (this->shape->HasType(MAP_SHAPE))
  {
    msg.mutable_geometry()->set_type(msgs::Geometry::IMAGE);
  }

  else if (this->shape->HasType(PLANE_SHAPE))
  {
    msg.mutable_geometry()->set_type(msgs::Geometry::PLANE);
  }
  else if (this->shape->HasType(TRIMESH_SHAPE))
  {
    TrimeshShape *msh = static_cast<TrimeshShape*>(this->shape.get());
    msg.mutable_geometry()->set_type(msgs::Geometry::MESH);
    math::Vector3 size = msh->GetSize();
    msgs::Set(geom->mutable_mesh()->mutable_scale(), size);
    geom->mutable_mesh()->set_filename(msh->GetFilename());
  }
  else
    gzerr << "Unknown shape[" << this->shape->GetType() << "]\n";

  msgs::Set(msg.mutable_pose(), this->GetRelativePose());

  return msg;
}


