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
#include "physics/SurfaceParams.hh"
#include "physics/Model.hh"
#include "physics/Link.hh"
#include "physics/Collision.hh"

using namespace gazebo;
using namespace physics;

////////////////////////////////////////////////////////////////////////////////
// Constructor
Collision::Collision( LinkPtr link )
    : Entity(link)
{
  this->AddType(Base::COLLISION);

  this->link = link;

  this->transparency = 0;
  this->contactsEnabled = false;

  this->connections.push_back( event::Events::ConnectShowBoundingBoxes( boost::bind(&Collision::ShowBoundingBox, this, _1) ) );
  this->connections.push_back( this->link->ConnectEnabled( boost::bind(&Collision::EnabledCB, this, _1) ) );
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
Collision::~Collision()
{
  if (!this->bbVisual.empty())
  {
    msgs::Visual msg;
    msg.set_name( this->bbVisual );
    msg.set_delete_me( true );
    this->visPub->Publish(msg);
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Finalize the collision
void Collision::Fini()
{
  Entity::Fini();
  this->link.reset();
  this->shape.reset();
  this->connections.clear();
}

////////////////////////////////////////////////////////////////////////////////
// First step in the loading process
void Collision::Load( sdf::ElementPtr &_sdf )
{
  Entity::Load(_sdf);

  if (this->shape)
    this->shape->Load(this->sdf->GetElement("geometry")->GetFirstElement());
  else
    gzwarn << "No shape has been specified. Error!!!\n";

  //TODO: this shouldn't exist in the physics sim
  //this->CreateBoundingBox();
}

void Collision::Init()
{
  this->SetContactsEnabled(false);
  this->SetRelativePose( 
      this->sdf->GetOrCreateElement("origin")->GetValuePose("pose") );
  this->shape->Init();
}

////////////////////////////////////////////////////////////////////////////////
// Create the bounding box for the collision
void Collision::CreateBoundingBox()
{
  // Create the bounding box
  if (this->GetShapeType() != PLANE_SHAPE && this->GetShapeType() != MAP_SHAPE)
  {
    math::Box box;

    box = this->GetBoundingBox();

    std::ostringstream visname;
    visname << this->GetCompleteScopedName() << "::BBVISUAL" ;

    msgs::Visual msg;
    msg.mutable_geometry()->set_type( msgs::Geometry::BOX );
    msg.set_parent_name( this->GetCompleteScopedName() );
    msg.set_name( this->GetCompleteScopedName() + "_BBVISUAL" );
    msg.set_cast_shadows(false);

    //msg.set_visible( RenderState::GetShowBoundingBoxes() );
    if (this->IsStatic() )
      msg.mutable_material()->set_script( "Gazebo/YellowTransparent" );
    else
      msg.mutable_material()->set_script( "Gazebo/GreenTransparent" );

    msgs::Set( msg.mutable_geometry()->mutable_box()->mutable_size(), 
               (box.max - box.min) * 1.05 );
    msgs::Set(msg.mutable_pose()->mutable_position(), math::Vector3(0,0,0.0));
    msgs::Set(msg.mutable_pose()->mutable_orientation(), math::Quaternion(1,0,0,0));
    msg.set_transparency( .5 );

    this->visPub->Publish(msg);
  }
}

////////////////////////////////////////////////////////////////////////////////
void Collision::SetCollision(bool placeable)
{
  this->placeable = placeable;

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

////////////////////////////////////////////////////////////////////////////////
// Return whether this is a placeable collision.
bool Collision::IsPlaceable() const
{
  return this->placeable;
}


////////////////////////////////////////////////////////////////////////////////
/// Set the laser retro reflectiveness
void Collision::SetLaserRetro(float retro)
{
  this->sdf->GetAttribute("laser_retro")->Set(retro);
}

////////////////////////////////////////////////////////////////////////////////
/// Get the laser retro reflectiveness
float Collision::GetLaserRetro() const
{
  return this->sdf->GetValueDouble("laser_retro");
}


////////////////////////////////////////////////////////////////////////////////
/// Set the visibility of the Bounding box
void Collision::ShowBoundingBox(const bool &show)
{
  if (!this->bbVisual.empty())
  {
    msgs::Visual msg;
    msg.set_name( this->bbVisual );
    msg.set_visible( show );
    msg.set_delete_me( true );
    this->visPub->Publish(msg);
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Get the link this collision belongs to
LinkPtr Collision::GetLink() const
{
  return this->link;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the model this collision belongs to
ModelPtr Collision::GetModel() const
{
  return this->link->GetModel();
}

////////////////////////////////////////////////////////////////////////////////
// Get the shape type
Base::EntityType Collision::GetShapeType()
{
  return this->shape->GetLeafType();
}

////////////////////////////////////////////////////////////////////////////////
/// Set the shape for this collision
void Collision::SetShape(ShapePtr shape)
{
  this->shape = shape;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the attached shape
ShapePtr Collision::GetShape() const
{
  return this->shape;
}

////////////////////////////////////////////////////////////////////////////////
// Turn contact recording on or off
void Collision::SetContactsEnabled(const bool &enable)
{
  this->contactsEnabled = enable;
  //gzerr << "setting " << this->GetName() << " contacts enabled " << this->contactsEnabled << "\n";
}

////////////////////////////////////////////////////////////////////////////////
// Return true of contact recording is on
bool Collision::GetContactsEnabled() const
{
  //gzerr << "checking " << this->GetName() << " contacts enabled " << this->contactsEnabled << "\n";
  return this->contactsEnabled;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the number of contacts
unsigned int Collision::GetContactCount() const
{
  // TODO: redo this
  return 0;
}

////////////////////////////////////////////////////////////////////////////////
/// Add an occurance of a contact to this collision
void Collision::AddContact(const Contact &_contact)
{
  if (!this->GetContactsEnabled() || this->GetShapeType() == RAY_SHAPE || this->GetShapeType() == PLANE_SHAPE)
    return;

  // previously, Collision keeps a list of contacts, Nate was trying to do this on a per moel basis
  // for now, I'll recover the Collision storage behavior

  //gzerr << "Add Contact to parent Link of thie Collision \n";
  // TODO: redo this
  this->GetParentModel()->StoreContact(
      boost::shared_dynamic_cast<Collision>(shared_from_this()), 
      _contact.Clone());
  this->contact( _contact );

}           

////////////////////////////////////////////////////////////////////////////////
/// Get a specific contact
Contact Collision::GetContact(unsigned int _i)
{
  //return Contact();
  // TODO: redo this
  return this->GetParentModel()->GetContact(
      boost::shared_dynamic_cast<Collision>(shared_from_this()), _i);
}

////////////////////////////////////////////////////////////////////////////////
/// Enable callback: Called when the link changes
void Collision::EnabledCB(bool enabled)
{
  msgs::Visual msg;
  msg.set_name( this->bbVisual);

  if (enabled)
    msg.mutable_material()->set_script( "Gazebo/GreenTransparent" );
  else
    msg.mutable_material()->set_script( "Gazebo/RedTransparent" );

  this->visPub->Publish(msg);
}

////////////////////////////////////////////////////////////////////////////////
/// Get the linear velocity of the collision
math::Vector3 Collision::GetRelativeLinearVel() const
{
  if (this->link)
    return this->link->GetRelativeLinearVel();
  else
    return math::Vector3();
}

////////////////////////////////////////////////////////////////////////////////
/// Get the linear velocity of the collision in the world frame
math::Vector3 Collision::GetWorldLinearVel() const
{
  if (this->link)
    return this->link->GetWorldLinearVel();
  else
    return math::Vector3();
}

////////////////////////////////////////////////////////////////////////////////
/// Get the angular velocity of the collision
math::Vector3 Collision::GetRelativeAngularVel() const
{
  if (this->link)
    return this->link->GetRelativeAngularVel();
  else
    return math::Vector3();
}

////////////////////////////////////////////////////////////////////////////////
/// Get the angular velocity of the collision in the world frame
math::Vector3 Collision::GetWorldAngularVel() const
{
  if (this->link)
    return this->link->GetWorldAngularVel();
  else
    return math::Vector3();
}

////////////////////////////////////////////////////////////////////////////////
/// Get the linear acceleration of the collision
math::Vector3 Collision::GetRelativeLinearAccel() const
{
  if (this->link)
    return this->link->GetRelativeLinearAccel();
  else
    return math::Vector3();
}

////////////////////////////////////////////////////////////////////////////////
/// Get the linear acceleration of the collision in the world frame
math::Vector3 Collision::GetWorldLinearAccel() const
{
  if (this->link)
    return this->link->GetWorldLinearAccel();
  else
    return math::Vector3();
}

////////////////////////////////////////////////////////////////////////////////
/// Get the angular acceleration of the collision
math::Vector3 Collision::GetRelativeAngularAccel() const
{
  if (this->link)
    return this->link->GetRelativeAngularAccel();
  else
    return math::Vector3();
}

////////////////////////////////////////////////////////////////////////////////
/// Get the angular acceleration of the collision in the world frame
math::Vector3 Collision::GetWorldAngularAccel() const
{
  if (this->link)
    return this->link->GetWorldAngularAccel();
  else
    return math::Vector3();
}

////////////////////////////////////////////////////////////////////////////////
/// Update the parameters using new sdf values
void Collision::UpdateParameters( sdf::ElementPtr &_sdf )
{
  Entity::UpdateParameters(_sdf);
}

////////////////////////////////////////////////////////////////////////////////
/// Fill a collision message
void Collision::FillCollisionMsg( msgs::Collision &_msg )
{
  msgs::Set( _msg.mutable_pose(), this->GetWorldPose() );
  _msg.set_name( this->GetCompleteScopedName() );
  _msg.set_laser_retro( this->GetLaserRetro() );
}
 
