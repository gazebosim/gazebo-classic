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
/* Desc: Geom class
 * Author: Nate Koenig
 * Date: 13 Feb 2006
 */

#include <sstream>

#include "common/Events.hh"
#include "common/Messages.hh"
#include "common/Console.hh"
#include "common/XMLConfig.hh"

#include "transport/Publisher.hh"

#include "physics/Contact.hh"
#include "physics/Shape.hh"
#include "physics/SurfaceParams.hh"
#include "physics/Body.hh"
#include "physics/Geom.hh"

using namespace gazebo;
using namespace physics;

////////////////////////////////////////////////////////////////////////////////
// Constructor
Geom::Geom( BodyPtr body )
    : Entity(body)
{
  this->AddType(Base::GEOM);

  this->body = body;
  this->surface.reset( new SurfaceParams() );

  this->transparency = 0;
  this->contactsEnabled = false;

  common::Param::Begin(&this->parameters);

  this->massP = new common::ParamT<double>("mass",0.001,0);
  this->massP->Callback( &Geom::SetMass, this);

  this->xyzP = new common::ParamT<math::Vector3>("xyz", math::Vector3(), 0);
  this->xyzP->Callback( &Entity::SetRelativePosition, (Entity*)this);

  this->rpyP = new common::ParamT<math::Quaternion>("rpy", math::Quaternion(), 0);
  this->rpyP->Callback( &Entity::SetRelativeRotation, (Entity*)this);

  this->laserFiducialIdP = new common::ParamT<int>("laser_fiducial_id",-1,0);
  this->laserRetroP = new common::ParamT<float>("laser_retro",-1,0);

  this->enableContactsP = new common::ParamT<bool>("enable_contacts", false, 0);
  //this->enableContactsP->Callback( &Geom::SetContactsEnabled, this );
  
  common::Param::End();

  this->connections.push_back( event::Events::ConnectShowBoundingBoxesSignal( boost::bind(&Geom::ShowBoundingBox, this, _1) ) );
  this->connections.push_back( this->body->ConnectEnabledSignal( boost::bind(&Geom::EnabledCB, this, _1) ) );
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
Geom::~Geom()
{
  if (!this->bbVisual.empty())
  {
    msgs::Visual msg;
    common::Message::Init(msg, this->bbVisual);
    msg.set_action( msgs::Visual::DELETE);
    this->visPub->Publish(msg);
  }

  delete this->massP;
  delete this->xyzP;
  delete this->rpyP;
  delete this->laserFiducialIdP;
  delete this->laserRetroP;
  delete this->enableContactsP;
}

////////////////////////////////////////////////////////////////////////////////
/// Finalize the geom
void Geom::Fini()
{
  this->connections.clear();
}

////////////////////////////////////////////////////////////////////////////////
// First step in the loading process
void Geom::Load(common::XMLConfigNode *node)
{
  common::XMLConfigNode *childNode;

  Entity::Load(node);

  this->massP->Load(node);
  this->laserFiducialIdP->Load(node);
  this->laserRetroP->Load(node);
  this->enableContactsP->Load(node);
  this->surface->Load(node);

  if ( (childNode = node->GetChild("origin")) != NULL)
  {
    this->xyzP->Load(childNode);
    this->rpyP->Load(childNode);
  }

  if (this->shape)
    this->shape->Load(node->GetChild("geometry"));
  else
    gzwarn << "No shape has been specified. Error!!!\n";

  //TODO: this shouldn't exist in the physics sim
  //this->CreateBoundingBox();
}

void Geom::Init()
{
  this->SetContactsEnabled(**this->enableContactsP);
  this->SetRelativePose( math::Pose( **this->xyzP, **this->rpyP ) );
  this->mass.SetMass( **this->massP );

  this->shape->Init();
}

////////////////////////////////////////////////////////////////////////////////
// Create the bounding box for the geom
void Geom::CreateBoundingBox()
{
  // Create the bounding box
  if (this->GetShapeType() != PLANE_SHAPE && this->GetShapeType() != MAP_SHAPE)
  {
    math::Box box;

    box = this->GetBoundingBox();

    std::ostringstream visname;
    visname << this->GetCompleteScopedName() << "::BBVISUAL" ;

    msgs::Visual msg;
    msg.set_render_type( msgs::Visual::MESH_RESOURCE );
    msg.set_parent_id( this->GetCompleteScopedName() );
    msg.mutable_header()->set_str_id( this->GetCompleteScopedName() + "_BBVISUAL" );
    msg.set_cast_shadows(false);
    // NATY: Set it so bounding boxes are visible upon creation if a flag is
    // set. The flag should exist only on the rendering side. May need to
    // put something in the message to indicate that this is a bounding box
    //msg.set_visible( RenderState::GetShowBoundingBoxes() );
    msg.set_mesh( "unit_box" );
    if (this->IsStatic() )
      msg.set_material( "Gazebo/YellowTransparent" );
    else
      msg.set_material( "Gazebo/GreenTransparent" );

    common::Message::Set(msg.mutable_scale(), (box.max - box.min) * 1.05);
    common::Message::Set(msg.mutable_pose()->mutable_position(), math::Vector3(0,0,0.0));
    common::Message::Set(msg.mutable_pose()->mutable_orientation(), math::Quaternion(1,0,0,0));
    msg.set_transparency( .5 );

    this->visPub->Publish(msg);
  }
}

////////////////////////////////////////////////////////////////////////////////
// Save the body based on our common::XMLConfig node
void Geom::Save(std::string &prefix, std::ostream &stream)
{
  if (!this->GetSaveable())
    return;

  std::string p = prefix + "  ";

  this->xyzP->SetValue( this->GetRelativePose().pos );
  this->rpyP->SetValue( this->GetRelativePose().rot );

  stream << prefix << "<geom name=\"" 
         << this->GetName() << "\">\n";

  stream << prefix << "  " << *(this->xyzP) << "\n";
  stream << prefix << "  " << *(this->rpyP) << "\n";

  this->shape->Save(p,stream);

  stream << prefix << "  " << *(this->massP) << "\n";

  stream << prefix << "  " << *(this->laserFiducialIdP) << "\n";
  stream << prefix << "  " << *(this->laserRetroP) << "\n";

  stream << prefix << "</geom>\n";
}

////////////////////////////////////////////////////////////////////////////////
// Set the encapsulated geometry object
void Geom::SetGeom(bool placeable)
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
// Return whether this is a placeable geom.
bool Geom::IsPlaceable() const
{
  return this->placeable;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the laser fiducial integer id
void Geom::SetLaserFiducialId(int id)
{
  this->laserFiducialIdP->SetValue( id );
}

////////////////////////////////////////////////////////////////////////////////
/// Get the laser fiducial integer id
int Geom::GetLaserFiducialId() const
{
  return this->laserFiducialIdP->GetValue();
}

////////////////////////////////////////////////////////////////////////////////
/// Set the laser retro reflectiveness
void Geom::SetLaserRetro(float retro)
{
  this->laserRetroP->SetValue(retro);
}

////////////////////////////////////////////////////////////////////////////////
/// Get the laser retro reflectiveness
float Geom::GetLaserRetro() const
{
  return this->laserRetroP->GetValue();
}


////////////////////////////////////////////////////////////////////////////////
/// Set the visibility of the Bounding box of this geometry
void Geom::ShowBoundingBox(const bool &show)
{
  if (!this->bbVisual.empty())
  {
    msgs::Visual msg;
    common::Message::Init(msg, this->bbVisual);
    msg.set_visible( show );
    msg.set_action( msgs::Visual::UPDATE );
    this->visPub->Publish(msg);
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Set the mass
void Geom::SetMass(const Mass &_mass)
{
  this->mass = _mass;
  //this->body->UpdateCoM();
}

////////////////////////////////////////////////////////////////////////////////
/// Set the mass
void Geom::SetMass(const double &_mass)
{
  this->mass.SetMass( _mass );
  //this->body->UpdateCoM();
}

////////////////////////////////////////////////////////////////////////////////
/// Get the body this geom belongs to
BodyPtr Geom::GetBody() const
{
  return this->body;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the model this geom belongs to
ModelPtr Geom::GetModel() const
{
  return this->body->GetModel();
}

////////////////////////////////////////////////////////////////////////////////
/// Set the friction mode of the geom
void Geom::SetFrictionMode( const bool &v )
{
  this->surface->enableFriction = v;
}


////////////////////////////////////////////////////////////////////////////////
/// Get a pointer to the mass
const Mass &Geom::GetMass() const
{
  return this->mass;
}

////////////////////////////////////////////////////////////////////////////////
// Get the shape type
Base::EntityType Geom::GetShapeType()
{
  return this->shape->GetLeafType();
}

////////////////////////////////////////////////////////////////////////////////
/// Set the shape for this geom
void Geom::SetShape(ShapePtr shape)
{
  this->shape = shape;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the attached shape
ShapePtr Geom::GetShape() const
{
  return this->shape;
}

////////////////////////////////////////////////////////////////////////////////
// Turn contact recording on or off
void Geom::SetContactsEnabled(const bool &enable)
{
  this->contactsEnabled = enable;
}

////////////////////////////////////////////////////////////////////////////////
// Return true of contact recording is on
bool Geom::GetContactsEnabled() const
{
  return this->contactsEnabled;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the number of contacts
unsigned int Geom::GetContactCount() const
{
  // TODO: redo this
  return 0;
}

////////////////////////////////////////////////////////////////////////////////
/// Add an occurance of a contact to this geom
void Geom::AddContact(const Contact &/*_contact*/)
{
  if (!this->GetContactsEnabled() || this->GetShapeType() == RAY_SHAPE || this->GetShapeType() == PLANE_SHAPE)
    return;

  // TODO: redo this
  //this->GetParentModel()->StoreContact(shared_from_this(), contact);
  //this->contactSignal( contact );
}           

////////////////////////////////////////////////////////////////////////////////
/// Get a specific contact
Contact Geom::GetContact(unsigned int /*_i*/) const
{
  return Contact();
  // TODO: redo this
  //return this->GetParentModel()->RetrieveContact(, _i);
}

////////////////////////////////////////////////////////////////////////////////
/// Enable callback: Called when the body changes
void Geom::EnabledCB(bool enabled)
{
  msgs::Visual msg;
  common::Message::Init(msg, this->bbVisual);

  if (enabled)
    msg.set_material( "Gazebo/GreenTransparent" );
  else
    msg.set_material( "Gazebo/RedTransparent" );

  this->visPub->Publish(msg);
}

////////////////////////////////////////////////////////////////////////////////
/// Get the linear velocity of the geom
math::Vector3 Geom::GetRelativeLinearVel() const
{
  if (this->body)
    return this->body->GetRelativeLinearVel();
  else
    return math::Vector3();
}

////////////////////////////////////////////////////////////////////////////////
/// Get the linear velocity of the geom in the world frame
math::Vector3 Geom::GetWorldLinearVel() const
{
  if (this->body)
    return this->body->GetWorldLinearVel();
  else
    return math::Vector3();
}

////////////////////////////////////////////////////////////////////////////////
/// Get the angular velocity of the geom
math::Vector3 Geom::GetRelativeAngularVel() const
{
  if (this->body)
    return this->body->GetRelativeAngularVel();
  else
    return math::Vector3();
}

////////////////////////////////////////////////////////////////////////////////
/// Get the angular velocity of the geom in the world frame
math::Vector3 Geom::GetWorldAngularVel() const
{
  if (this->body)
    return this->body->GetWorldAngularVel();
  else
    return math::Vector3();
}

////////////////////////////////////////////////////////////////////////////////
/// Get the linear acceleration of the geom
math::Vector3 Geom::GetRelativeLinearAccel() const
{
  if (this->body)
    return this->body->GetRelativeLinearAccel();
  else
    return math::Vector3();
}

////////////////////////////////////////////////////////////////////////////////
/// Get the linear acceleration of the geom in the world frame
math::Vector3 Geom::GetWorldLinearAccel() const
{
  if (this->body)
    return this->body->GetWorldLinearAccel();
  else
    return math::Vector3();
}

////////////////////////////////////////////////////////////////////////////////
/// Get the angular acceleration of the geom
math::Vector3 Geom::GetRelativeAngularAccel() const
{
  if (this->body)
    return this->body->GetRelativeAngularAccel();
  else
    return math::Vector3();
}

////////////////////////////////////////////////////////////////////////////////
/// Get the angular acceleration of the geom in the world frame
math::Vector3 Geom::GetWorldAngularAccel() const
{
  if (this->body)
    return this->body->GetWorldAngularAccel();
  else
    return math::Vector3();
}
