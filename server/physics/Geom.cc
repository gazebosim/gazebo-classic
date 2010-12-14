/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/* Desc: Geom class
 * Author: Nate Koenig
 * Date: 13 Feb 2006
 * SVN: $Id$
 */

#include <sstream>

#include "RenderState.hh"
#include "Events.hh"
#include "Model.hh"
#include "Shape.hh"
#include "Mass.hh"
#include "PhysicsEngine.hh"
#include "Global.hh"
#include "GazeboMessage.hh"
#include "SurfaceParams.hh"
#include "World.hh"
#include "Body.hh"
#include "Geom.hh"
#include "Simulator.hh"

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
// Constructor
Geom::Geom( Body *body )
    : Entity(body->GetCoMEntity())
{
  this->bbVisualMsg = NULL;
  this->AddType(GEOM);

  this->body = body;

  // Create the contact parameters
  this->surface = new SurfaceParams();

  this->transparency = 0;

  this->shape = NULL;

  this->contactsEnabled = false;

  Param::Begin(&this->parameters);
  this->typeP = new ParamT<std::string>("type","unknown",1);

  this->massP = new ParamT<double>("mass",0.001,0);
  this->massP->Callback( &Geom::SetMass, this);

  this->xyzP = new ParamT<Vector3>("xyz", Vector3(), 0);
  this->xyzP->Callback( &Entity::SetRelativePosition, (Entity*)this);

  this->rpyP = new ParamT<Quatern>("rpy", Quatern(), 0);
  this->rpyP->Callback( &Entity::SetRelativeRotation, (Entity*)this);

  this->laserFiducialIdP = new ParamT<int>("laserFiducialId",-1,0);
  this->laserRetroP = new ParamT<float>("laserRetro",-1,0);

  this->enableContactsP = new ParamT<bool>("enableContacts", false, 0);
  //this->enableContactsP->Callback( &Geom::SetContactsEnabled, this );
  
  Param::End();

  Events::ConnectShowJointsSignal( boost::bind(&Geom::ToggleTransparent, this) );
  Events::ConnectShowPhysicsSignal( boost::bind(&Geom::ToggleTransparent, this) );
  Events::ConnectShowBoundingBoxesSignal( boost::bind(&Geom::ToggleShowBoundingBox, this) );

  this->body->ConnectEnabledSignal( boost::bind(&Geom::EnabledCB, this, _1) );
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
Geom::~Geom()
{
  for (unsigned int i=0; i < this->visualMsgs.size(); i++)
  {
    this->visualMsgs[i]->action = VisualMsg::DELETE;
    Simulator::Instance()->SendMessage( *this->visualMsgs[i] );
    delete this->visualMsgs[i];
  }
  this->visualMsgs.clear();

  if (this->bbVisualMsg)
  {
    this->bbVisualMsg->action = VisualMsg::DELETE;
    Simulator::Instance()->SendMessage( *this->bbVisualMsg );
    delete this->bbVisualMsg;
  }

  delete this->typeP;
  delete this->massP;
  delete this->xyzP;
  delete this->rpyP;
  delete this->laserFiducialIdP;
  delete this->laserRetroP;
  delete this->enableContactsP;

  if (this->shape)
    delete this->shape;
  this->shape = NULL;
}

////////////////////////////////////////////////////////////////////////////////
/// Finalize the geom
void Geom::Fini()
{
  this->body->DisconnectEnabledSignal(boost::bind(&Geom::EnabledCB, this, _1));

  Events::DisconnectShowPhysicsSignal( boost::bind(&Geom::ToggleTransparent, this) );

  Events::DisconnectShowJointsSignal( boost::bind(&Geom::ToggleTransparent, this) );

  Events::DisconnectShowBoundingBoxesSignal( 
      boost::bind(&Geom::ToggleShowBoundingBox, this) );
}

////////////////////////////////////////////////////////////////////////////////
// First step in the loading process
void Geom::Load(XMLConfigNode *node)
{
  XMLConfigNode *childNode = NULL;

  this->typeP->Load(node);
  this->nameP->Load(node);
  this->SetName(this->nameP->GetValue());
  this->massP->Load(node);
  this->xyzP->Load(node);
  this->rpyP->Load(node);
  this->laserFiducialIdP->Load(node);
  this->laserRetroP->Load(node);
  this->enableContactsP->Load(node);

  this->SetContactsEnabled(**this->enableContactsP);

  this->SetRelativePose( Pose3d( **this->xyzP, **this->rpyP ) );

  this->mass.SetMass( **this->massP );

  this->surface->Load(node);

  this->shape->Load(node);

  this->CreateBoundingBox();

  this->body->AttachGeom(this);

  childNode = node->GetChild("visual");
  while (childNode)
  {
    std::ostringstream visname;
    visname << this->GetCompleteScopedName() << "_VISUAL_" << 
               this->visualMsgs.size();


    VisualMsg *msg = new VisualMsg();

    msg->parentId = this->GetName();
    msg->id = visname.str();
    msg->render = VisualMsg::MESH_RESOURCE;
    msg->Load(childNode);
    msg->castShadows = false;
    msg->pose = this->GetWorldPose();
    std::cout << "Geom[" << this->GetName() << "] Create a Visual Message with Pose[" << msg->pose << "]\n";

    Simulator::Instance()->SendMessage( *msg );

    this->visualMsgs.push_back(msg);
   
    childNode = childNode->GetNext("visual");
  }
}

////////////////////////////////////////////////////////////////////////////////
// Create the bounding box for the geom
void Geom::CreateBoundingBox()
{
  // Create the bounding box
  if (this->GetShapeType() != PLANE_SHAPE && this->GetShapeType() != MAP_SHAPE)
  {
    Vector3 min;
    Vector3 max;

    this->GetBoundingBox(min,max);

    std::ostringstream visname;
    visname << this->GetCompleteScopedName() << "_BBVISUAL" ;

    this->bbVisualMsg = new VisualMsg();

    this->bbVisualMsg->render = VisualMsg::MESH_RESOURCE;
    this->bbVisualMsg->parentId = this->GetName();
    this->bbVisualMsg->id = this->GetName() + "_BBVISUAL";
    this->bbVisualMsg->castShadows = false;
    this->bbVisualMsg->visible = RenderState::GetShowBoundingBoxes();
    this->bbVisualMsg->boundingbox.min = min;
    this->bbVisualMsg->boundingbox.max = max;

    Simulator::Instance()->SendMessage( *this->bbVisualMsg );
  }
}

////////////////////////////////////////////////////////////////////////////////
// Save the body based on our XMLConfig node
void Geom::Save(std::string &prefix, std::ostream &stream)
{
  if (!this->GetSaveable())
    return;

  std::string p = prefix + "  ";

  this->xyzP->SetValue( this->GetRelativePose().pos );
  this->rpyP->SetValue( this->GetRelativePose().rot );

  stream << prefix << "<geom:" << **this->typeP << " name=\"" 
         << this->nameP->GetValue() << "\">\n";

  stream << prefix << "  " << *(this->xyzP) << "\n";
  stream << prefix << "  " << *(this->rpyP) << "\n";

  this->shape->Save(p,stream);

  stream << prefix << "  " << *(this->massP) << "\n";

  stream << prefix << "  " << *(this->laserFiducialIdP) << "\n";
  stream << prefix << "  " << *(this->laserRetroP) << "\n";

  // NATY: put back in functionality
  /*std::vector<Visual*>::iterator iter;
  for (iter = this->visuals.begin(); iter != this->visuals.end(); iter++)
  {
    if (*iter)
      (*iter)->Save(p, stream);
  }*/

  stream << prefix << "</geom>\n";
}

////////////////////////////////////////////////////////////////////////////////
// Set the encapsulated geometry object
void Geom::SetGeom(bool placeable)
{
  this->GetWorld()->GetPhysicsEngine()->LockMutex();

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

  this->GetWorld()->GetPhysicsEngine()->UnlockMutex();
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
// Toggle bounding box visibility
void Geom::ToggleShowBoundingBox()
{
  if (this->bbVisualMsg)
  {
    this->bbVisualMsg->visible = !this->bbVisualMsg->visible;
    this->bbVisualMsg->action = VisualMsg::UPDATE;
    Simulator::Instance()->SendMessage( *this->bbVisualMsg );
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Set the visibility of the Bounding box of this geometry
void Geom::ShowBoundingBox(bool show)
{
  if (this->bbVisualMsg)
  {
    this->bbVisualMsg->visible = show;
    this->bbVisualMsg->action = VisualMsg::UPDATE;
    Simulator::Instance()->SendMessage( *this->bbVisualMsg );
  }
}

////////////////////////////////////////////////////////////////////////////////
// Toggle transparency
void Geom::ToggleTransparent()
{
  for (unsigned int i = 0; i < this->visualMsgs.size(); i++)
  {
    this->visualMsgs[i]->action = VisualMsg::UPDATE;
    if (this->visualMsgs[i]->transparency == 0.0)
      this->visualMsgs[i]->transparency = 0.6;
    else
      this->visualMsgs[i]->transparency = 0.0;
    Simulator::Instance()->SendMessage( *this->visualMsgs[i] );
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Set the transparency
void Geom::SetTransparent(bool show)
{
for (unsigned int i = 0; i < this->visualMsgs.size(); i++)
  {
    this->visualMsgs[i]->action = VisualMsg::UPDATE;
    if (show)
      this->visualMsgs[i]->transparency = 0.6;
    else
      this->visualMsgs[i]->transparency = 0.0;
    Simulator::Instance()->SendMessage( *this->visualMsgs[i] );
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
Body *Geom::GetBody() const
{
  return this->body;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the model this geom belongs to
Model *Geom::GetModel() const
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
EntityType Geom::GetShapeType()
{
  return this->shape->GetLeafType();
}

////////////////////////////////////////////////////////////////////////////////
/// Set the shape for this geom
void Geom::SetShape(Shape *shape)
{
  this->shape = shape;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the attached shape
Shape *Geom::GetShape() const
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
  return this->GetParentModel()->GetContactCount(this);
}

////////////////////////////////////////////////////////////////////////////////
/// Add an occurance of a contact to this geom
void Geom::AddContact(const Contact &contact)
{
  if (!this->GetContactsEnabled() || this->GetShapeType() == RAY_SHAPE || this->GetShapeType() == PLANE_SHAPE)
    return;

  this->GetParentModel()->StoreContact(this, contact);
  this->contactSignal( contact );
}           

////////////////////////////////////////////////////////////////////////////////
/// Get a specific contact
Contact Geom::GetContact(unsigned int i) const
{
  return this->GetParentModel()->RetrieveContact(this, i);
}

////////////////////////////////////////////////////////////////////////////////
/// Enable callback: Called when the body changes
void Geom::EnabledCB(bool enabled)
{
  if (enabled)
    this->bbVisualMsg->material = "Gazebo/GreenTransparent";
  else
    this->bbVisualMsg->material = "Gazebo/RedTransparent";

  Simulator::Instance()->SendMessage( *this->bbVisualMsg );
}

////////////////////////////////////////////////////////////////////////////////
/// Get the linear velocity of the geom
Vector3 Geom::GetRelativeLinearVel() const
{
  if (this->body)
    return this->body->GetRelativeLinearVel();
  else
    return Vector3();
}

////////////////////////////////////////////////////////////////////////////////
/// Get the linear velocity of the geom in the world frame
Vector3 Geom::GetWorldLinearVel() const
{
  if (this->body)
    return this->body->GetWorldLinearVel();
  else
    return Vector3();
}

////////////////////////////////////////////////////////////////////////////////
/// Get the angular velocity of the geom
Vector3 Geom::GetRelativeAngularVel() const
{
  if (this->body)
    return this->body->GetRelativeAngularVel();
  else
    return Vector3();
}

////////////////////////////////////////////////////////////////////////////////
/// Get the angular velocity of the geom in the world frame
Vector3 Geom::GetWorldAngularVel() const
{
  if (this->body)
    return this->body->GetWorldAngularVel();
  else
    return Vector3();
}

////////////////////////////////////////////////////////////////////////////////
/// Get the linear acceleration of the geom
Vector3 Geom::GetRelativeLinearAccel() const
{
  if (this->body)
    return this->body->GetRelativeLinearAccel();
  else
    return Vector3();
}

////////////////////////////////////////////////////////////////////////////////
/// Get the linear acceleration of the geom in the world frame
Vector3 Geom::GetWorldLinearAccel() const
{
  if (this->body)
    return this->body->GetWorldLinearAccel();
  else
    return Vector3();
}

////////////////////////////////////////////////////////////////////////////////
/// Get the angular acceleration of the geom
Vector3 Geom::GetRelativeAngularAccel() const
{
  if (this->body)
    return this->body->GetRelativeAngularAccel();
  else
    return Vector3();
}

////////////////////////////////////////////////////////////////////////////////
/// Get the angular acceleration of the geom in the world frame
Vector3 Geom::GetWorldAngularAccel() const
{
  if (this->body)
    return this->body->GetWorldAngularAccel();
  else
    return Vector3();
}
