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
/* Desc: Body class
 * Author: Nate Koenig
 * Date: 13 Feb 2006
 */

#include <sstream>
#include <float.h>

#include "common/Messages.hh"
#include "common/Events.hh"
#include "common/Quatern.hh"
#include "common/XMLConfig.hh"
#include "common/Console.hh"
#include "common/Global.hh"
#include "common/Exception.hh"

#include "physics/Model.hh"
#include "physics/World.hh"
#include "physics/PhysicsEngine.hh"
#include "physics/Geom.hh"
#include "physics/Body.hh"

using namespace gazebo;
using namespace physics;

////////////////////////////////////////////////////////////////////////////////
// Constructor
Body::Body(EntityPtr parent)
    : Entity(parent)
{
  this->AddType(Base::BODY);

  common::Param::Begin(&this->parameters);
  this->autoDisableP = new common::ParamT<bool>("auto_disable", true, 0);
  this->autoDisableP->Callback( &Body::SetAutoDisable, this );

  this->xyzP = new common::ParamT<common::Vector3>("xyz", common::Vector3(), 0);
  this->xyzP->Callback( &Entity::SetRelativePosition, (Entity*)this );

  this->rpyP = new common::ParamT<common::Quatern>("rpy", common::Quatern(), 0);
  this->rpyP->Callback( &Entity::SetRelativeRotation, (Entity*)this );
  this->dampingFactorP = new common::ParamT<double>("damping_factor", 0.0, 0);

  // option to turn gravity off for individual body
  this->turnGravityOffP = new common::ParamT<bool>("turn_gravity_off", false, 0);

  // option to make body collide with bodies of the same parent
  this->selfCollideP = new common::ParamT<bool>("self_collide", false, 0);

  // User can specify mass/inertia property for the body
  this->customMassMatrixP = new common::ParamT<bool>("mass_matrix",false,0);
  this->cxP = new common::ParamT<double>("cx",0.0,0);
  this->cyP = new common::ParamT<double>("cy",0.0,0);
  this->czP = new common::ParamT<double>("cz",0.0,0);
  this->bodyMassP = new common::ParamT<double>("mass",0.001,0);
  this->ixxP = new common::ParamT<double>("ixx",1e-6,0);
  this->iyyP = new common::ParamT<double>("iyy",1e-6,0);
  this->izzP = new common::ParamT<double>("izz",1e-6,0);
  this->ixyP = new common::ParamT<double>("ixy",0.0,0);
  this->ixzP = new common::ParamT<double>("ixz",0.0,0);
  this->iyzP = new common::ParamT<double>("iyz",0.0,0);

  this->kinematicP = new common::ParamT<bool>("kinematic",true,0);
  this->kinematicP->SetHelp("true = kinematic state only, false = dynamic body");
  this->kinematicP->Callback( &Body::SetKinematic, this );

  common::Param::End();
}


////////////////////////////////////////////////////////////////////////////////
// Destructor
Body::~Body()
{
  std::vector<Entity*>::iterator iter;

  for (unsigned int i=0; i < this->visuals.size(); i++)
  {
    msgs::Visual msg;
    common::Message::Init(msg, this->visuals[i]);
    msg.set_action( msgs::Visual::DELETE );
    this->vis_pub->Publish(msg);
  }
  this->visuals.clear();

  if (this->cgVisuals.size() > 0)
  {
    for (unsigned int i=0; i < this->cgVisuals.size(); i++)
    {
      msgs::Visual msg;
      common::Message::Init(msg, this->cgVisuals[i]);
      msg.set_action( msgs::Visual::DELETE );
      this->vis_pub->Publish(msg);
    }
    this->cgVisuals.clear();
  }

  delete this->xyzP;
  delete this->rpyP;
  delete this->dampingFactorP;
  delete this->turnGravityOffP;
  delete this->selfCollideP;

  delete this->customMassMatrixP;
  delete this->cxP ;
  delete this->cyP ;
  delete this->czP ;
  delete this->bodyMassP;
  delete this->ixxP;
  delete this->iyyP;
  delete this->izzP;
  delete this->ixyP;
  delete this->ixzP;
  delete this->iyzP;
  delete this->kinematicP;
}

////////////////////////////////////////////////////////////////////////////////
// Load the body based on an common::XMLConfig node
void Body::Load(common::XMLConfigNode *node)
{
  common::XMLConfigNode *childNode;

  Entity::Load(node);

  //this->comEntity.reset( new Entity(shared_from_this()) );
  //this->comEntity->SetName("COM_Entity");
  //this->comEntity->Load(NULL);

  // before loading child geometry, we have to figure out of selfCollide is true
  // and modify parent class Entity so this body has its own spaceId
  this->selfCollideP->Load(node);

  // option to enter full mass matrix
  // load custom inertia matrix for the body
  this->customMassMatrixP->Load(node);
  this->cxP ->Load(node);
  this->cyP ->Load(node);
  this->czP ->Load(node);
  this->bodyMassP->Load(node);
  this->ixxP->Load(node);
  this->iyyP->Load(node);
  this->izzP->Load(node);
  this->ixyP->Load(node);
  this->ixzP->Load(node);
  this->iyzP->Load(node);
  this->kinematicP->Load(node);
  this->dampingFactorP->Load(node);
  this->turnGravityOffP->Load(node);

  if ( (childNode = node->GetChild("origin")) != NULL)
  {
    this->xyzP->Load(childNode);
    this->rpyP->Load(childNode);
  }

  // before loading child geometry, we have to figure out of selfCollide is true
  // and modify parent class Entity so this body has its own spaceId
  this->SetSelfCollide( **this->selfCollideP );

  // TODO: this shouldn't be in the physics sim
  childNode = node->GetChild("visual");
  while (childNode)
  {
    std::ostringstream visname;
    visname << this->GetCompleteScopedName() << "::VISUAL_" << 
               this->visuals.size();

    msgs::Visual msg = common::Message::VisualFromXML(childNode);
    common::Message::Init(msg, visname.str());
    msg.set_parent_id( this->GetCompleteScopedName() );
    msg.set_is_static( this->IsStatic() );

    this->vis_pub->Publish(msg);
    this->visuals.push_back(msg.header().str_id());
   
    childNode = childNode->GetNext("visual");
  }
 
  // Load the geometries
  childNode = node->GetChild("collision");
  while (childNode)
  {
    // Create and Load a geom, which will belong to this body.
    this->LoadGeom(childNode);
    childNode = childNode->GetNext("collision");
  }

}

////////////////////////////////////////////////////////////////////////////////
// Initialize the body
void Body::Init()
{
  this->customMass.SetCoG(**this->cxP, **this->cyP,** this->czP);
  this->customMass.SetInertiaMatrix( **this->ixxP, **this->iyyP, **this->izzP,
                                     **this->ixyP, **this->ixzP, **this->iyzP);
  this->customMass.SetMass(**this->bodyMassP);

  this->mass = this->customMass;
 
  Base_V::iterator iter;
  for (iter = this->children.begin(); iter != this->children.end(); iter++)
  {
    if ((*iter)->HasType(Base::GEOM))
    {
      GeomPtr g = boost::shared_static_cast<Geom>(*iter);
      g->Init();
      if (!**this->customMassMatrixP)
        this->mass += g->GetMass();
    }
  }

  // save transform from this Parent Model Frame to this Body Frame
  // this is only used in setting Model pose from canonicalBody
  // the true model pose given a canonical body is
  //   this body's pose - this body's offsetFromModelFrame
  this->initModelOffset = this->GetRelativePose().CoordPoseSolve(common::Pose3d());

  this->SetKinematic(**this->kinematicP);

  // If no geoms are attached, then don't let gravity affect the body.
  if (this->children.size()==0 || **this->turnGravityOffP)
    this->SetGravityMode(false);

  // global-inertial damping is implemented in ode svn trunk
  if(this->GetId() && this->dampingFactorP->GetValue() > 0)
  {
    this->SetLinearDamping(**this->dampingFactorP);
    this->SetAngularDamping(**this->dampingFactorP);
  }

  this->linearAccel.Set(0,0,0);
  this->angularAccel.Set(0,0,0);

  /// Attach mesh for CG visualization
  /// Add a renderable visual for CG, make visible in Update()
  /// TODO: this shouldn't be in the physics sim
  /*if (this->mass.GetAsDouble() > 0.0)
  {
    std::ostringstream visname;
    visname << this->GetCompleteScopedName() + ":" + this->GetName() << "_CGVISUAL" ;

    msgs::Visual msg;
    common::Message::Init(msg, visname.str());
    msg.set_parent_id( this->comEntity->GetCompleteScopedName() );
    msg.set_render_type( msgs::Visual::MESH_RESOURCE );
    msg.set_mesh( "unit_box" );
    msg.set_material( "Gazebo/RedGlow" );
    msg.set_cast_shadows( false );
    msg.set_attach_axes( true );
    msg.set_visible( false );
    common::Message::Set(msg.mutable_scale(), common::Vector3(0.1, 0.1, 0.1));
    this->vis_pub->Publish(msg);
    this->cgVisuals.push_back( msg.header().str_id() );

    if (this->children.size() > 1)
    {
      msgs::Visual g_msg;
      common::Message::Init(g_msg, visname.str() + "_connectors");

      g_msg.set_parent_id( this->comEntity->GetCompleteScopedName() );
      g_msg.set_render_type( msgs::Visual::LINE_LIST );
      g_msg.set_attach_axes( false );
      g_msg.set_material( "Gazebo/GreenGlow" );
      g_msg.set_visible( false );

      // Create a line to each geom
      for (Base_V::iterator giter = this->children.begin(); 
           giter != this->children.end(); giter++)
      {
        EntityPtr e = boost::shared_dynamic_cast<Entity>(*giter);

        msgs::Point *pt;
        pt = g_msg.add_points();
        pt->set_x(0);
        pt->set_y(0);
        pt->set_z(0);

        pt = g_msg.add_points();
        pt->set_x(e->GetRelativePose().pos.x);
        pt->set_y(e->GetRelativePose().pos.y);
        pt->set_z(e->GetRelativePose().pos.z);
      }
      this->vis_pub->Publish(msg);
      this->cgVisuals.push_back( g_msg.header().str_id() );
    }
  }*/

  this->enabled = true;

  // DO THIS LAST!
  this->SetRelativePose(common::Pose3d(**this->xyzP, **this->rpyP));
}

////////////////////////////////////////////////////////////////////////////////
// Save the body based on our common::XMLConfig node
void Body::Save(std::string &prefix, std::ostream &stream)
{
  Base_V::iterator giter;

  this->xyzP->SetValue( this->GetRelativePose().pos );
  this->rpyP->SetValue( this->GetRelativePose().rot );

  stream << prefix << "<body name=\"" << this->GetName() << "\">\n";
  stream << prefix << "  " << *(this->xyzP) << "\n";
  stream << prefix << "  " << *(this->rpyP) << "\n";

  std::string p = prefix + "  ";

  for (giter = this->children.begin(); giter != this->children.end(); giter++)
  {
    stream << "\n";
    (*giter)->Save(p, stream);
  }

  // TODO: put back in functionality
  /*std::vector<Visual*>::iterator iter;
  for (iter = this->visuals.begin(); iter != this->visuals.end(); iter++)
  {
    if (*iter)
      (*iter)->Save(p, stream);
  }*/


  stream << prefix << "</body>\n";
}


////////////////////////////////////////////////////////////////////////////////
// Finalize the body
void Body::Fini()
{
  Base_V::iterator giter;

  this->connections.clear();

  for (giter = this->children.begin(); giter != this->children.end(); giter++)
    (*giter)->Fini();
}

////////////////////////////////////////////////////////////////////////////////
/// Set the friction mode of the body
void Body::SetFrictionMode( const bool &v )
{
  Base_V::iterator iter;
  for (iter = this->children.begin(); iter != this->children.end(); iter++)
  {
    if ((*iter)->HasType(Base::GEOM))
      boost::shared_static_cast<Geom>(*iter)->SetFrictionMode( v );
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Set the collide mode of the body
void Body::SetCollideMode( const std::string &m )
{
  Base_V::iterator giter;

  unsigned int collideBits;

  if (m == "all")
    collideBits =  GZ_ALL_COLLIDE;
  else if (m == "none")
    collideBits =  GZ_NONE_COLLIDE;
  else if (m == "sensors")
    collideBits = GZ_SENSOR_COLLIDE;
  else if (m == "ghost")
    collideBits = GZ_GHOST_COLLIDE;
  else
  {
    gzerr << "Unknown collide mode[" << m << "]\n";
    return;
  }

  // TODO: Put this back in
  /*for (giter = this->geoms.begin(); giter != this->geoms.end(); giter++)
  {
    (*giter)->SetCategoryBits(collideBits);
    (*giter)->SetCollideBits(collideBits);
  }*/
}

////////////////////////////////////////////////////////////////////////////////
// Return Self-Collision Setting
bool Body::GetSelfCollide()
{
  return this->selfCollideP->GetValue();
}

////////////////////////////////////////////////////////////////////////////////
/// Set the laser fiducial integer id of this body
void Body::SetLaserFiducialId(int id)
{
  Base_V::iterator iter;

  for (iter = this->children.begin(); iter != this->children.end(); iter++)
  {
    if ((*iter)->HasType(Base::GEOM))
      boost::shared_static_cast<Geom>(*iter)->SetLaserFiducialId( id );
  }
}


////////////////////////////////////////////////////////////////////////////////
/// Set the laser retro reflectiveness of this body
void Body::SetLaserRetro(float retro)
{
  Base_V::iterator iter;

  for (iter = this->children.begin(); iter != this->children.end(); iter++)
  {
    if ((*iter)->HasType(Base::GEOM))
      boost::shared_static_cast<Geom>(*iter)->SetLaserRetro( retro );
  }
}

////////////////////////////////////////////////////////////////////////////////
// Update the body
void Body::Update()
{
  // Apply our linear accel
  this->SetForce(this->linearAccel);

  // Apply our angular accel
  this->SetTorque(this->angularAccel);

  // FIXME: FIXME: @todo: @todo: race condition on factory-based model loading!!!!!
   if (this->GetEnabled() != this->enabled)
   {
     this->enabled = this->GetEnabled();
     this->enabledSignal(this->enabled);
   }
}

////////////////////////////////////////////////////////////////////////////////
/// Update the center of mass
void Body::UpdateCoM()
{
  common::Pose3d bodyPose;
  common::Pose3d origPose, newPose;
  Base_V::iterator iter;

  bodyPose = this->GetRelativePose();

  if (**this->customMassMatrixP)
  {
    this->mass = this->customMass;
  }

  // Translate all the geoms so that the CoG is at (0,0,0) in the body frame
  for (iter = this->children.begin(); iter != this->children.end(); iter++)
  {
    EntityPtr e = boost::shared_dynamic_cast<Entity>(*iter);
    if (e)
    {
      common::Vector3 offset;
      origPose = e->GetRelativePose();
      newPose = origPose;

      newPose.pos -= this->mass.GetCoG();
      e->SetRelativePose(newPose, true);
    }
  }

  //this->comEntity->SetRelativePose(common::Pose3d(this->mass.GetCoG(),common::Quatern()),true);
  this->OnPoseChange();
}


////////////////////////////////////////////////////////////////////////////////
// Load a new geom helper function
void Body::LoadGeom(common::XMLConfigNode *node)
{
  GeomPtr geom;
  if (!node->GetChild("geometry"))
    gzthrow("Collision needs a geometry");

  std::string type = node->GetChild("geometry")->GetChild()->GetName();

  /*if (type == "heightmap" || type == "map")
    this->SetStatic(true);
    */

  geom = this->GetWorld()->GetPhysicsEngine()->CreateGeom( type, 
      boost::shared_static_cast<Body>(shared_from_this()) );

  if (!geom)
    gzthrow("Unknown Geometry Type["+
        node->GetString("name",std::string(),0)+"]");

  geom->Load(node);
}

////////////////////////////////////////////////////////////////////////////////
/// Set the linear acceleration of the body
void Body::SetLinearAccel(const common::Vector3 &accel)
{
  //this->SetEnabled(true); Disabled this line to make autoDisable work
  this->linearAccel = accel;// * this->GetMass();
}



////////////////////////////////////////////////////////////////////////////////
/// Set the angular acceleration of the body
void Body::SetAngularAccel(const common::Vector3 &accel)
{
  //this->SetEnabled(true); Disabled this line to make autoDisable work
  this->angularAccel = accel * this->mass.GetAsDouble();
}

////////////////////////////////////////////////////////////////////////////////
/// Get the linear velocity of the body
common::Vector3 Body::GetRelativeLinearVel() const
{
  return this->GetWorldPose().rot.RotateVectorReverse(this->GetWorldLinearVel());
}

////////////////////////////////////////////////////////////////////////////////
/// Get the angular velocity of the body
common::Vector3 Body::GetRelativeAngularVel() const
{
  return this->GetWorldPose().rot.RotateVectorReverse(this->GetWorldAngularVel());
}

////////////////////////////////////////////////////////////////////////////////
/// Get the linear acceleration of the body
common::Vector3 Body::GetRelativeLinearAccel() const
{
  return this->GetRelativeForce() / this->mass.GetAsDouble();
}

////////////////////////////////////////////////////////////////////////////////
/// Get the angular acceleration of the body
common::Vector3 Body::GetWorldLinearAccel() const
{
  return this->GetWorldForce() / this->mass.GetAsDouble();
}

////////////////////////////////////////////////////////////////////////////////
/// Get the angular acceleration of the body
common::Vector3 Body::GetRelativeAngularAccel() const
{
  return this->GetRelativeTorque() /  this->mass.GetAsDouble();
}

////////////////////////////////////////////////////////////////////////////////
/// Get the angular acceleration of the body
common::Vector3 Body::GetWorldAngularAccel() const
{
  return this->GetWorldTorque() /  this->mass.GetAsDouble();
}

////////////////////////////////////////////////////////////////////////////////
/// Get the force applied to the body
common::Vector3 Body::GetRelativeForce() const
{
  return this->GetWorldPose().rot.RotateVectorReverse(this->GetWorldForce());
}

////////////////////////////////////////////////////////////////////////////////
/// Get the torque applied to the body
common::Vector3 Body::GetRelativeTorque() const
{
  return this->GetWorldPose().rot.RotateVectorReverse(this->GetWorldTorque());
}

////////////////////////////////////////////////////////////////////////////////
/// Get the model that this body belongs to
ModelPtr Body::GetModel() const
{
  return boost::shared_dynamic_cast<Model>(this->GetParent());
}

////////////////////////////////////////////////////////////////////////////////
/// Get the size of the body
common::Box Body::GetBoundingBox() const
{
  common::Box box;
  Base_V::const_iterator iter;

  box.min.Set(FLT_MAX, FLT_MAX, FLT_MAX);
  box.max.Set(0,0,0);

  for (iter = this->children.begin(); iter != this->children.end(); iter++)
  {
    if ((*iter)->HasType(Base::GEOM))
      box += boost::shared_static_cast<Geom>(*iter)->GetBoundingBox();
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Set whether this entity has been selected by the user through the gui
bool Body::SetSelected( bool s )
{
  Entity::SetSelected(s);

  if (s == false)
    this->SetEnabled(true);
}

////////////////////////////////////////////////////////////////////////////////
// Set Mass
void Body::SetMass(Mass mass)
{
  /// @todo: needs verification
  this->mass = mass;
  this->customMass = mass;
}

////////////////////////////////////////////////////////////////////////////////
/// Return true if auto disable is enabled
bool Body::GetAutoDisable() const
{
  return **this->autoDisableP;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the auto disable flag.
void Body::SetAutoDisable(const bool &value)
{
  this->autoDisableP->SetValue(value);
}
