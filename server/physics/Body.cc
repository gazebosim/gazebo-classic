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
 * SVN: $Id$
 */

#include <sstream>
#include <float.h>

#include "RenderTypes.hh"
#include "Events.hh"
#include "SensorManager.hh"
#include "XMLConfig.hh"
#include "Model.hh"
#include "GazeboMessage.hh"
#include "Geom.hh"
#include "Timer.hh"
#include "Global.hh"
#include "Vector2.hh"
#include "Quatern.hh"
#include "GazeboError.hh"
#include "SensorFactory.hh"
#include "Sensor.hh"
#include "Simulator.hh"
#include "World.hh"
#include "PhysicsEngine.hh"

#include "Body.hh"

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
// Constructor
Body::Body(Entity *parent)
    : Entity(parent)
{
  this->AddType(BODY);

  // NATY: put back in functionality
  // this->GetVisualNode()->SetShowInGui(false);

  this->comEntity = new Entity(this);
  this->comEntity->SetName("COM_Entity");
  this->comEntity->SetShowInGui(false);

  // NATY: put back in functionality
  //this->comEntity->GetVisualNode()->SetShowInGui(false);

  Param::Begin(&this->parameters);
  this->autoDisableP = new ParamT<bool>("auto_disable", true, 0);
  this->autoDisableP->Callback( &Body::SetAutoDisable, this );

  this->xyzP = new ParamT<Vector3>("xyz", Vector3(), 0);
  this->xyzP->Callback( &Entity::SetRelativePosition, (Entity*)this );

  this->rpyP = new ParamT<Quatern>("rpy", Quatern(), 0);
  this->rpyP->Callback( &Entity::SetRelativeRotation, (Entity*)this );
  this->dampingFactorP = new ParamT<double>("damping_factor", 0.0, 0);

  // option to turn gravity off for individual body
  this->turnGravityOffP = new ParamT<bool>("turn_gravity_off", false, 0);

  // option to make body collide with bodies of the same parent
  this->selfCollideP = new ParamT<bool>("self_collide", false, 0);

  // User can specify mass/inertia property for the body
  this->customMassMatrixP = new ParamT<bool>("mass_matrix",false,0);
  this->cxP = new ParamT<double>("cx",0.0,0);
  this->cyP = new ParamT<double>("cy",0.0,0);
  this->czP = new ParamT<double>("cz",0.0,0);
  this->bodyMassP = new ParamT<double>("mass",0.001,0);
  this->ixxP = new ParamT<double>("ixx",1e-6,0);
  this->iyyP = new ParamT<double>("iyy",1e-6,0);
  this->izzP = new ParamT<double>("izz",1e-6,0);
  this->ixyP = new ParamT<double>("ixy",0.0,0);
  this->ixzP = new ParamT<double>("ixz",0.0,0);
  this->iyzP = new ParamT<double>("iyz",0.0,0);

  this->kinematicP = new ParamT<bool>("kinematic",true,0);
  this->kinematicP->SetHelp("true = kinematic state only, false = dynamic body");
  this->kinematicP->Callback( &Body::SetKinematic, this );

  Param::End();

  this->connections.push_back( Events::ConnectShowJointsSignal( boost::bind(&Body::SetTransparent, this, _1) ) );
  this->connections.push_back( Events::ConnectShowPhysicsSignal( boost::bind(&Body::SetTransparent, this, _1) ) );

}


////////////////////////////////////////////////////////////////////////////////
// Destructor
Body::~Body()
{
  std::map< std::string, Geom* >::iterator giter;
  std::vector<Entity*>::iterator iter;
  std::vector< Sensor* >::iterator siter;

  for (unsigned int i=0; i < this->visuals.size(); i++)
  {
    msgs::Visual msg;
    Message::Init(msg, this->visuals[i]);
    msg.set_action( msgs::Visual::DELETE );
    this->vis_pub->Publish(msg);
  }
  this->visuals.clear();

  if (this->cgVisuals.size() > 0)
  {
    for (unsigned int i=0; i < this->cgVisuals.size(); i++)
    {
      msgs::Visual msg;
      Message::Init(msg, this->cgVisuals[i]);
      msg.set_action( msgs::Visual::DELETE );
      this->vis_pub->Publish(msg);
    }
    this->cgVisuals.clear();
  }

  for (giter = this->geoms.begin(); giter != this->geoms.end(); giter++)
    if (giter->second)
      delete giter->second;
  this->geoms.clear();

  for (siter = this->sensors.begin(); siter != this->sensors.end(); siter++)
    SensorManager::Instance()->RemoveSensor(*siter);

  if (this->comEntity)
    delete this->comEntity;
  this->comEntity = NULL;


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
// Load the body based on an XMLConfig node
void Body::Load(XMLConfigNode *node)
{
  Entity::Load(node);
  this->comEntity->RegisterVisual();

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

  this->customMass.SetCoG(**this->cxP, **this->cyP,** this->czP);
  this->customMass.SetInertiaMatrix( **this->ixxP, **this->iyyP, **this->izzP,
                                     **this->ixyP, **this->ixzP, **this->iyzP);
  this->customMass.SetMass(**this->bodyMassP);

  this->mass = this->customMass;
     
  XMLConfigNode *childNode;

  this->xyzP->Load(node);
  this->rpyP->Load(node);
  this->dampingFactorP->Load(node);
  this->turnGravityOffP->Load(node);

  this->SetRelativePose(Pose3d(**this->xyzP, **this->rpyP));
  
  // before loading child geometry, we have to figure out of selfCollide is true
  // and modify parent class Entity so this body has its own spaceId
  this->SetSelfCollide( **this->selfCollideP );

  // save transform from this Parent Model Frame to this Body Frame
  // this is only used in setting Model pose from canonicalBody
  // the true model pose given a canonical body is
  //   this body's pose - this body's offsetFromModelFrame
  this->initModelOffset = this->GetRelativePose().CoordPoseSolve(Pose3d());

  childNode = node->GetChild("geom");

  // Load the geometries
  while (childNode)
  {
    // Create and Load a geom, which will belong to this body.
    this->LoadGeom(childNode);
    childNode = childNode->GetNext("geom");
  }

  childNode = node->GetChild("sensor");

  // Load the sensors
  while (childNode)
  {
    // Create and Load a sensor, which will belong to this body.
    this->LoadSensor(childNode);
    childNode = childNode->GetNext("sensor");
  }

  this->SetKinematic(**this->kinematicP);

  this->showPhysicsConnection = Events::ConnectShowPhysicsSignal( boost::bind(&Body::ShowPhysics, this, _1) );

  childNode = node->GetChild("visual");
  while (childNode)
  {
    std::ostringstream visname;
    visname << this->GetCompleteScopedName() << "::VISUAL_" << 
               this->visuals.size();


    msgs::Visual msg = Message::VisualFromXML(childNode);
    Message::Init(msg, visname.str());
    msg.set_parent_id( this->GetCompleteScopedName() );
    msg.set_render_type( msgs::Visual::MESH_RESOURCE );
    msg.set_cast_shadows( false );

    this->vis_pub->Publish(msg);
    this->visuals.push_back(msg.header().str_id());
   
    childNode = childNode->GetNext("visual");
  }
}

////////////////////////////////////////////////////////////////////////////////
// Save the body based on our XMLConfig node
void Body::Save(std::string &prefix, std::ostream &stream)
{
  std::map<std::string, Geom* >::iterator giter;
  std::vector< Sensor* >::iterator siter;

  this->xyzP->SetValue( this->GetRelativePose().pos );
  this->rpyP->SetValue( this->GetRelativePose().rot );

  stream << prefix << "<body name=\"" << this->nameP->GetValue() << "\">\n";
  stream << prefix << "  " << *(this->xyzP) << "\n";
  stream << prefix << "  " << *(this->rpyP) << "\n";

  std::string p = prefix + "  ";

  for (giter = this->geoms.begin(); giter != this->geoms.end(); giter++)
  {
    stream << "\n";
    giter->second->Save(p, stream);
  }

  for (siter = this->sensors.begin(); siter != this->sensors.end(); siter++)
  {
    stream << "\n";
    (*siter)->Save(p, stream);
  }

  // NATY: put back in functionality
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
  std::vector< Sensor* >::iterator siter;
  std::map< std::string, Geom* >::iterator giter;

  this->connections.clear();

  for (siter = this->sensors.begin(); siter != this->sensors.end(); siter++)
    (*siter)->Fini();

  for (giter = this->geoms.begin(); giter != this->geoms.end(); giter++)
    giter->second->Fini();
}

////////////////////////////////////////////////////////////////////////////////
/// Set the friction mode of the body
void Body::SetFrictionMode( const bool &v )
{
  std::map< std::string, Geom* >::iterator giter;

  for (giter = this->geoms.begin(); giter != this->geoms.end(); giter++)
  {
    giter->second->SetFrictionMode( v );
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Set the collide mode of the body
void Body::SetCollideMode( const std::string &m )
{
  std::map< std::string, Geom* >::iterator giter;

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
    gzerr(0) << "Unknown collide mode[" << m << "]\n";
    return;
  }

  for (giter = this->geoms.begin(); giter != this->geoms.end(); giter++)
  {
    giter->second->SetCategoryBits(collideBits);
    giter->second->SetCollideBits(collideBits);
  }
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
  std::map< std::string, Geom* >::iterator giter;

  for (giter = this->geoms.begin(); giter != this->geoms.end(); giter++)
  {
    giter->second->SetLaserFiducialId( id );
  }

}


////////////////////////////////////////////////////////////////////////////////
/// Set the laser retro reflectiveness of this body
void Body::SetLaserRetro(float retro)
{
  std::map< std::string, Geom* >::iterator giter;

  for (giter = this->geoms.begin(); giter != this->geoms.end(); giter++)
  {
    giter->second->SetLaserRetro( retro );
  }

}


////////////////////////////////////////////////////////////////////////////////
// Initialize the body
void Body::Init()
{
  // If no geoms are attached, then don't let gravity affect the body.
  if (this->geoms.size()==0 || **this->turnGravityOffP)
    this->SetGravityMode(false);

  // global-inertial damping is implemented in ode svn trunk
  if(this->GetId() && this->dampingFactorP->GetValue() > 0)
  {
    this->SetLinearDamping(**this->dampingFactorP);
    this->SetAngularDamping(**this->dampingFactorP);
  }

  std::vector< Sensor* >::iterator siter;
  for (siter = this->sensors.begin(); siter != this->sensors.end(); siter++)
    (*siter)->Init();

  this->linearAccel.Set(0,0,0);
  this->angularAccel.Set(0,0,0);

  /// Attach mesh for CG visualization
  /// Add a renderable visual for CG, make visible in Update()
  if (this->mass.GetAsDouble() > 0.0)
  {
    std::ostringstream visname;
    visname << this->GetCompleteScopedName() + ":" + this->GetName() << "_CGVISUAL" ;

    msgs::Visual msg;
    Message::Init(msg, visname.str());
    msg.set_parent_id( this->comEntity->GetCompleteScopedName() );
    msg.set_render_type( msgs::Visual::MESH_RESOURCE );
    msg.set_mesh( "unit_box" );
    msg.set_material( "Gazebo/RedGlow" );
    msg.set_cast_shadows( false );
    msg.set_attach_axes( true );
    msg.set_visible( false );
    Message::Set(msg.mutable_scale(), Vector3(0.1, 0.1, 0.1));
    this->vis_pub->Publish(msg);
    this->cgVisuals.push_back( msg.header().str_id() );

    if (this->geoms.size() > 1)
    {
      msgs::Visual g_msg;
      Message::Init(g_msg, visname.str() + "_connectors");

      g_msg.set_parent_id( this->comEntity->GetCompleteScopedName() );
      g_msg.set_render_type( msgs::Visual::LINE_LIST );
      g_msg.set_attach_axes( false );
      g_msg.set_material( "Gazebo/GreenGlow" );
      g_msg.set_visible( false );

      // Create a line to each geom
      for (std::map< std::string, Geom* >::iterator giter = this->geoms.begin(); giter != this->geoms.end(); giter++)
      {
        msgs::Point *pt;
        pt = g_msg.add_points();
        pt->set_x(0);
        pt->set_y(0);
        pt->set_z(0);

        pt = g_msg.add_points();
        pt->set_x(giter->second->GetRelativePose().pos.x);
        pt->set_y(giter->second->GetRelativePose().pos.y);
        pt->set_z(giter->second->GetRelativePose().pos.z);
      }
      this->vis_pub->Publish(msg);
      this->cgVisuals.push_back( g_msg.header().str_id() );
    }
  }

  this->enabled = true;
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
// Attach a geom to this body
void Body::AttachGeom( Geom *geom )
{
  std::map<std::string, Geom*>::iterator iter;
  iter = this->geoms.find(geom->GetName());

  if (iter == this->geoms.end())
  {
    this->geoms[geom->GetName()] = geom;

    if (!**this->customMassMatrixP)
    {
      Mass tmpMass = geom->GetMass();
      //tmpMass.Rotate( this->GetRelativePose().rot );
      this->mass += tmpMass;
    }
  }
  else
    gzerr(0) << "Attempting to add two geoms with the same name[" << geom->GetName() << "] to body[" << this->GetName() << "].\n";

}

////////////////////////////////////////////////////////////////////////////////
/// Update the center of mass
void Body::UpdateCoM()
{
  Pose3d bodyPose;
  Pose3d origPose, newPose;
  std::map<std::string, Geom*>::iterator iter;

  bodyPose = this->GetRelativePose();

  if (**this->customMassMatrixP)
  {
    this->mass = this->customMass;
  }

  // Translate all the geoms so that the CoG is at (0,0,0) in the body frame
  for (iter = this->geoms.begin(); iter != this->geoms.end(); iter++)
  {
    Vector3 offset;
    origPose = iter->second->GetRelativePose();
    newPose = origPose;

    newPose.pos -= this->mass.GetCoG();
    iter->second->SetRelativePose(newPose, true);
  }

  this->comEntity->SetRelativePose(Pose3d(this->mass.GetCoG(),Quatern()),true);
  this->OnPoseChange();
}


////////////////////////////////////////////////////////////////////////////////
// Load a new geom helper function
void Body::LoadGeom(XMLConfigNode *node)
{
  Geom *geom = NULL;
  std::string type = node->GetString("type","",1);

  if (type == "heightmap" || type == "map")
    this->SetStatic(true);

  geom = this->GetWorld()->GetPhysicsEngine()->CreateGeom(type, this);

  if (!geom)
    gzthrow("Unknown Geometry Type["+
        node->GetString("name",std::string(),0)+"]");

  geom->Load(node);
}

////////////////////////////////////////////////////////////////////////////////
// Load a sensor
void Body::LoadSensor(XMLConfigNode *node)
{
  Sensor *sensor = NULL;

  if (node==NULL)
  {
    gzthrow("Null node pointer. Invalid sensor in the world file.");
  }

  std::string type = node->GetString("type","",1);

  sensor = SensorFactory::NewSensor(type, this);

  if (sensor)
  {
    sensor->Load(node);
    this->sensors.push_back(sensor);
  }
  else
  {
    std::ostringstream stream;
    stream << "Null sensor. Invalid sensor name[" << node->GetString("name",std::string(), 0) << "]";
    gzthrow(stream.str());
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Set the linear acceleration of the body
void Body::SetLinearAccel(const Vector3 &accel)
{
  //this->SetEnabled(true); Disabled this line to make autoDisable work
  this->linearAccel = accel;// * this->GetMass();
}



////////////////////////////////////////////////////////////////////////////////
/// Set the angular acceleration of the body
void Body::SetAngularAccel(const Vector3 &accel)
{
  //this->SetEnabled(true); Disabled this line to make autoDisable work
  this->angularAccel = accel * this->mass.GetAsDouble();
}

////////////////////////////////////////////////////////////////////////////////
/// Get the linear velocity of the body
Vector3 Body::GetRelativeLinearVel() const
{
  return this->GetWorldPose().rot.RotateVectorReverse(this->GetWorldLinearVel());
}

////////////////////////////////////////////////////////////////////////////////
/// Get the angular velocity of the body
Vector3 Body::GetRelativeAngularVel() const
{
  return this->GetWorldPose().rot.RotateVectorReverse(this->GetWorldAngularVel());
}

////////////////////////////////////////////////////////////////////////////////
/// Get the linear acceleration of the body
Vector3 Body::GetRelativeLinearAccel() const
{
  return this->GetRelativeForce() / this->mass.GetAsDouble();
}

////////////////////////////////////////////////////////////////////////////////
/// Get the angular acceleration of the body
Vector3 Body::GetWorldLinearAccel() const
{
  return this->GetWorldForce() / this->mass.GetAsDouble();
}

////////////////////////////////////////////////////////////////////////////////
/// Get the angular acceleration of the body
Vector3 Body::GetRelativeAngularAccel() const
{
  return this->GetRelativeTorque() /  this->mass.GetAsDouble();
}

////////////////////////////////////////////////////////////////////////////////
/// Get the angular acceleration of the body
Vector3 Body::GetWorldAngularAccel() const
{
  return this->GetWorldTorque() /  this->mass.GetAsDouble();
}

////////////////////////////////////////////////////////////////////////////////
/// Get the force applied to the body
Vector3 Body::GetRelativeForce() const
{
  return this->GetWorldPose().rot.RotateVectorReverse(this->GetWorldForce());
}

////////////////////////////////////////////////////////////////////////////////
/// Get the torque applied to the body
Vector3 Body::GetRelativeTorque() const
{
  return this->GetWorldPose().rot.RotateVectorReverse(this->GetWorldTorque());
}

////////////////////////////////////////////////////////////////////////////////
/// Get the vector of all geoms
const std::map<std::string, Geom*> *Body::GetGeoms() const
{
  return &(this->geoms);
}

////////////////////////////////////////////////////////////////////////////////
/// Get the model that this body belongs to
Model *Body::GetModel() const
{
  return dynamic_cast<Model*>(this->GetParent());
}

////////////////////////////////////////////////////////////////////////////////
/// Get a sensor by name
Sensor *Body::GetSensor( const std::string &name ) const
{
  Sensor *sensor = NULL;
  std::vector< Sensor* >::const_iterator iter;

  for (iter = this->sensors.begin(); iter != this->sensors.end(); iter++)
  {
    if ((*iter)->GetName() == name)
    {
      sensor = (*iter);
      break;
    }
  }

  return sensor;
}

////////////////////////////////////////////////////////////////////////////////
/// Get a geom by name
Geom *Body::GetGeom(const std::string &name) const
{
  std::map<std::string, Geom*>::const_iterator iter = this->geoms.find(name);

  if (iter != this->geoms.end())
    return iter->second;
  else
    return NULL;
}

////////////////////////////////////////////////////////////////////////////////
// Get all children
std::vector< Sensor* > &Body::GetSensors() 
{
  return this->sensors;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the list of interfaces e.g 
/// "pioneer2dx_model1::laser::laser_iface0->laser"
void Body::GetInterfaceNames(std::vector<std::string>& list) const
{
  std::vector< Sensor* >::const_iterator iter;

  for (iter = this->sensors.begin(); iter != this->sensors.end(); iter++)
    (*iter)->GetInterfaceNames(list);
}

////////////////////////////////////////////////////////////////////////////////
/// Get the size of the body
void Body::GetBoundingBox(Vector3 &min, Vector3 &max ) const
{
  Vector3 bbmin;
  Vector3 bbmax;
  std::map<std::string, Geom*>::const_iterator iter;

  min.Set(FLT_MAX, FLT_MAX, FLT_MAX);
  max.Set(0,0,0);

  for (iter = this->geoms.begin(); iter != this->geoms.end(); iter++)
  {
    iter->second->GetBoundingBox(bbmin, bbmax);
    min.x = std::min(bbmin.x, min.x);
    min.y = std::min(bbmin.y, min.y);
    min.z = std::min(bbmin.z, min.z);

    max.x = std::max(bbmax.x, max.x);
    max.y = std::max(bbmax.y, max.y);
    max.z = std::max(bbmax.z, max.z);
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Set to true to show the physics visualizations
void Body::ShowPhysics(const bool &show)
{
  for (unsigned int i=0; i < this->cgVisuals.size(); i++)
  {
    msgs::Visual msg;
    Message::Init(msg, this->cgVisuals[i] );
    msg.set_visible( show );
    this->vis_pub->Publish(msg);
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

////////////////////////////////////////////////////////////////////////////////
/// Set the transparency
void Body::SetTransparent(const bool &show)
{
  for (unsigned int i = 0; i < this->visuals.size(); i++)
  {
    msgs::Visual msg;
    Message::Init(msg, this->visuals[i]);
    msg.set_action( msgs::Visual::UPDATE );
    if (show)
      msg.set_transparency( 0.6 );
    else
      msg.set_transparency( 0.0 );
    this->vis_pub->Publish(msg);
  }
}
