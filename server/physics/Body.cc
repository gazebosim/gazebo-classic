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
/* Desc: Body class
 * Author: Nate Koenig
 * Date: 13 Feb 2006
 */

#include <sstream>
#include <float.h>

#include "SensorManager.hh"
#include "XMLConfig.hh"
#include "Model.hh"
#include "GazeboMessage.hh"
#include "Geom.hh"
#include "Timer.hh"
#include "OgreVisual.hh"
#include "OgreCreator.hh"
#include "OgreDynamicLines.hh"
#include "Global.hh"
#include "Vector2.hh"
#include "Quatern.hh"
#include "GazeboError.hh"
#include "SensorFactory.hh"
#include "Sensor.hh"
#include "Simulator.hh"
#include "World.hh"
#include "Controller.hh"
#include "ControllerFactory.hh"
#include "PhysicsEngine.hh"

#include "Body.hh"

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
// Constructor
Body::Body(Entity *parent)
    : Entity(parent)
{
  this->type = Entity::BODY;
  this->comEntity = new Entity(this);
  this->comEntity->SetName("COM_Entity");

  this->physicsEngine = World::Instance()->GetPhysicsEngine();

  this->cgVisual = NULL;

  Param::Begin(&this->parameters);
  this->xyzP = new ParamT<Vector3>("xyz", Vector3(), 0);
  this->xyzP->Callback( &Entity::SetRelativePosition, (Entity*)this );

  this->rpyP = new ParamT<Quatern>("rpy", Quatern(), 0);
  this->rpyP->Callback( &Entity::SetRelativeRotation, (Entity*)this );
  this->dampingFactorP = new ParamT<double>("dampingFactor", 0.0, 0);

  // option to turn gravity off for individual body
  this->turnGravityOffP = new ParamT<bool>("turnGravityOff", false, 0);

  // option to make body collide with bodies of the same parent
  this->selfCollideP = new ParamT<bool>("selfCollide", false, 0);

  // User can specify mass/inertia property for the body
  this->customMassMatrixP = new ParamT<bool>("massMatrix",false,0);
  this->cxP = new ParamT<double>("cx",0.0,0);
  this->cyP = new ParamT<double>("cy",0.0,0);
  this->czP = new ParamT<double>("cz",0.0,0);
  this->bodyMassP = new ParamT<double>("mass",0.0,0);
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

}


////////////////////////////////////////////////////////////////////////////////
// Destructor
Body::~Body()
{
  std::vector< Geom* >::iterator giter;
  std::vector<Entity*>::iterator iter;
  std::vector< Sensor* >::iterator siter;

  if (this->cgVisual)
  {
    OgreCreator::Instance()->DeleteVisual( this->cgVisual );
    this->cgVisual = NULL;
  }

  for (giter = this->geoms.begin(); giter != this->geoms.end(); giter++)
    delete (*giter);
  this->geoms.clear();

  for (siter = this->sensors.begin(); siter != this->sensors.end(); siter++)
    SensorManager::Instance()->RemoveSensor(*siter);

  if (this->comEntity)
    delete this->comEntity;
  this->comEntity = NULL;

  std::map< std::string, Controller* >::iterator citer;
  for (citer = this->controllers.begin();
       citer != this->controllers.end(); citer++)
  {
    if (citer->second)
    {
      delete citer->second;
      citer->second = NULL;
    }
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
// Load the body based on an XMLConfig node
void Body::Load(XMLConfigNode *node)
{
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

  this->nameP->Load(node);
  this->xyzP->Load(node);
  this->rpyP->Load(node);
  this->dampingFactorP->Load(node);
  this->turnGravityOffP->Load(node);

  this->SetRelativePose(Pose3d(**this->xyzP, **this->rpyP));
  
  // Load controllers
  childNode = node->GetChildByNSPrefix("controller");
  while (childNode)
  {
    this->LoadController(childNode);
    childNode = childNode->GetNextByNSPrefix("controller");
  }

  // before loading child geometry, we have to figure out of selfCollide is true
  // and modify parent class Entity so this body has its own spaceId
  this->SetSelfCollide( **this->selfCollideP );

  // save transform from this Parent Model Frame to this Body Frame
  // this is only used in setting Model pose from canonicalBody
  // the true model pose given a canonical body is
  //   this body's pose - this body's offsetFromModelFrame
  this->initModelOffset = this->GetRelativePose().CoordPoseSolve(Pose3d());

  childNode = node->GetChildByNSPrefix("geom");

  // Load the geometries
  while (childNode)
  {
    // Create and Load a geom, which will belong to this body.
    this->LoadGeom(childNode);
    childNode = childNode->GetNextByNSPrefix("geom");
  }

  childNode = node->GetChildByNSPrefix("sensor");

  // Load the sensors
  while (childNode)
  {
    // Create and Load a sensor, which will belong to this body.
    this->LoadSensor(childNode);
    childNode = childNode->GetNextByNSPrefix("sensor");
  }

  this->SetKinematic(**this->kinematicP);

  //this->GetModel()->ConnectUpdateSignal( boost::bind(&Body::Update, this) );
}

////////////////////////////////////////////////////////////////////////////////
// Save the body based on our XMLConfig node
void Body::Save(std::string &prefix, std::ostream &stream)
{
  std::vector< Geom* >::iterator giter;
  std::vector< Sensor* >::iterator siter;
  std::map<std::string, Controller* >::iterator contIter;

  this->xyzP->SetValue( this->GetRelativePose().pos );
  this->rpyP->SetValue( this->GetRelativePose().rot );

  stream << prefix << "<body name=\"" << this->nameP->GetValue() << "\">\n";
  stream << prefix << "  " << *(this->xyzP) << "\n";
  stream << prefix << "  " << *(this->rpyP) << "\n";

  std::string p = prefix + "  ";

  for (giter = this->geoms.begin(); giter != this->geoms.end(); giter++)
  {
    stream << "\n";
    (*giter)->Save(p, stream);
  }

  for (siter = this->sensors.begin(); siter != this->sensors.end(); siter++)
  {
    stream << "\n";
    (*siter)->Save(p, stream);
  }

  // Save all the controllers
  for (contIter=this->controllers.begin();
      contIter!=this->controllers.end(); contIter++)
  {
    if (contIter->second)
      contIter->second->Save(p, stream);
  }

  stream << prefix << "</body>\n";
}


////////////////////////////////////////////////////////////////////////////////
// Finalize the body
void Body::Fini()
{
  std::map<std::string, Controller* >::iterator contIter;
  for (contIter = this->controllers.begin();
       contIter != this->controllers.end(); contIter++)
  {
    contIter->second->Fini();
  }

  std::vector< Sensor* >::iterator siter;
  std::vector< Geom* >::iterator giter;

  for (siter = this->sensors.begin(); siter != this->sensors.end(); siter++)
    (*siter)->Fini();

  for (giter = this->geoms.begin(); giter != this->geoms.end(); giter++)
    (*giter)->Fini();
}

////////////////////////////////////////////////////////////////////////////////
/// Set the friction mode of the body
void Body::SetFrictionMode( const bool &v )
{
  std::vector< Geom* >::iterator giter;

  for (giter = this->geoms.begin(); giter != this->geoms.end(); giter++)
  {
    (*giter)->SetFrictionMode( v );
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Set the collide mode of the body
void Body::SetCollideMode( const std::string &m )
{
  std::vector< Geom* >::iterator giter;

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
    (*giter)->SetCategoryBits(collideBits);
    (*giter)->SetCollideBits(collideBits);
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
  std::vector< Geom* >::iterator giter;

  for (giter = this->geoms.begin(); giter != this->geoms.end(); giter++)
  {
    (*giter)->SetLaserFiducialId( id );
  }

}


////////////////////////////////////////////////////////////////////////////////
/// Set the laser retro reflectiveness of this body
void Body::SetLaserRetro(float retro)
{
  std::vector< Geom* >::iterator giter;

  for (giter = this->geoms.begin(); giter != this->geoms.end(); giter++)
  {
    (*giter)->SetLaserRetro( retro );
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

    if (this->cgVisual == NULL)
    {
      this->cgVisual = OgreCreator::Instance()->CreateVisual(visname.str(),
          this->comEntity->GetVisualNode());
    }
    else
      this->cgVisual->DetachObjects();

    if (this->cgVisual)
    {
      this->cgVisual->AttachMesh("body_cg");
      this->cgVisual->SetMaterial("Gazebo/Red");
      this->cgVisual->SetCastShadows(false);
      this->cgVisual->AttachAxes();

      std::vector< Geom* >::iterator giter;

      // Create a line to each geom
      for (giter = this->geoms.begin(); giter != this->geoms.end(); giter++)
      {
        OgreDynamicLines *line = OgreCreator::Instance()->CreateDynamicLine(OgreDynamicRenderable::OT_LINE_LIST);
        line->setMaterial("Gazebo/GreenEmissive");
        this->cgVisual->AttachObject(line);
        line->AddPoint(Vector3(0,0,0));
        line->AddPoint((*giter)->GetRelativePose().pos);
      }

      this->cgVisual->SetVisible(false);
    }
  }

  std::map<std::string, Controller* >::iterator contIter;
  for (contIter=this->controllers.begin();
       contIter!=this->controllers.end(); contIter++)
  {
    contIter->second->Init();
  }

  this->enabled = true;
}

////////////////////////////////////////////////////////////////////////////////
// Update the body
void Body::Update()
{
  //DiagnosticTimer timer("Body[" + this->GetName() +"] Update");
  
  std::vector< Geom* >::iterator geomIter;
  Vector3 vel;
  Vector3 avel;

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
 
  {
    //DiagnosticTimer timer("Body[" + this->GetName() +"] Update Geoms");

    for (geomIter=this->geoms.begin();
         geomIter!=this->geoms.end(); geomIter++)
    {
#ifdef USE_THREADPOOL
      World::Instance()->threadPool->schedule(boost::bind(&Geom::Update, (*geomIter)));
#else
      (*geomIter)->Update();
#endif
    }
  }

  std::map<std::string, Controller* >::iterator contIter;
  {
    //DiagnosticTimer timer("Model[" + this->GetName() + "] Controllers Update ");
    for (contIter=this->controllers.begin();
        contIter!=this->controllers.end(); contIter++)
    {
      if (contIter->second)
      {
#ifdef USE_THREADPOOL
        World::Instance()->threadPool->schedule(boost::bind(&Controller::Update,(contIter->second)));
#else
        contIter->second->Update();
#endif
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
// Attach a geom to this body
void Body::AttachGeom( Geom *geom )
{
  Geom *g = this->GetGeom(geom->GetName());

  if (g)
    gzthrow("Attempting to add two geoms with the same name[" << geom->GetName() << "] to body[" << this->GetName() << "].");

  this->geoms.push_back(geom);

  if (!**this->customMassMatrixP)
  {
    Mass tmpMass = geom->GetMass();
    //tmpMass.Rotate( this->GetRelativePose().rot );
    this->mass += tmpMass;
  }
}

void Body::DettachGeom(Geom *geom)
{
  Geom *g = this->GetGeom(geom->GetName());
  if (!g)
    gzthrow("Attemping to remove a geom that doesn't belong to this body\n");

  std::vector<Geom*>::iterator iter;
  for (iter = this->geoms.begin(); iter != this->geoms.end(); iter++)
  {
    if ((*iter)->GetName() == geom->GetName())
    {
      this->geoms.erase(iter);
      this->comEntity->RemoveChild(geom);
      break;
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Update the center of mass
void Body::UpdateCoM()
{
  Pose3d bodyPose;
  Pose3d origPose, newPose;
  std::vector<Geom*>::iterator iter;

  std::cout << "UpdateCOM for Body[" << this->GetName() << "]\n";
  bodyPose = this->GetRelativePose();

  if (**this->customMassMatrixP)
  {
    this->mass = this->customMass;
  }

  // Translate all the geoms so that the CoG is at (0,0,0) in the body frame
  for (iter = this->geoms.begin(); iter != this->geoms.end(); iter++)
  {
    Vector3 offset;
    origPose = (*iter)->GetRelativePose() + this->comEntity->GetRelativePose();
    newPose = origPose;

    newPose.pos -= this->mass.GetCoG();
    std::cout << "  Move Geom[" << (*iter)->GetName() << "] Old[" << origPose.pos << "] to [" << newPose.pos << "]\n";
    (*iter)->SetRelativePose(newPose, true);
  }

  this->comEntity->SetRelativePose(Pose3d(this->mass.GetCoG(),Quatern()),true);
  this->OnPoseChange();
}


////////////////////////////////////////////////////////////////////////////////
// Load a new geom helper function
void Body::LoadGeom(XMLConfigNode *node)
{
  Geom *geom = NULL;

  if (node->GetName() == "heightmap" || node->GetName() == "map")
    this->SetStatic(true);

  geom = this->physicsEngine->CreateGeom(node->GetName(), this);

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

  sensor = SensorFactory::NewSensor(node->GetName(), this);

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
  this->SetEnabled(true);
  this->linearAccel = accel;// * this->GetMass();
}



////////////////////////////////////////////////////////////////////////////////
/// Set the angular acceleration of the body
void Body::SetAngularAccel(const Vector3 &accel)
{
  this->SetEnabled(true);
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
/// Get the number of geoms
unsigned int Body::GetGeomCount() const
{
  return this->geoms.size();
}

////////////////////////////////////////////////////////////////////////////////
/// Get a geoms
Geom *Body::GetGeom(unsigned int index) const
{
  if (index < this->geoms.size())
    return this->geoms[index];
  else
    gzerr(0) << "Invalid index\n";
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
  std::vector<Geom*>::const_iterator iter;

  for (iter = this->geoms.begin(); iter != this->geoms.end(); iter++)
    if ((*iter)->GetName() == name || (*iter)->HasAltName(name))
      return (*iter);
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
  std::vector<Geom*>::const_iterator iter;

  min.Set(FLT_MAX, FLT_MAX, FLT_MAX);
  max.Set(0,0,0);

  for (iter = this->geoms.begin(); iter != this->geoms.end(); iter++)
  {
    (*iter)->GetBoundingBox(bbmin, bbmax);
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
void Body::ShowPhysics(bool show)
{
  if (this->cgVisual)
    this->cgVisual->SetVisible(show);
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


void Body::AddSensor( Sensor *sensor )
{
  this->sensors.push_back(sensor);
}

void Body::ClearSensors( )
{
  this->sensors.clear();
}

////////////////////////////////////////////////////////////////////////////////
/// Load a controller helper function
void Body::LoadController(XMLConfigNode *node)
{
  if (!node)
    gzthrow( "node parameter is NULL" );

  Controller *controller=0;
  std::ostringstream stream;

  // Get the controller's type
  std::string controllerType = node->GetName();

  // Get the unique name of the controller
  std::string controllerName = node->GetString("name",std::string(),1);
  
  // See if the controller is in a plugin
  std::string pluginName = node->GetString("plugin","",0);
  if (pluginName != "")
	  ControllerFactory::LoadPlugin(pluginName, controllerType);

  // Create the controller based on it's type
  controller = ControllerFactory::NewController(controllerType, this);

  if (controller)
  {
    // Load the controller
    try
    {
      controller->Load(node);
    }
    catch (GazeboError e)
    {
      gzerr(0) << "Error Loading Controller[" <<  controllerName
      << "]\n" << e << std::endl;
      delete controller;
      controller = NULL;
      return;
    }

    // Store the controller
    this->controllers[controllerName] = controller;
  }
  else
  {
    gzmsg(0) << "Unknown controller[" << controllerType << "]\n";
  }
}
