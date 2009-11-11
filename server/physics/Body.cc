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
 * SVN: $Id$
 */

#include <sstream>
#include <float.h>

#include "XMLConfig.hh"
#include "Model.hh"
#include "GazeboMessage.hh"
#include "Geom.hh"

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
#include "PhysicsEngine.hh"

#include "Body.hh"

#ifdef TIMING
#include "Simulator.hh"// for timing
#endif

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
// Constructor
Body::Body(Entity *parent)
    : Entity(parent)
{
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
  this->bodyMassP = new ParamT<double>("mass",0.001,0);
  this->ixxP = new ParamT<double>("ixx",1e-6,0);
  this->iyyP = new ParamT<double>("iyy",1e-6,0);
  this->izzP = new ParamT<double>("izz",1e-6,0);
  this->ixyP = new ParamT<double>("ixy",0.0,0);
  this->ixzP = new ParamT<double>("ixz",0.0,0);
  this->iyzP = new ParamT<double>("iyz",0.0,0);

  Param::End();
}


////////////////////////////////////////////////////////////////////////////////
// Destructor
Body::~Body()
{
  std::map< std::string, Geom* >::iterator giter;
  std::vector< Sensor* >::iterator siter;

  for (giter = this->geoms.begin(); giter != this->geoms.end(); giter++)
  {
    if (giter->second)
      delete giter->second;
    giter->second = NULL;
  }
  this->geoms.clear();

  for (siter = this->sensors.begin(); siter != this->sensors.end(); siter++)
  {
    if (*siter)
      delete (*siter);
    (*siter) = NULL;
  }
  this->sensors.clear();

  if (this->cgVisual)
    delete this->cgVisual;

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

  stream << prefix << "</body>\n";
}


////////////////////////////////////////////////////////////////////////////////
// Finalize the body
void Body::Fini()
{
  std::vector< Sensor* >::iterator siter;

  for (siter = this->sensors.begin(); siter != this->sensors.end(); siter++)
  {
    (*siter)->Fini();
  }
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

  std::vector< Sensor* >::iterator siter;

  for (siter = this->sensors.begin(); siter != this->sensors.end(); siter++)
    (*siter)->Init();

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
  if (this->mass.GetAsDouble() > 0.0)
  {
    std::ostringstream visname;
    visname << this->GetCompleteScopedName() + ":" + this->GetName() << "_CGVISUAL" ;

    if (this->cgVisual == NULL)
      this->cgVisual = OgreCreator::Instance()->CreateVisual(visname.str(),
          this->GetVisualNode());
    else
      this->cgVisual->DetachObjects();

    if (this->cgVisual)
    {
      this->cgVisual->AttachMesh("body_cg");
      this->cgVisual->SetMaterial("Gazebo/Red");
      this->cgVisual->SetCastShadows(false);

      std::map< std::string, Geom* >::iterator giter;

      // Create a line to each geom
      for (giter = this->geoms.begin(); giter != this->geoms.end(); giter++)
      {
        OgreDynamicLines *line = OgreCreator::Instance()->CreateDynamicLine(OgreDynamicRenderable::OT_LINE_LIST);
        line->setMaterial("Gazebo/GreenEmissive");
        this->cgVisual->AttachObject(line);
        line->AddPoint(Vector3(0,0,0));
        line->AddPoint(giter->second->GetRelativePose().pos);
      }

      this->cgVisual->SetVisible(false);
    }
  }


}

////////////////////////////////////////////////////////////////////////////////
// Update the body
void Body::Update()
{
#ifdef USE_THREADPOOL
  // If calling Body::Update in threadpool
  World::Instance()->GetPhysicsEngine()->InitForThread();
#endif

  std::vector< Sensor* >::iterator sensorIter;
  std::map< std::string, Geom* >::iterator geomIter;
  Vector3 vel;
  Vector3 avel;

#ifdef TIMING
  double tmpT1 = Simulator::Instance()->GetWallTime();
#endif

#ifdef TIMING
  double tmpT2 = Simulator::Instance()->GetWallTime();
  std::cout << "           Body Name (" << this->nameP->GetValue() << ")" << std::endl;
  std::cout << "               UpdatePose dt (" << tmpT2-tmpT1 << ")" << std::endl;
#endif

  // Apply our linear accel
  this->SetForce(this->linearAccel);

  // Apply our angular accel
  this->SetTorque(this->angularAccel);

#ifdef TIMING
  double tmpT3 = Simulator::Instance()->GetWallTime();
  std::cout << "               Static SetPose dt (" << tmpT3-tmpT2 << ")" << std::endl;
#endif

  for (geomIter=this->geoms.begin();
       geomIter!=this->geoms.end(); geomIter++)
  {
#ifdef USE_THREADPOOL
    World::Instance()->threadpool->schedule(boost::bind(&Geom::Update, (geomIter->second)));
#else
    geomIter->second->Update();
#endif
  }

#ifdef TIMING
  double tmpT4 = Simulator::Instance()->GetWallTime();
  std::cout << "               Geom Update DT (" << tmpT4-tmpT3 << ")" << std::endl;
#endif

  for (sensorIter=this->sensors.begin();
       sensorIter!=this->sensors.end(); sensorIter++)
  {
#ifdef USE_THREADPOOL
    World::Instance()->threadpool->schedule(boost::bind(&Sensor::Update, (sensorIter)));
#else
    (*sensorIter)->Update();
#endif
  }

#ifdef TIMING
  double tmpT5 = Simulator::Instance()->GetWallTime();
  std::cout << "               ALL Sensors Update DT (" << tmpT5-tmpT4 << ")" << std::endl;
  std::cout << "               Body::Update Total DT (" << tmpT5-tmpT1 << ")" << std::endl;
#endif

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
      tmpMass.Rotate( this->GetRelativePose().rot );
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

  if (**this->customMassMatrixP)
    this->mass = this->customMass;

  bodyPose = this->GetRelativePose();

  // Translate all the geoms so that the CoG is at (0,0,0) in the body frame
  for (iter = this->geoms.begin(); iter != this->geoms.end(); iter++)
  {
    Vector3 offset;
    origPose = iter->second->GetRelativePose();
    newPose = origPose;

    offset = bodyPose.rot.GetInverse().RotateVector(this->mass.GetCoG());
    newPose.pos -= offset;
    iter->second->SetRelativePose(newPose, true);
  }

  // Apply the CoG offset to the body
  Pose3d p = this->GetRelativePose();
  p.pos += this->mass.GetCoG();

  this->SetRelativePose( p, true );
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
/// Get the Center of Mass pose
const Pose3d &Body::GetCoMPose() const
{
  return this->comPose;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the linear acceleration of the body
void Body::SetLinearAccel(const Vector3 &accel)
{
  this->linearAccel = accel;// * this->GetMass();
}

////////////////////////////////////////////////////////////////////////////////
/// Get the angular acceleration of the body
Vector3 Body::GetLinearAccel() const
{
  return this->GetForce() / this->mass.GetAsDouble();
}

////////////////////////////////////////////////////////////////////////////////
/// Set the angular acceleration of the body
void Body::SetAngularAccel(const Vector3 &accel)
{
  this->angularAccel = accel * this->mass.GetAsDouble();
}

////////////////////////////////////////////////////////////////////////////////
/// Get the angular acceleration of the body
Vector3 Body::GetAngularAccel() const
{
  return this->GetTorque() /  this->mass.GetAsDouble();
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
void Body::ShowPhysics(bool show)
{
  if (this->cgVisual)
    this->cgVisual->SetVisible(show);
}


