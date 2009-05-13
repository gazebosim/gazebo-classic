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

#include "XMLConfig.hh"
#include "Model.hh"
#include "GazeboMessage.hh"

#include "HeightmapGeom.hh"
#include "MapGeom.hh"
#include "SphereGeom.hh"
#include "TrimeshGeom.hh"
#include "BoxGeom.hh"
#include "CylinderGeom.hh"
#include "PlaneGeom.hh"
#include "Geom.hh"

#include "OgreVisual.hh"
#include "OgreCreator.hh"
#include "Global.hh"
#include "Vector2.hh"
#include "Quatern.hh"
#include "GazeboError.hh"
#include "SensorFactory.hh"
#include "Sensor.hh"
#include "Simulator.hh"
#include "World.hh"
#include "ODEPhysics.hh"
#include "PhysicsEngine.hh"

#include "Body.hh"

#ifdef TIMING
#include "Simulator.hh"// for timing
#endif

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
// Constructor
Body::Body(Entity *parent, dWorldID worldId)
    : Entity(parent)
{
  this->physicsEngine = World::Instance()->GetPhysicsEngine();

  if ( !this->IsStatic() )
  {
    this->bodyId = dBodyCreate(worldId);

    dMassSetZero( &this->mass );
  }
  else
  {
    this->bodyId = NULL;
  }

  Param::Begin(&this->parameters);
  this->xyzP = new ParamT<Vector3>("xyz", Vector3(), 0);
  this->xyzP->Callback( &Body::SetPosition, this );

  this->rpyP = new ParamT<Quatern>("rpy", Quatern(), 0);
  this->rpyP->Callback( &Body::SetRotation, this );
  this->dampingFactorP = new ParamT<double>("dampingFactor", 0.03, 0);

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
  if (this->selfCollideP->GetValue())
  {
    //std::cout << "setting self collide: " << this->nameP->GetValue() << std::endl;
    ODEPhysics* pe = dynamic_cast<ODEPhysics*>(World::Instance()->GetPhysicsEngine());
    this->spaceId = dSimpleSpaceCreate( pe->spaceId);
  }

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

  this->customMassMatrix = this->customMassMatrixP->GetValue();
  this->cx  = this->cxP ->GetValue();
  this->cy  = this->cyP ->GetValue();
  this->cz  = this->czP ->GetValue();
  this->bodyMass = this->bodyMassP->GetValue();
  this->ixx = this->ixxP->GetValue();
  this->iyy = this->iyyP->GetValue();
  this->izz = this->izzP->GetValue();
  this->ixy = this->ixyP->GetValue();
  this->ixz = this->ixzP->GetValue();
  this->iyz = this->iyzP->GetValue();

  XMLConfigNode *childNode;

  this->nameP->Load(node);
  this->xyzP->Load(node);
  this->rpyP->Load(node);
  this->dampingFactorP->Load(node);
  this->turnGravityOffP->Load(node);

  Pose3d initPose;

  initPose.pos = **(this->xyzP);
  initPose.rot = **(this->rpyP);

  childNode = node->GetChildByNSPrefix("geom");

  // Load the geometries
  while (childNode)
  {
    // Create and Load a geom, which will belong to this body.
    this->LoadGeom(childNode);
    childNode = childNode->GetNextByNSPrefix("geom");
  }

  /// Attach mesh for CG visualization
  /// Add a renderable visual for CG, make visible in Update()
  if (this->mass.mass > 0.0)
  {
    std::ostringstream visname;
    visname << this->GetName() << "_CGVISUAL" ;

    this->cgVisual = OgreCreator::Instance()->CreateVisual(visname.str(),
                        this->GetModel()->GetVisualNode());
    if (this->cgVisual)
    {
      this->cgVisual->AttachMesh("body_cg");
      this->cgVisual->SetVisible(false);
      this->cgVisual->SetMaterial("Gazebo/Red");
    }
  }


  childNode = node->GetChildByNSPrefix("sensor");

  // Load the sensors
  while (childNode)
  {
    // Create and Load a sensor, which will belong to this body.
    this->LoadSensor(childNode);
    childNode = childNode->GetNextByNSPrefix("sensor");
  }

  // If no geoms are attached, then don't let gravity affect the body.
  if (this->geoms.size()==0 || this->turnGravityOffP->GetValue())
  {
    //std::cout << "setting gravity to zero for: " << this->nameP->GetValue() << std::endl;
    this->SetGravityMode(false);
  }

  this->SetPose(initPose);

}

////////////////////////////////////////////////////////////////////////////////
// Save the body based on our XMLConfig node
void Body::Save(std::string &prefix, std::ostream &stream)
{
  std::map<std::string, Geom* >::iterator giter;
  std::vector< Sensor* >::iterator siter;
  Model *model = dynamic_cast<Model*>(this->parent);
  //Vector3 pose = model->GetPose() - this->GetPose();

  this->xyzP->SetValue( this->GetPose().pos - model->GetPose().pos );
  this->rpyP->SetValue( this->GetRotation() );

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
// Set whether gravity affects this body
void Body::SetGravityMode(bool mode)
{
  if (this->bodyId)
  {
    this->physicsEngine->LockMutex();
    dBodySetGravityMode(this->bodyId, mode ? 1: 0);
    this->physicsEngine->UnlockMutex();
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
// Initialize the body
void Body::Init()
{
  // Set the intial pose. Must do this to handle static models
  this->SetPose(this->GetPose());

  std::vector< Sensor* >::iterator siter;

  for (siter = this->sensors.begin(); siter != this->sensors.end(); siter++)
  {
    (*siter)->Init();
  }
}

////////////////////////////////////////////////////////////////////////////////
// Update the body
void Body::Update()
{
  std::vector< Sensor* >::iterator sensorIter;
  std::map< std::string, Geom* >::iterator geomIter;
  Vector3 vel;
  Vector3 avel;

  double force;

#ifdef TIMING
  double tmpT1 = Simulator::Instance()->GetWallTime();
#endif

  this->UpdatePose();

#ifdef TIMING
  double tmpT2 = Simulator::Instance()->GetWallTime();
  std::cout << "           Body Name (" << this->nameP->GetValue() << ")" << std::endl;
  std::cout << "               UpdatePose dt (" << tmpT2-tmpT1 << ")" << std::endl;
#endif

  // Merged from ogre-1.4.9 branch...not sure if we want this
  /*if (!this->IsStatic())
  {
    // Set the pose of the scene node
      this->visualNode->SetPose(this->pose);

      if (this->mass.mass > 0.0)
      {
        // Visualization of Body CG
        if (this->cgVisual)
        {
          this->cgVisual->SetVisible(true);

          // set visual pose of CG (CoM)
          // comPose is only good for initial pose, to represent dynamically changing CoM, rotate by pose.rot
          this->cgVisual->SetPose(this->pose + this->comPose.RotatePositionAboutOrigin(this->pose.rot.GetInverse()));
        }
      }
  }*/

#ifdef TIMING
  double tmpT3 = Simulator::Instance()->GetWallTime();
  std::cout << "               Static SetPose dt (" << tmpT3-tmpT2 << ")" << std::endl;
#endif

  for (geomIter=this->geoms.begin();
       geomIter!=this->geoms.end(); geomIter++)
  {
    geomIter->second->Update();
  }

#ifdef TIMING
  double tmpT4 = Simulator::Instance()->GetWallTime();
  std::cout << "               Geom Update DT (" << tmpT4-tmpT3 << ")" << std::endl;
#endif

  for (sensorIter=this->sensors.begin();
       sensorIter!=this->sensors.end(); sensorIter++)
  {
    (*sensorIter)->Update();
  }

  if(this->GetId())
  {
    this->physicsEngine->LockMutex();

	  force = this->dampingFactorP->GetValue() * this->mass.mass;
	  vel = this->GetLinearVel();
	  dBodyAddForce(this->GetId(), -((vel.x * fabs(vel.x)) * force), 
                  -((vel.y * fabs(vel.y)) * force), 
                  -((vel.z * fabs(vel.z)) * force));

	  avel = this->GetAngularVel();
	  dBodyAddTorque(this->GetId(), -avel.x * force, -avel.y * force, 
                   -avel.z * force);

    this->physicsEngine->UnlockMutex();
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
  if ( this->bodyId )
  {
    if (geom->IsPlaceable())
    {

      this->physicsEngine->LockMutex();

      if (geom->GetTransId())
        dGeomSetBody(geom->GetTransId(), this->bodyId);
      else if (geom->GetGeomId())
        dGeomSetBody(geom->GetGeomId(), this->bodyId);

      this->physicsEngine->UnlockMutex();
    }
  }

  std::map<std::string, Geom*>::iterator iter = this->geoms.find(geom->GetName());

  if (iter == this->geoms.end())
    this->geoms[geom->GetName()] = geom;
  else
    gzerr(0) << "Attempting to add two geoms with the same name[" << geom->GetName() << "] to body[" << this->GetName() << "].\n";

}

////////////////////////////////////////////////////////////////////////////////
// Set the pose of the body
void Body::SetPose(const Pose3d &_pose)
{

  // check if pose is NaN, if so, constraint solver likely blew up
  if (std::isnan(pose.pos.x) || std::isnan(pose.pos.y) || 
      std::isnan(pose.pos.z) || std::isnan(pose.rot.u) || 
      std::isnan(pose.rot.x) || std::isnan(pose.rot.y) || 
      std::isnan(pose.rot.z))
  {
    std::cout << "Trying to SetPose() for Body(" << this->GetName() 
              << ") with some NaN's (" << pose << ")" << std::endl;
    return;
  }

  if (this->IsStatic())
  {
    Pose3d oldPose = this->staticPose;
    Pose3d newPose;
    this->staticPose = _pose;

    this->pose = this->staticPose;

    std::map<std::string, Geom*>::iterator iter;

    // This loop doesn't work properly when rotating objects
    for (iter = this->geoms.begin(); iter != this->geoms.end(); iter++)
    {
      newPose = iter->second->GetPose() - oldPose;
      newPose += this->staticPose;
      iter->second->SetPose(newPose);
    }
  }
  else
  {
    Pose3d localPose;

    // Compute pose of CoM
    localPose = this->comPose + _pose;

    this->pose = localPose;
    this->SetPosition(localPose.pos);
    this->SetRotation(localPose.rot);
  }
}

////////////////////////////////////////////////////////////////////////////////
// Return the pose of the body
Pose3d Body::GetPose() const
{

  /*if (this->IsStatic())
    return this->staticPose;
  else
  */
  return this->pose;
}

////////////////////////////////////////////////////////////////////////////////
// Update the pose of the body
void Body::UpdatePose()
{
  this->pose.pos = this->GetPosition();
  this->pose.rot = this->GetRotation();

  this->pose = this->comPose.CoordPoseSolve(pose);
}

////////////////////////////////////////////////////////////////////////////////
// Set the position of the body
void Body::SetPosition(const Vector3 &pos)
{
  if (this->bodyId)
  {
    this->physicsEngine->LockMutex();
    dBodySetPosition(this->bodyId, pos.x, pos.y, pos.z);
    this->physicsEngine->UnlockMutex();
  }
}

////////////////////////////////////////////////////////////////////////////////
// Set the rotation of the body
void Body::SetRotation(const Quatern &rot)
{
  if (this->bodyId)
  {
    this->physicsEngine->LockMutex();

    dQuaternion q;
    q[0] = rot.u;
    q[1] = rot.x;
    q[2] = rot.y;
    q[3] = rot.z;

    // Set the rotation of the ODE body
    dBodySetQuaternion(this->bodyId, q);

    this->physicsEngine->UnlockMutex();
  }
}

////////////////////////////////////////////////////////////////////////////////
// Return the position of the body. in global CS
Vector3 Body::GetPosition() const
{
  Vector3 pos;

  if (this->bodyId)
  {
    this->physicsEngine->LockMutex();
    const dReal *p;

    p = dBodyGetPosition(this->bodyId);

    pos.x = p[0];
    pos.y = p[1];
    pos.z = p[2];

    // check for NaN
    if (std::isnan(pos.x) || std::isnan(pos.y) || std::isnan(pos.z))
    {
      std::cout << "Your simulation has exploded, position of body(" << this->GetName() << ") has NaN(" << pos << ")" << std::endl;
      //pos = this->pose.pos;
      assert(0);
    }

    this->physicsEngine->UnlockMutex();
  }
  else
  {
    pos = this->staticPose.pos;
  }

  return pos;
}


////////////////////////////////////////////////////////////////////////////////
// Return the rotation
Quatern Body::GetRotation() const
{
  Quatern rot;

  if (this->bodyId)
  {
    const dReal *r;

    this->physicsEngine->LockMutex();
    r = dBodyGetQuaternion(this->bodyId);
    this->physicsEngine->UnlockMutex();

    rot.u = r[0];
    rot.x = r[1];
    rot.y = r[2];
    rot.z = r[3];

    // check for NaN
    if (std::isnan(rot.u) || std::isnan(rot.x) || std::isnan(rot.y) || std::isnan(rot.z))
    {
      std::cout << "Your simulation has exploded, rotation of body(" << this->GetName() << ") has NaN(" << rot << ")" << std::endl;
      //rot = this->pose.rot;
      assert(0);
    }
  }
  else
  {
    rot = this->staticPose.rot;
  }

  return rot;
}

////////////////////////////////////////////////////////////////////////////////
// Return the position of the body. in global CS
Vector3 Body::GetPositionRate() const
{
  Vector3 vel;

  if (this->bodyId)
  {
    const dReal *v;

    this->physicsEngine->LockMutex();
    v = dBodyGetLinearVel(this->bodyId);
    this->physicsEngine->UnlockMutex();

    vel.x = v[0];
    vel.y = v[1];
    vel.z = v[2];
  }
  else
  {
    vel.x = 0;
    vel.y = 0;
    vel.z = 0;
  }

  return vel;
}


////////////////////////////////////////////////////////////////////////////////
// Return the rotation
Quatern Body::GetRotationRate() const
{
  Quatern velQ;
  Vector3 vel;

  if (this->bodyId)
  {
    const dReal *v;

    this->physicsEngine->LockMutex();
    v = dBodyGetAngularVel(this->bodyId);
    this->physicsEngine->UnlockMutex();

    vel.x = v[0];
    vel.y = v[1];
    vel.z = v[2];

    velQ.SetFromEuler(vel);
  }
  else
  {
    vel.x = 0;
    vel.y = 0;
    vel.z = 0;
    velQ.SetFromEuler(vel);
  }

  return velQ;
}

////////////////////////////////////////////////////////////////////////////////
// Return the rotation
Vector3 Body::GetEulerRate() const
{
  Vector3 vel;

  if (this->bodyId)
  {
    const dReal *v;

    this->physicsEngine->LockMutex();
    v = dBodyGetAngularVel(this->bodyId);
    this->physicsEngine->UnlockMutex();
    vel.x = v[0];
    vel.y = v[1];
    vel.z = v[2];

  }
  else
  {
    vel.x = 0;
    vel.y = 0;
    vel.z = 0;
  }

  return vel;
}

////////////////////////////////////////////////////////////////////////////////
// Return the ID of this body
dBodyID Body::GetId() const
{
  return this->bodyId;
}


////////////////////////////////////////////////////////////////////////////////
// Set whether this body is enabled
void Body::SetEnabled(bool enable) const
{
  if (!this->bodyId)
    return;

  this->physicsEngine->LockMutex();

  if (enable)
    dBodyEnable(this->bodyId);
  else
    dBodyDisable(this->bodyId);

  this->physicsEngine->UnlockMutex();
}

////////////////////////////////////////////////////////////////////////////////
// Load a new geom helper function
void Body::LoadGeom(XMLConfigNode *node)
{
  Geom *geom;

  if (node->GetName() == "sphere")
    geom = new SphereGeom(this);
  else if (node->GetName() == "cylinder")
    geom = new CylinderGeom(this);
  else if (node->GetName() == "box")
    geom = new BoxGeom(this);
  else if (node->GetName() == "plane")
    geom = new PlaneGeom(this);
  else if (node->GetName() == "trimesh")
    geom = new TrimeshGeom(this);
  else if (node->GetName() == "heightmap")
  {
    this->SetStatic(true);
    geom = new HeightmapGeom(this);
  }
  else if (node->GetName() == "map")
  {
    this->SetStatic(true);
    geom = new MapGeom(this);
  }
  else
  {
    gzthrow("Unknown Geometry Type["+node->GetString("name",std::string(),0)+"]");
  }

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

/////////////////////////////////////////////////////////////////////
// Update the CoM and mass matrix
/*
  What's going on here?  In ODE the CoM of a body corresponds to the
  origin of the body-fixed coordinate system.  In Gazebo, however, we
  want to have arbitrary body coordinate systems (i.e., CoM may be
  displaced from the body-fixed cs).  To get around this limitation in
  ODE, we have an extra fudge-factor (comPose), describing the pose of
  the CoM relative to Gazebo's body-fixed cs.  When using low-level
  ODE functions, one must use apply this factor appropriately.

  The UpdateCoM() function is used to compute this offset, based on
  the mass distribution of attached geoms.  This function also shifts
  the ODE-pose of the geoms, to keep everything in the same place in the
  Gazebo cs.  Simple, neh?

  TODO: messes up if you call it twice; should fix.
*/
void Body::UpdateCoM()
{
  if (!this->bodyId)
    return;

  // user can specify custom mass matrix or alternatively, UpdateCoM will calculate CoM for
  // combined mass of all children geometries.
  if (this->customMassMatrix)
  {
    // Old pose for the CoM
    Pose3d oldPose, newPose, tmpPose;

    // oldPose is the last comPose
    // newPose is mass CoM
    oldPose = this->comPose;

    //std::cout << " in UpdateCoM, name: " << this->GetName() << std::endl;
    //std::cout << " in UpdateCoM, comPose or oldPose: " << this->comPose << std::endl;

    // New pose for the CoM
    newPose.pos.x = this->cx;
    newPose.pos.y = this->cy;
    newPose.pos.z = this->cz;

    std::map< std::string, Geom* >::iterator giter;
    // Fixup the poses of the geoms (they are attached to the CoM)
    for (giter = this->geoms.begin(); giter != this->geoms.end(); giter++)
    {
      if (giter->second->IsPlaceable())
      {
        // FOR GEOMS:
        // get pose with comPose set to oldPose
        this->comPose = oldPose;
        tmpPose = giter->second->GetPose();

        // get pose with comPose set to newPose
        this->comPose = newPose;
        giter->second->SetPose(tmpPose, false);
      }
    }

    // FOR BODY: Fixup the pose of the CoM (ODE body)
    // get pose with comPose set to oldPose
    this->comPose = oldPose;
    tmpPose = this->GetPose();
    // get pose with comPose set to newPose
    this->comPose = newPose;
    this->SetPose(tmpPose);

    // Settle on the new CoM pose
    this->comPose = newPose;



    // comPose is zero in this case, we'll keep cx, cy, cz
    this->comPose.Reset();

    this->comPose.pos.x = this->cx;
    this->comPose.pos.y = this->cy;
    this->comPose.pos.z = this->cz;

    this->physicsEngine->LockMutex();
    // setup this->mass as well
    dMassSetParameters(&this->mass, this->bodyMass,
                       this->cx, this->cy, this->cz,
                       //0,0,0,
                       this->ixx,this->iyy,this->izz,
                       this->ixy,this->ixz,this->iyz);

    dMassTranslate( &this->mass, -this->cx, -this->cy, -this->cz);

    // dMatrix3 rot;
    // dMassRotate(&this->mass, rot);

    // Set the mass matrix
    if (this->mass.mass > 0)
      dBodySetMass( this->bodyId, &this->mass );

    // std::cout << " c[0] " << this->mass.c[0] << std::endl;
    // std::cout << " c[1] " << this->mass.c[1] << std::endl;
    // std::cout << " c[2] " << this->mass.c[2] << std::endl;
    // std::cout << " I[0] " << this->mass.I[0] << std::endl;
    // std::cout << " I[1] " << this->mass.I[1] << std::endl;
    // std::cout << " I[2] " << this->mass.I[2] << std::endl;
    // std::cout << " I[3] " << this->mass.I[3] << std::endl;
    // std::cout << " I[4] " << this->mass.I[4] << std::endl;
    // std::cout << " I[5] " << this->mass.I[5] << std::endl;
    // std::cout << " I[6] " << this->mass.I[6] << std::endl;
    // std::cout << " I[7] " << this->mass.I[7] << std::endl;
    // std::cout << " I[8] " << this->mass.I[8] << std::endl;

    this->physicsEngine->UnlockMutex();
  }
  else
  {

    // original gazebo subroutine that gathers mass from all geoms and sums into one single mass matrix

    const dMass *lmass;
    std::map< std::string, Geom* >::iterator giter;

    this->physicsEngine->LockMutex();
    // Construct the mass matrix by combining all the geoms
    dMassSetZero( &this->mass );

    for (giter = this->geoms.begin(); giter != this->geoms.end(); giter++)
    {
      lmass = giter->second->GetBodyMassMatrix();
      if (giter->second->IsPlaceable() && giter->second->GetGeomId())
      {
        dMassAdd( &this->mass, lmass );
      }
    }

    // Old pose for the CoM
    Pose3d oldPose, newPose, tmpPose;

    // oldPose is the last comPose
    // newPose is mass CoM
    oldPose = this->comPose;

    if (std::isnan(this->mass.c[0]))
      this->mass.c[0] = 0;

    if (std::isnan(this->mass.c[1]))
      this->mass.c[1] = 0;

    if (std::isnan(this->mass.c[2]))
      this->mass.c[2] = 0;

    // New pose for the CoM
    newPose.pos.x = this->mass.c[0];
    newPose.pos.y = this->mass.c[1];
    newPose.pos.z = this->mass.c[2];

    // Fixup the poses of the geoms (they are attached to the CoM)
    for (giter = this->geoms.begin(); giter != this->geoms.end(); giter++)
    {
      if (giter->second->IsPlaceable())
      {
        // FOR GEOMS:
        // get pose with comPose set to oldPose
        this->comPose = oldPose;
        tmpPose = giter->second->GetPose();

        // get pose with comPose set to newPose
        this->comPose = newPose;
        giter->second->SetPose(tmpPose, false);
      }
    }

    // FOR BODY: Fixup the pose of the CoM (ODE body)
    // get pose with comPose set to oldPose
    this->comPose = oldPose;
    tmpPose = this->GetPose();
    // get pose with comPose set to newPose
    this->comPose = newPose;
    this->SetPose(tmpPose);


    // Settle on the new CoM pose
    this->comPose = newPose;

    // My Cheap Hack, to put the center of mass at the origin
    this->mass.c[0] = this->mass.c[1] = this->mass.c[2] = 0;

    // Set the mass matrix
    if (this->mass.mass > 0)
      dBodySetMass( this->bodyId, &this->mass );

    this->physicsEngine->UnlockMutex();
  }

}

////////////////////////////////////////////////////////////////////////////////
/// Get the Center of Mass pose
const Pose3d &Body::GetCoMPose() const
{
  return this->comPose;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the velocity of the body
void Body::SetLinearVel(const Vector3 &vel)
{
  if (this->bodyId)
  {
    this->physicsEngine->LockMutex();
    dBodySetLinearVel(this->bodyId, vel.x, vel.y, vel.z);
    this->physicsEngine->UnlockMutex();
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Get the velocity of the body
Vector3 Body::GetLinearVel() const
{
  Vector3 vel;

  if (this->bodyId)
  {
    const dReal *dvel;

    this->physicsEngine->LockMutex();
    dvel = dBodyGetLinearVel(this->bodyId);
    this->physicsEngine->UnlockMutex();

    vel.x = dvel[0];
    vel.y = dvel[1];
    vel.z = dvel[2];
  }

  return vel;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the velocity of the body
void Body::SetAngularVel(const Vector3 &vel)
{
  if (this->bodyId)
  {
    this->physicsEngine->LockMutex();
    dBodySetAngularVel(this->bodyId, vel.x, vel.y, vel.z);
    this->physicsEngine->UnlockMutex();
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Get the velocity of the body
Vector3 Body::GetAngularVel() const
{
  Vector3 vel;

  if (this->bodyId)
  {
    const dReal *dvel;

    this->physicsEngine->LockMutex();
    dvel = dBodyGetAngularVel(this->bodyId);
    this->physicsEngine->UnlockMutex();

    vel.x = dvel[0];
    vel.y = dvel[1];
    vel.z = dvel[2];
  }

  return vel;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the linear acceleration of the body
void Body::SetLinearAccel(const Vector3 &accel)
{
  this->SetForce( accel * this->GetMass());
}

////////////////////////////////////////////////////////////////////////////////
/// Get the angular acceleration of the body
Vector3 Body::GetLinearAccel() const
{
  return this->GetForce() /  this->GetMass();
}

////////////////////////////////////////////////////////////////////////////////
/// Set the angular acceleration of the body
void Body::SetAngularAccel(const Vector3 &accel)
{
  this->SetTorque( accel * this->GetMass());
}

////////////////////////////////////////////////////////////////////////////////
/// Get the angular acceleration of the body
Vector3 Body::GetAngularAccel() const
{
  return this->GetTorque() /  this->GetMass();
}

////////////////////////////////////////////////////////////////////////////////
/// \brief Set the force applied to the body
void Body::SetForce(const Vector3 &force)
{
  if (this->bodyId)
  {
    this->physicsEngine->LockMutex();
    dBodyAddForce(this->bodyId, force.x, force.y, force.z);
    this->physicsEngine->UnlockMutex();
  }
}

////////////////////////////////////////////////////////////////////////////////
/// \brief Get the force applied to the body
Vector3 Body::GetForce() const
{
  Vector3 force;

  if (this->bodyId)
  {
    const dReal *dforce;

    this->physicsEngine->LockMutex();
    dforce = dBodyGetForce(this->bodyId);
    this->physicsEngine->UnlockMutex();

    force.x = dforce[0];
    force.y = dforce[1];
    force.z = dforce[2];
  }

  return force;
}

////////////////////////////////////////////////////////////////////////////////
/// \brief Set the torque applied to the body
void Body::SetTorque(const Vector3 &torque)
{
  if (this->bodyId)
  {
    this->physicsEngine->LockMutex();
    dBodySetTorque(this->bodyId, torque.x, torque.y, torque.z);
    this->physicsEngine->UnlockMutex();
  }
}

////////////////////////////////////////////////////////////////////////////////
/// \brief Get the torque applied to the body
Vector3 Body::GetTorque() const
{
  Vector3 torque;

  if (this->bodyId)
  {
    const dReal *dtorque;

    this->physicsEngine->LockMutex();
    dtorque = dBodyGetTorque(this->bodyId);
    this->physicsEngine->UnlockMutex();

    torque.x = dtorque[0];
    torque.y = dtorque[1];
    torque.z = dtorque[2];
  }

  return torque;
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
