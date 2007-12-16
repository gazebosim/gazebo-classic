#include <Ogre.h>
#include <sstream>

#include "Vector2.hh"
#include "Quatern.hh"
#include "GazeboError.hh"
#include "SensorFactory.hh"
#include "Sensor.hh"
#include "OgreAdaptor.hh"
#include "SphereGeom.hh"
#include "TrimeshGeom.hh"
#include "BoxGeom.hh"
#include "CylinderGeom.hh"
#include "PlaneGeom.hh"
#include "HeightmapGeom.hh"
#include "Geom.hh"
#include "Body.hh"

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
// Constructor
Body::Body(Entity *parent, dWorldID worldId)
  : Entity(parent)
{
  if (!this->IsStatic())
  {
    this->bodyId = dBodyCreate(worldId);

    dMassSetZero( &this->mass );
  }
  else
  {
    this->bodyId = NULL;
  }
}


////////////////////////////////////////////////////////////////////////////////
// Destructor
Body::~Body()
{
  std::vector< Geom* >::iterator giter;
  std::vector< Sensor* >::iterator siter;

  for (giter = this->geoms.begin(); giter != this->geoms.end(); giter++)
  {
    delete (*giter);
  }
  this->geoms.clear();

  for (siter = this->sensors.begin(); siter != this->sensors.end(); siter++)
  {
    delete (*siter);
  }
  this->sensors.clear();
}

////////////////////////////////////////////////////////////////////////////////
// Load the body based on an XMLConfig node
int Body::Load(XMLConfigNode *node)
{
  XMLConfigNode *childNode;

  this->SetName(node->GetString("name","",1));
  this->SetPosition(node->GetVector3("xyz",Vector3(0,0,0)));
  this->SetRotation(node->GetRotation("rpy",Quatern(1,0,0,0)));

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

  childNode = node->GetChild("visual");

  while (childNode)
  {
    this->LoadVisual(childNode);
    childNode = childNode->GetNext("visual");
  }

  // If no geoms are attached, then don't let gravity affect the body.
  if (this->geoms.size()==0)
  {
    this->SetGravityMode(false);
  }

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
    dBodySetGravityMode(this->bodyId, mode);
}

////////////////////////////////////////////////////////////////////////////////
// Initialize the body
int Body::Init()
{
  // Set the intial pose. Must do this to handle static models
  this->SetPose(this->GetPose());

  std::vector< Sensor* >::iterator siter;

  for (siter = this->sensors.begin(); siter != this->sensors.end(); siter++)
  {
    (*siter)->Init();
  }

  return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Update the body
int Body::Update(UpdateParams &params)
{
  std::vector< Sensor* >::iterator sensorIter;
  std::vector< Geom* >::iterator geomIter;

  if (!this->IsStatic())
  {
    Pose3d pose = this->GetPose();

    // Set the pose of the scene node
    OgreAdaptor::Instance()->SetSceneNodePose(this->sceneNode, pose);
  }

  for (geomIter=this->geoms.begin(); 
       geomIter!=this->geoms.end(); geomIter++)
  {
    (*geomIter)->Update();
  }

  for (sensorIter=this->sensors.begin(); 
       sensorIter!=this->sensors.end(); sensorIter++)
  {
    (*sensorIter)->Update(params);
  }

  return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Attach a geom to this body
void Body::AttachGeom( Geom *geom )
{
  if (this->bodyId)
  {
    if (geom->IsPlaceable())
    {
      if (geom->GetTransId())
        dGeomSetBody(geom->GetTransId(), this->bodyId);
      else if (geom->GetGeomId())
        dGeomSetBody(geom->GetGeomId(), this->bodyId);
    }
  }

  this->geoms.push_back(geom);
}

////////////////////////////////////////////////////////////////////////////////
// Set the pose of the body
void Body::SetPose(const Pose3d &pose)
{
  if (this->IsStatic())
  {
    std::vector< Geom* >::iterator giter;
    PlaneGeom *plane = NULL;
    this->staticPose = pose;

    this->SetPosition(this->staticPose.pos);
    this->SetRotation(this->staticPose.rot);

    // Hack to fix the altitude of the ODE plane
    for (giter = this->geoms.begin(); giter != this->geoms.end(); giter++)
    {
      if ((*giter)->GetGeomClass() == dPlaneClass)
      {
        plane = dynamic_cast<PlaneGeom*>(*giter);
        plane->SetAltitude(pose.pos.z);
      }
      else
        (*giter)->SetPose(this->staticPose);
    }
  }
  else
  {
    Pose3d localPose;

    // Compute pose of CoM
    localPose =this->comPose + pose;

    this->SetPosition(localPose.pos);
    this->SetRotation(localPose.rot);
  }
}

////////////////////////////////////////////////////////////////////////////////
// Return the pose of the body
Pose3d Body::GetPose() const
{
  if (this->IsStatic())
    return this->staticPose;
  else
  {
    Pose3d pose;

    pose.pos = this->GetPosition();
    pose.rot = this->GetRotation();

    pose = this->comPose.CoordPoseSolve(pose);

    return pose;
  }
}

////////////////////////////////////////////////////////////////////////////////
// Set the position of the body
void Body::SetPosition(const Vector3 &pos)
{

  if (!this->IsStatic())
  {
    // Set the position of the ODE body
    dBodySetPosition(this->bodyId, pos.x, pos.y, pos.z);
  }
  else
  {
    this->staticPose.pos = pos;
  }
 
  // Set the position of the scene node
  OgreAdaptor::Instance()->SetSceneNodePosition(this->sceneNode, pos);
}

////////////////////////////////////////////////////////////////////////////////
// Set the rotation of the body
void Body::SetRotation(const Quatern &rot)
{
  if (!this->IsStatic())
  {
    dQuaternion q;
    q[0] = rot.u;
    q[1] = rot.x;
    q[2] = rot.y;
    q[3] = rot.z;

    // Set the rotation of the ODE body
    dBodySetQuaternion(this->bodyId, q);

  }
  else
  {
    this->staticPose.rot = rot;
  }

  // Set the orientation of the scene node
  OgreAdaptor::Instance()->SetSceneNodeRotation(this->sceneNode, rot);
}

////////////////////////////////////////////////////////////////////////////////
// Return the position of the body. in global CS
Vector3 Body::GetPosition() const
{

  if (!this->IsStatic())
  {
    Vector3 pos;
    const dReal *p;

    p = dBodyGetPosition(this->bodyId);

    pos.x = p[0];
    pos.y = p[1];
    pos.z = p[2];

    return pos;
  }
  else
    return this->staticPose.pos;
}


////////////////////////////////////////////////////////////////////////////////
// Return the rotation
Quatern Body::GetRotation() const
{
  if (!this->IsStatic())
  {
    Quatern rot;
    const dReal *r;

    r = dBodyGetQuaternion(this->bodyId);

    rot.u = r[0];
    rot.x = r[1];
    rot.y = r[2];
    rot.z = r[3];

    return rot;
  }
  else
    return this->staticPose.rot;

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
  if (enable)
    dBodyEnable(this->bodyId);
  else
    dBodyDisable(this->bodyId);
}

////////////////////////////////////////////////////////////////////////////////
// Load a new geom helper function
int Body::LoadGeom(XMLConfigNode *node)
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
  else
  {
    std::cerr << "Unkown Geometry Type[" << node->GetName() << "]\n";
    return -1;
  }

  geom->Load(node);

  return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Load a sensor
void Body::LoadSensor(XMLConfigNode *node)
{
  Sensor *sensor = NULL;

  if (node==NULL)
  {
    std::ostringstream stream;
    stream << "Null node pointer. Invalid sensor in the world file.";
    gzthrow(stream.str());
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
    stream << "Null sensor. Invalid sensor name[" << node->GetName() << "]";
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
  int i;
  const dMass *lmass;
  Pose3d oldPose, newPose, pose;
  std::vector< Geom* >::iterator giter;

  // Dummy bodies dont have mass
  if (!this->bodyId)
    return;

  // Construct the mass matrix by combining all the geoms
  dMassSetZero( &this->mass );

  for (giter = this->geoms.begin(); giter != this->geoms.end(); giter++)
  {
    lmass = (*giter)->GetBodyMassMatrix();
    if ((*giter)->IsPlaceable() && (*giter)->GetGeomId())
    {
      dMassAdd( &this->mass, lmass );
    }
  }

  // Old pose for the CoM
  oldPose = this->comPose;

  if (isnan(this->mass.c[0]))
    this->mass.c[0] = 0;

  if (isnan(this->mass.c[1]))
    this->mass.c[1] = 0;

  if (isnan(this->mass.c[2]))
    this->mass.c[2] = 0;

  // New pose for the CoM
  newPose.pos.x = this->mass.c[0];
  newPose.pos.y = this->mass.c[1];
  newPose.pos.z = this->mass.c[2];

  // Fixup the poses of the geoms (they are attached to the CoM)
  for (giter = this->geoms.begin(); giter != this->geoms.end(); giter++)
  {
    if ((*giter)->IsPlaceable())
    {
      this->comPose = oldPose;
      pose = (*giter)->GetPose();
      this->comPose = newPose;
      (*giter)->SetPose(pose, false);
    }
  }

  // Fixup the pose of the CoM (ODE body)
  this->comPose = oldPose;
  pose = this->GetPose();
  this->comPose = newPose;
  this->SetPose(pose);

  // Settle on the new CoM pose
  this->comPose = newPose;

  // My Cheap Hack, to put the center of mass at the origin
  this->mass.c[0] = this->mass.c[1] = this->mass.c[2] = 0;
 
  // Set the mass matrix
  if (this->mass.mass > 0)
    dBodySetMass( this->bodyId, &this->mass );
}

////////////////////////////////////////////////////////////////////////////////
/// Get the Center of Mass pose
const Pose3d &Body::GetCoMPose() const
{
  return this->comPose;
}

////////////////////////////////////////////////////////////////////////////////
/// Load a renderable
void Body::LoadVisual(XMLConfigNode *node)
{
  std::ostringstream stream;
  Ogre::SceneNode *snode = NULL;
  Ogre::Entity *entity;
  Pose3d pose;

  std::string name = node->GetString("name","",1);
  std::string meshName = node->GetString("mesh","",1);
  std::string materialName = node->GetString("material","",0);

  pose.pos = node->GetVector3("xyz", Vector3(0,0,0));
  pose.rot = node->GetRotation("rpy", Quatern());

  snode = this->sceneNode->createChildSceneNode( name );

  stream << "VISUAL_" << name;
  entity = snode->getCreator()->createEntity(stream.str(), meshName);

  snode->attachObject(entity);

  OgreAdaptor::Instance()->SetSceneNodePose(snode,pose);

  this->visuals.push_back( snode );
}
