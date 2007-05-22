#include <Ogre.h>

#include "OgreAdaptor.hh"
#include "SphereGeom.hh"
#include "BoxGeom.hh"
#include "CylinderGeom.hh"
#include "PlaneGeom.hh"
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
  this->geoms.clear();
}

////////////////////////////////////////////////////////////////////////////////
// Load the body based on an XMLConfig node
int Body::Load(XMLConfigNode *node)
{
  XMLConfigNode *geomNode = node->GetChildByNSPrefix("geom");

  this->SetName(node->GetString("name","",1));
  this->SetPosition(node->GetVector3("xyz",Vector3(0,0,0)));
  this->SetRotation(node->GetRotation("rpy",Quatern(1,0,0,0)));

  // Load the geometries
  while (geomNode)
  {
    // Create and Load a geom, which will belong to this body.
    this->LoadGeom(geomNode);
    geomNode = geomNode->GetNext();
  }

}

////////////////////////////////////////////////////////////////////////////////
// Initialize the body
int Body::Init()
{
  return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Update the body
int Body::Update()
{
  if (!this->IsStatic())
    this->SetPose(this->GetPose());

  return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Attach a geom to this body
void Body::AttachGeom( Geom *geom )
{
  if (this->bodyId)
  {
    dGeomSetBody(geom->GetGeomId(), this->bodyId);
  }

  this->geoms.push_back(geom);
}

////////////////////////////////////////////////////////////////////////////////
// Set the pose of the body
void Body::SetPose(const Pose3d &pose)
{
  this->SetPosition(pose.pos);
  this->SetRotation(pose.rot);
}

////////////////////////////////////////////////////////////////////////////////
// Return the pose of the body
Pose3d Body::GetPose() const
{
  Pose3d pose;

  pose.pos = this->GetPosition();
  pose.rot = this->GetRotation();

  return pose;
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
    std::vector< Geom* >::iterator iter;
    for (iter=this->geoms.begin(); iter!=this->geoms.end(); iter++)
    {
      (*iter)->SetPosition(pos);
    }
  }

  // Set the position of the scene node
  this->sceneNode->setPosition(pos.x, pos.y, pos.z);
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
    std::vector< Geom* >::iterator iter;
    for (iter=this->geoms.begin(); iter!=this->geoms.end(); iter++)
    {
      //(*iter)->SetRotation(rot);
    }
  }

  // Set the orientation of the scene node
  this->sceneNode->setOrientation(rot.u, rot.x, rot.y, rot.z);
}

////////////////////////////////////////////////////////////////////////////////
// Return the position of the body
Vector3 Body::GetPosition() const
{
  Vector3 pos;
  if (!this->IsStatic())
  {
    const dReal *p;

    p = dBodyGetPosition(this->bodyId);

    pos.x = p[0];
    pos.y = p[1];
    pos.z = p[2];
  }

  return pos;
}


////////////////////////////////////////////////////////////////////////////////
// Return the rotation
Quatern Body::GetRotation() const
{
  Quatern rot;
  if (!this->IsStatic())
  {
    const dReal *r;

    r = dBodyGetQuaternion(this->bodyId);

    rot.u = r[0];
    rot.x = r[1];
    rot.y = r[2];
    rot.z = r[3];
  }

  return rot;
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

  // The mesh used for visualization
  std::string mesh = node->GetString("mesh","",0);

  if (node->GetName() == "sphere")
  {
    double radius = node->GetDouble("size",0.0,0);
    geom = new SphereGeom(this, radius, mesh);
  }
  else if (node->GetName() == "cylinder")
  {
    double radius = node->GetTupleDouble("size",0,1.0);
    double length = node->GetTupleDouble("size",1,1.0);
    geom = new CylinderGeom(this, radius, length, mesh);
  }
  else if (node->GetName() == "box")
  {
    Vector3 size = node->GetVector3("size",Vector3(1,1,1));
    geom = new BoxGeom(this, size, mesh);
  }
  else if (node->GetName() == "plane")
  {
    Vector3 normal = node->GetVector3("normal",Vector3(0,1,0));
    double altitude = node->GetDouble("altitude",0,0);
    geom = new PlaneGeom(this,altitude,normal);
  }
  else
  {
    std::cerr << "Unkown Geometry Type[" << node->GetName() << "]\n";
    return -1;
  }

  geom->SetName(node->GetString("name","",1));
  geom->SetMeshMaterial(node->GetString("material","",0));

  this->AttachGeom(geom);

  return 0;
}
