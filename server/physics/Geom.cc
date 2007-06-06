#include <Ogre.h>
#include <sstream>

#include "ContactParams.hh"
#include "OgreAdaptor.hh"
#include "Body.hh"
#include "Geom.hh"

using namespace gazebo;

int Geom::geomIdCounter = 0;

////////////////////////////////////////////////////////////////////////////////
// Constructor
Geom::Geom( Body *body)
  : Entity(body)
{
  this->body = body;

  // Create the contact parameters
  this->contact = new ContactParams();

  this->geomId = NULL;

  this->ogreObj = NULL;

  // Zero out the mass
  dMassSetZero(&this->mass);
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
Geom::~Geom()
{
  dGeomDestroy(this->geomId);
}

////////////////////////////////////////////////////////////////////////////////
// Set the encapsulated geometry object
void Geom::SetGeom(dGeomID geomId, bool placeable)
{
  assert(!this->geomId);

  this->placeable = placeable;

  this->geomId = geomId;

  if (this->geomId)
  {
    assert(dGeomGetSpace(this->geomId) != 0);
    dGeomSetData(this->geomId, this);
  }

  // Create a new name of the geom's mesh entity
  std::ostringstream stream;
  stream << "Entity[" << (int)this->geomId << "]";
  this->SetName(stream.str());
}

////////////////////////////////////////////////////////////////////////////////
// Return the geom id
dGeomID Geom::GetGeomId() const
{
  return this->geomId;
}

////////////////////////////////////////////////////////////////////////////////
// Return whether this is a placeable geom.
bool Geom::IsPlaceable() const
{
  return this->placeable;
}

////////////////////////////////////////////////////////////////////////////////
// Set the pose
void Geom::SetPose(const Pose3d &pose)
{
  this->SetPosition(pose.pos);
  this->SetRotation(pose.rot);
}

////////////////////////////////////////////////////////////////////////////////
// Return the pose of the geom
Pose3d Geom::GetPose() const
{
  const dReal *p;
  dQuaternion r;
  Pose3d pose;

  p = dGeomGetPosition(this->geomId);
  dGeomGetQuaternion(this->geomId, r);
  pose.pos.x = p[0];
  pose.pos.y = p[1];
  pose.pos.z = p[2];
  
  pose.rot.u = r[0];
  pose.rot.x = r[1];
  pose.rot.y = r[2];
  pose.rot.z = r[3];

  return pose;
}

////////////////////////////////////////////////////////////////////////////////
// Set the position
void Geom::SetPosition(const Vector3 &pos)
{
  if (this->geomId && this->placeable)
  {
    dGeomSetPosition(this->geomId, pos.x, pos.y, pos.z);
  }
}

////////////////////////////////////////////////////////////////////////////////
// Set the rotation
void Geom::SetRotation(const Quatern &rot)
{
  if (this->geomId && this->placeable)
  {
    dQuaternion q;
    q[0] = rot.u;
    q[1] = rot.x;
    q[2] = rot.y;
    q[3] = rot.z;

    // Set the rotation of the ODE body
    dGeomSetQuaternion(this->geomId, q);
  }
}


////////////////////////////////////////////////////////////////////////////////
/// Attach a mesh to the geom
void Geom::AttachMesh(const std::string &meshName)
{
  this->ogreObj = (Ogre::MovableObject*)(this->sceneNode->getCreator()->createEntity(this->GetName().c_str(), meshName));

  this->sceneNode->attachObject((Ogre::Entity*)this->ogreObj);
}

////////////////////////////////////////////////////////////////////////////////
/// Attach a moveable object to the node
void Geom::AttachObject( Ogre::MovableObject *obj )
{
  this->ogreObj = obj;
  this->sceneNode->attachObject(this->ogreObj);
}

////////////////////////////////////////////////////////////////////////////////
/// Set the scale of the mesh
void Geom::ScaleMesh(const Vector3 &scale)
{
  //this->body->GetSceneNode()->setScale(scale.x, scale.y, scale.z);
  this->sceneNode->setScale(scale.x, scale.y, scale.z);
}

////////////////////////////////////////////////////////////////////////////////
/// Set the mesh pose
void Geom::SetMeshPose(const Pose3d &pose)
{
  this->sceneNode->setPosition(pose.pos.x, pose.pos.y, pose.pos.z);
  this->sceneNode->setOrientation(pose.rot.u, pose.rot.x, pose.rot.y, pose.rot.z);
}

////////////////////////////////////////////////////////////////////////////////
/// Set the mesh position
void Geom::SetMeshPosition(const Vector3 &pos)
{
  this->sceneNode->setPosition(pos.x, pos.y, pos.z);
}

////////////////////////////////////////////////////////////////////////////////
/// Set whether the mesh casts shadows
void Geom::SetCastShadows(bool enable)
{
  if (this->ogreObj)
    this->ogreObj->setCastShadows(enable);
}

////////////////////////////////////////////////////////////////////////////////
/// Set the material to apply to the mesh
void Geom::SetMeshMaterial(const std::string &materialName)
{
  Ogre::Entity *ent = dynamic_cast<Ogre::Entity*>(this->ogreObj);

  if (materialName != "" && ent)
    ent->setMaterialName(materialName);
}

