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
  this->transId = NULL;

  this->ogreObj = NULL;

  // Zero out the mass
  dMassSetZero(&this->mass);
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
Geom::~Geom()
{
  if (this->geomId)
    dGeomDestroy(this->geomId);

  if (this->transId)
    dGeomDestroy(this->transId);
}

////////////////////////////////////////////////////////////////////////////////
// Set the encapsulated geometry object
void Geom::SetGeom(dGeomID geomId, bool placeable)
{
  assert(!this->geomId);

  this->placeable = placeable;

  this->geomId = geomId;
  this->transId = NULL;

  if (this->placeable)
  {
    this->transId = dCreateGeomTransform( this->spaceId );
    dGeomTransformSetGeom( this->transId, this->geomId );
    dGeomTransformSetInfo( this->transId, 1 );

    assert(dGeomGetSpace(this->geomId) == 0);
  }
  else
    assert(dGeomGetSpace(this->geomId) != 0);

  //TODO: Mass...

  dGeomSetData(this->geomId, this);

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
// Return the transform id
dGeomID Geom::GetTransId() const
{
  return this->transId;
}

////////////////////////////////////////////////////////////////////////////////
// Return whether this is a placeable geom.
bool Geom::IsPlaceable() const
{
  return this->placeable;
}

////////////////////////////////////////////////////////////////////////////////
// Set the pose relative to the body
void Geom::SetPose(const Pose3d &pose)
{
  if (this->placeable)
  {
    dQuaternion q;
    q[0] = pose.rot.u;
    q[1] = pose.rot.x;
    q[2] = pose.rot.y;
    q[3] = pose.rot.z;

    dGeomSetPosition(this->geomId, pose.pos.x, pose.pos.y, pose.pos.z);
    dMassTranslate(&this->mass, pose.pos.x, pose.pos.y, pose.pos.z);

    dGeomSetQuaternion(this->geomId, q); 
    dMassRotate(&this->mass, dGeomGetRotation(this->geomId));

    this->sceneNode->setPosition(pose.pos.x, pose.pos.y, pose.pos.z);
    this->sceneNode->setOrientation(pose.rot.u, pose.rot.x, pose.rot.y, pose.rot.z);
  }
}

////////////////////////////////////////////////////////////////////////////////
// Return the pose of the geom relative to the body
Pose3d Geom::GetPose() const
{
  Pose3d pose;

  if (this->geomId && this->placeable)
  {
    const dReal *p;
    dQuaternion r;

    p = dGeomGetPosition(this->geomId);
    dGeomGetQuaternion(this->geomId, r);
    pose.pos.x = p[0];
    pose.pos.y = p[1];
    pose.pos.z = p[2];

    pose.rot.u = r[0];
    pose.rot.x = r[1];
    pose.rot.y = r[2];
    pose.rot.z = r[3];
  }
  else
    pose = this->body->GetPose();

  return pose;
}

////////////////////////////////////////////////////////////////////////////////
// Set the position
void Geom::SetPosition(const Vector3 &pos)
{
  Pose3d pose;

  pose = this->GetPose();
  pose.pos = pos;
  this->SetPose(pose);
}

////////////////////////////////////////////////////////////////////////////////
// Set the rotation
void Geom::SetRotation(const Quatern &rot)
{
  Pose3d pose;

  pose = this->GetPose();
  pose.rot = rot;
  this->SetPose(pose);
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

////////////////////////////////////////////////////////////////////////////////
/// Set the category bits, used during collision detection
void Geom::SetCategoryBits(unsigned int bits)
{
  dGeomSetCategoryBits(this->geomId, bits);
}

////////////////////////////////////////////////////////////////////////////////
/// Set the collide bits, used during collision detection
void Geom::SetCollideBits(unsigned int bits)
{
  dGeomSetCollideBits(this->geomId, bits);
}
