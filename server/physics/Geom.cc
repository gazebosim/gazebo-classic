#include <Ogre.h>
#include <sstream>

#include "Global.hh"
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
  this->spaceId = this->body->spaceId;

  // Create the contact parameters
  this->contact = new ContactParams();
  this->geomId = NULL;
  this->transId = NULL;

  this->ogreObj = NULL;

  // Most geoms don't need extra rotation. Cylinders do.
  this->extraRotation.SetToIdentity();

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
  //assert(!this->geomId);

  this->placeable = placeable;

  this->geomId = geomId;
  this->transId = NULL;

  if (this->placeable)
  {
    if (dGeomGetClass(geomId) != dTriMeshClass)
    {
      this->transId = dCreateGeomTransform( this->spaceId );
      dGeomTransformSetGeom( this->transId, this->geomId );
      dGeomTransformSetInfo( this->transId, 1 );
      assert(dGeomGetSpace(this->geomId) == 0);
    }
  }
  else
    assert(dGeomGetSpace(this->geomId) != 0);

  dGeomSetData(this->geomId, this);

  if (this->IsStatic())
  {
    this->SetCategoryBits(GZ_FIXED_COLLIDE);
    this->SetCollideBits(~GZ_FIXED_COLLIDE);
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
// Return the transform id
dGeomID Geom::GetTransId() const
{
  return this->transId;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the ODE geom class
int Geom::GetGeomClass() const
{
  return dGeomGetClass(this->geomId);
}

////////////////////////////////////////////////////////////////////////////////
// Return whether this is a placeable geom.
bool Geom::IsPlaceable() const
{
  return this->placeable;
}

////////////////////////////////////////////////////////////////////////////////
// Set the pose relative to the body
void Geom::SetPose(const Pose3d &pose, bool updateCoM)
{
  if (this->placeable)
  {
    Pose3d localPose;
    dQuaternion q;

    // Transform into CoM relative Pose
    localPose = pose - this->body->GetCoMPose();

    OgreAdaptor::Instance()->SetSceneNodePose(this->sceneNode, localPose);

    localPose.rot = localPose.rot * this->extraRotation;

    q[0] = localPose.rot.u;
    q[1] = localPose.rot.x;
    q[2] = localPose.rot.y;
    q[3] = localPose.rot.z;

    // Set the pose of the encapsulated geom; this is always relative
    // to the CoM
    dGeomSetPosition(this->geomId, localPose.pos.x, localPose.pos.y, localPose.pos.z);
    dGeomSetQuaternion(this->geomId, q); 

    //dMassTranslate(&this->mass, pose.pos.x, pose.pos.y, pose.pos.z);
    //dMassRotate(&this->mass, dGeomGetRotation(this->geomId));


    if (updateCoM)
      this->body->UpdateCoM();

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

    // Get the pose of the encapsulated geom; this is always relative to
    // the CoM
    p = dGeomGetPosition(this->geomId);
    dGeomGetQuaternion(this->geomId, r);

    pose.pos.x = p[0];
    pose.pos.y = p[1];
    pose.pos.z = p[2];

    pose.rot.u = r[0];
    pose.rot.x = r[1];
    pose.rot.y = r[2];
    pose.rot.z = r[3];

    pose.rot = pose.rot * this->extraRotation.GetInverse();

    // Transform into body relative pose
    pose += this->body->GetCoMPose();

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
  // The ordering of the vector is converted to OGRE's coordinate system
  this->sceneNode->setScale(scale.y, scale.z, scale.x);
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
  Ogre::Entity *ent = NULL;
  Ogre::SimpleRenderable *simple = NULL;

  if (materialName.empty())
    return;

  ent = dynamic_cast<Ogre::Entity*>(this->ogreObj);
  simple = dynamic_cast<Ogre::SimpleRenderable*>(this->ogreObj);

  if (ent)
    ent->setMaterialName(materialName);
  //  ent->setMaterialName("cam1_sensor");
  else if (simple)
    //simple->setMaterial("cam1_sensor");
    simple->setMaterial(materialName);
}

////////////////////////////////////////////////////////////////////////////////
/// Set the category bits, used during collision detection
void Geom::SetCategoryBits(unsigned int bits)
{
  dGeomSetCategoryBits(this->geomId, bits);
  dGeomSetCategoryBits((dGeomID)this->spaceId, bits);
}

////////////////////////////////////////////////////////////////////////////////
/// Set the collide bits, used during collision detection
void Geom::SetCollideBits(unsigned int bits)
{
  dGeomSetCollideBits(this->geomId, bits);
  dGeomSetCollideBits((dGeomID)this->spaceId, bits);
}

////////////////////////////////////////////////////////////////////////////////
/// Get the mass of the geom
const dMass *Geom::GetBodyMassMatrix()
{
  Pose3d pose;
  dQuaternion q;
  dMatrix3 r;
  dMass bodyMass;

  if (!this->placeable)
    return NULL;

  pose = this->GetPose();

  q[0] = pose.rot.u;
  q[1] = pose.rot.x;
  q[2] = pose.rot.y;
  q[3] = pose.rot.z;

  dQtoR(q,r);

  this->bodyMass = this->mass;
  dMassRotate(&this->bodyMass, r);
  dMassTranslate( &this->bodyMass, pose.pos.x, pose.pos.y, pose.pos.z);

  return &this->bodyMass;
}

