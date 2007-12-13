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
/* Desc: Geom class
 * Author: Nate Koenig
 * Date: 13 Feb 2006
 * SVN: $Id$
 */

#include <Ogre.h>
#include <sstream>

#include "Global.hh"
#include "GazeboMessage.hh"
#include "ContactParams.hh"
#include "OgreAdaptor.hh"
#include "Body.hh"
#include "Geom.hh"

using namespace gazebo;

int Geom::geomIdCounter = 0;

////////////////////////////////////////////////////////////////////////////////
// Constructor
Geom::Geom( Body *body)//, const std::string &name)
  : Entity(body)
{
  //this->SetName(name);
  this->body = body;
  this->spaceId = this->body->spaceId;

  // Create the contact parameters
  this->contact = new ContactParams();
  this->geomId = NULL;
  this->transId = NULL;

  this->ogreObj = NULL;
  this->odeObj = NULL;

  this->laserFiducialId = -1;
  this->laserRetro = 0.0;

  // Most geoms don't need extra rotation. Cylinders do.
  this->extraRotation.SetToIdentity();

  this->boundingBoxNode = NULL;

  // Zero out the mass
  dMassSetZero(&this->mass);
  dMassSetZero(&this->bodyMass);

  this->sceneBlendType = Ogre::SBT_TRANSPARENT_ALPHA;

  this->transparency = 0;
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
/// Load the geom
void Geom::Load(XMLConfigNode *node)
{
  this->SetName(node->GetString("name","",1));

  // The mesh used for visualization
  this->meshName = node->GetString("mesh","",0);
  this->dblMass = node->GetDouble("mass",1.0,1e-5);

  if (this->dblMass <= 0)
  {
    this->dblMass = 1e-5;
  }

  this->LoadChild(node);

  this->body->AttachGeom(this);

  if (node->GetChild("meshScale"))
    this->ScaleMesh(node->GetVector3("meshScale",Vector3(1,1,1)));

  this->SetPosition(node->GetVector3("xyz",Vector3(0,0,0)));
  this->SetRotation(node->GetRotation("rpy",Quatern()));
  this->SetMeshMaterial(node->GetString("material","",0));
  this->SetLaserFiducialId(node->GetInt("laserFiducialId",-1,0));
  this->SetLaserRetro(node->GetDouble("laserRetro",0.0,0));

  // Create the bounding box
  if (dGeomGetClass(this->geomId) != dPlaneClass) 
  {
    dReal aabb[6];
    dGeomGetAABB(this->geomId, aabb);

    Vector3 min(aabb[0], aabb[2], aabb[4]);
    Vector3 max(aabb[1], aabb[3], aabb[5]);

    if (this->GetName() == "pan_geom")
    {
      std::cout << "MIN[" << min << "] MAX[" << max<< "]\n";
    }


    this->boundingBoxNode = this->sceneNode->createChildSceneNode(this->GetName()+"_AABB_NODE"); 
    this->boundingBoxNode->setInheritScale(false);

    this->odeObj = (Ogre::MovableObject*)(this->sceneNode->getCreator()->createEntity(this->GetName()+"_AABB", "unit_box"));

    this->boundingBoxNode->attachObject(this->odeObj);
    Vector3 diff = max-min;

    this->boundingBoxNode->setScale(diff.y, diff.z, diff.x);

    Ogre::Entity *ent = NULL;
    Ogre::SimpleRenderable *simple = NULL;

    ent = dynamic_cast<Ogre::Entity*>(this->odeObj);
    simple = dynamic_cast<Ogre::SimpleRenderable*>(this->odeObj);

    if (ent)
      ent->setMaterialName("Gazebo/TransparentTest");
    else if (simple)
      simple->setMaterial("Gazebo/TransparentTest");

    this->boundingBoxNode->setVisible(false);
  }
}
 
////////////////////////////////////////////////////////////////////////////////
// Set the encapsulated geometry object
void Geom::SetGeom(dGeomID geomId, bool placeable)
{
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
  //std::ostringstream stream;
  //stream << "Entity[" << (int)this->geomId << "]";
  //this->SetName(stream.str());
}

void Geom::Update()
{
  if (this->boundingBoxNode)
    this->boundingBoxNode->setVisible(Global::GetShowBoundingBoxes());

  if (Global::GetShowJoints())
    if (dGeomGetClass(this->geomId) != dPlaneClass) 
      this->SetTransparency(0.8);
  else
    this->SetTransparency(0);

  this->UpdateChild();
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
  if (this->geomId)
    return dGeomGetClass(this->geomId);
  else
    return 0;
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

  if (this->placeable && this->geomId)
  {
    Pose3d localPose;
    dQuaternion q;

    // Transform into CoM relative Pose
    localPose = pose - this->body->GetCoMPose();

    q[0] = localPose.rot.u;
    q[1] = localPose.rot.x;
    q[2] = localPose.rot.y;
    q[3] = localPose.rot.z;

    if (!this->IsStatic())
      OgreAdaptor::Instance()->SetSceneNodePose(this->sceneNode, localPose);

    // Set the pose of the encapsulated geom; this is always relative
    // to the CoM
    dGeomSetPosition(this->geomId, localPose.pos.x, localPose.pos.y, localPose.pos.z);
    dGeomSetQuaternion(this->geomId, q); 

    if (updateCoM)
    {
      this->body->UpdateCoM();
    }
  }
  

}

////////////////////////////////////////////////////////////////////////////////
// Return the pose of the geom relative to the body
Pose3d Geom::GetPose() const
{
  Pose3d pose;

  if (this->placeable && this->geomId)
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

    //pose.rot = pose.rot * this->extraRotation.GetInverse();

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
  std::ostringstream stream;
  stream << "Geom_" << this->GetName() << ":" << (long)this->geomId;

  this->ogreObj = (Ogre::MovableObject*)(this->sceneNode->getCreator()->createEntity(stream.str(), meshName));

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

  try
  {
    // Get the original material
    this->origMaterial= Ogre::MaterialManager::getSingleton().getByName (materialName);;
  }
  catch (Ogre::Exception e)
  {
    gzmsg(0) << "Unable to get Material[" << materialName << "] for Geometry[" 
      << this->GetName() << ". Object will appear white.\n";
    return;
  }

  if (this->origMaterial.isNull())
  {
    gzmsg(0) << "Unable to get Material[" << materialName << "] for Geometry[" 
      << this->GetName() << ". Object will appear white\n";
    return;
  }

  
  // Create a custom material name
  std::string myMaterialName = this->GetName() + "_MATERIAL_" + materialName;

  // Clone the material. This will allow us to change the look of each geom
  // individually.
  this->myMaterial = this->origMaterial->clone(myMaterialName);

  Ogre::Material::TechniqueIterator techniqueIt = this->myMaterial->getTechniqueIterator ();

  while (techniqueIt.hasMoreElements ()) 
  {
    Ogre::Technique *t = techniqueIt.getNext ();
    Ogre::Technique::PassIterator passIt = t->getPassIterator ();
    while (passIt.hasMoreElements ()) 
    {
      passIt.peekNext ()->setDepthWriteEnabled (true);
      passIt.peekNext ()->setSceneBlending (this->sceneBlendType);
      passIt.moveNext ();
    }
  }

  ent = dynamic_cast<Ogre::Entity*>(this->ogreObj);
  simple = dynamic_cast<Ogre::SimpleRenderable*>(this->ogreObj);

  try
  {
    if (ent)
      ent->setMaterialName(myMaterialName);
    else if (simple)
      simple->setMaterial(myMaterialName);
  } catch (Ogre::Exception e)
  { 
    gzmsg(0) << "Unable to set Material[" << myMaterialName << "] to Geometry[" 
             << this->GetName() << ". Object will appear white.\n";
  }

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
  
  if (dMassCheck(&this->bodyMass))
  {
    dMassRotate(&this->bodyMass, r);
    dMassTranslate( &this->bodyMass, pose.pos.x, pose.pos.y, pose.pos.z);
  }

  return &this->bodyMass;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the laser fiducial integer id
void Geom::SetLaserFiducialId(int id)
{
  this->laserFiducialId = id;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the laser fiducial integer id
int Geom::GetLaserFiducialId() const
{
  return this->laserFiducialId;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the laser retro reflectiveness 
void Geom::SetLaserRetro(float retro)
{
  this->laserRetro = retro;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the laser retro reflectiveness 
float Geom::GetLaserRetro() const
{
  return this->laserRetro;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the transparency
void Geom::SetTransparency( float trans )
{
  unsigned short i = 0, j=0;
  Ogre::ColourValue sc, dc;
  Ogre::Technique *t;

  this->transparency = std::min(std::max(trans, (float)0.0), (float)1.0);

  if (this->myMaterial.isNull())
  {
    gzmsg(0) << "Can't set transparency for a geom without a material\n";
    return;
  }

  Ogre::Material::TechniqueIterator techniqueIt = this->myMaterial->getTechniqueIterator();


  while ( techniqueIt.hasMoreElements() ) 
  {
    t = techniqueIt.getNext ();
    Ogre::Technique::PassIterator passIt = t->getPassIterator ();

    j = 0;

    while (passIt.hasMoreElements ()) 
    {
      sc = this->origMaterial->getTechnique (i)->getPass (j)->getDiffuse ();

      passIt.peekNext ()->setDiffuse (0,0,0,1-this->transparency);
      if (this->transparency >0.0)
        passIt.peekNext ()->setDepthWriteEnabled (false);
      else
        passIt.peekNext ()->setDepthWriteEnabled (true);
        

      /*switch (this->sceneBlendType) 
      {
        case Ogre::SBT_ADD:
          dc = sc;
          dc.r -= sc.r * this->transparency;
          dc.g -= sc.g * this->transparency;
          dc.b -= sc.b * this->transparency;
          passIt.peekNext ()->setAmbient (Ogre::ColourValue::Black);
          break;

        case Ogre::SBT_TRANSPARENT_ALPHA:
        default:
          dc = sc;
          dc.a = sc.a * (1.0f - this->transparency);
          passIt.peekNext()->setAmbient(this->origMaterial->getTechnique (i)->getPass (j)->getAmbient ());
          break;
      }
      passIt.peekNext ()->setDiffuse (dc);
      */
      passIt.moveNext ();

      ++j;
    }

    ++i;
  }
}

////////////////////////////////////////////////////////////////////////////////
///  Get the value of the transparency
float Geom::GetTransparency() const
{
  return this->transparency;
}
 
