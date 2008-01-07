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
/* Desc: Ogre Visual Class
 * Author: Nate Koenig
 * Date: 14 Dec 2007
 * SVN: $Id$
 */
#include <Ogre.h>
#include "GazeboMessage.hh"
#include "XMLConfig.hh"
#include "OgreAdaptor.hh"
#include "OgreVisual.hh"

using namespace gazebo;

OgreVisual::OgreVisual(OgreVisual *node)
{

  std::ostringstream stream;

  if (!node)
    this->parentNode = OgreAdaptor::Instance()->sceneMgr->getRootSceneNode();
  else
    this->parentNode = node->GetSceneNode();

  this->sceneBlendType = Ogre::SBT_TRANSPARENT_ALPHA;

  // Create a unique name for the scene node
  stream << this->parentNode->getName() << "_VISUAL_" << this->parentNode->numChildren();

  // Create the scene node
  this->sceneNode = this->parentNode->createChildSceneNode( stream.str() );

  this->boundingBoxNode = NULL;
  this->sceneNode->setInheritScale(false);
}

/// \brief Destructor
OgreVisual::~OgreVisual()
{
 this->parentNode->removeAndDestroyChild(this->sceneNode->getName());
}

////////////////////////////////////////////////////////////////////////////////
// Load the visual
void OgreVisual::Load(XMLConfigNode *node)
{
  std::ostringstream stream;
  Pose3d pose;
  Vector3 size;
  Ogre::Vector3 meshSize;
  Ogre::MovableObject *obj;

  this->xmlNode=node; 
  std::string meshName = node->GetString("mesh","",1);
  std::string materialName = node->GetString("material","",0);

  // Read the desired position and rotation of the mesh
  pose.pos = node->GetVector3("xyz", Vector3(0,0,0));
  pose.rot = node->GetRotation("rpy", Quatern());


  // Create the entity
  stream << this->sceneNode->getName() << "_ENTITY";
  obj = (Ogre::MovableObject*)this->sceneNode->getCreator()->createEntity(stream.str(), meshName);

  // Attach the entity to the node
  this->AttachObject(obj);  
  
  // Get the size of the mesh
  meshSize = obj->getBoundingBox().getSize();

  // Get the desired size of the mesh
  if (node->GetChild("size") != NULL)
    size = node->GetVector3("size",Vector3(1,1,1));
  else
    size = Vector3(meshSize.z, meshSize.x, meshSize.y);

  // Get and set teh desired scale of the mesh
  if (node->GetChild("scale") != NULL)
  {
    Vector3 scale = node->GetVector3("scale",Vector3(1,1,1));

    this->sceneNode->setScale(scale.y, scale.z, scale.x);
  }
  else
  {
    this->sceneNode->setScale(size.y/meshSize.x, size.z/meshSize.y, size.x/meshSize.z);
  }

  // Set the pose of the scene node
  this->SetPose(pose);
  
  // Set the material of the mesh
  this->SetMaterial(node->GetString("material","",0));

  // Allow the sphere to cast shadows
  this->SetCastShadows(true);
}

void OgreVisual::Save()
{
  this->xmlNode->SetValue("xyz", this->GetPosition());
  this->xmlNode->SetValue("rpy", this->GetRotation());
  //TODO: A lot of information!
}

////////////////////////////////////////////////////////////////////////////////
/// Attach a renerable object to the visual
void OgreVisual::AttachObject( Ogre::MovableObject *obj)
{
  this->sceneNode->attachObject(obj);
}

////////////////////////////////////////////////////////////////////////////////
/// Attach a mesh to this visual by name
void OgreVisual::AttachMesh( const std::string &meshName )
{
  std::ostringstream stream;
  Ogre::MovableObject *obj;
  stream << this->sceneNode->getName() << "_ENTITY_" << meshName;

  obj = (Ogre::MovableObject*)(this->sceneNode->getCreator()->createEntity(stream.str(), meshName));

  this->sceneNode->attachObject((Ogre::Entity*)obj);
}

////////////////////////////////////////////////////////////////////////////////
///  Set the scale
void OgreVisual::SetScale( Vector3 scale )
{
  Ogre::Vector3 vscale;
  vscale.x=scale.y;  
  vscale.y=scale.z;
  vscale.z=scale.x;
  this->sceneNode->setScale(vscale);
}

////////////////////////////////////////////////////////////////////////////////
/// Get the scale
Vector3 OgreVisual::GetScale()
{
  Ogre::Vector3 vscale;
  vscale=this->sceneNode->getScale();
  return Vector3(vscale.z, vscale.x, vscale.y);
}


////////////////////////////////////////////////////////////////////////////////
// Set the material
void OgreVisual::SetMaterial(const std::string &materialName)
{
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
      << this->sceneNode->getName() << ". Object will appear white.\n";
    return;
  }

  if (this->origMaterial.isNull())
  {
    gzmsg(0) << "Unable to get Material[" << materialName << "] for Geometry[" 
      << this->sceneNode->getName() << ". Object will appear white\n";
    return;
  }

  
  // Create a custom material name
  std::string myMaterialName = this->sceneNode->getName() + "_MATERIAL_" + materialName;

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

  try
  {
    for (int i=0; i < this->sceneNode->numAttachedObjects(); i++)
    {
      Ogre::MovableObject *obj = this->sceneNode->getAttachedObject(i);

      if (dynamic_cast<Ogre::Entity*>(obj))
        ((Ogre::Entity*)obj)->setMaterialName(myMaterialName);
      else
        ((Ogre::SimpleRenderable*)obj)->setMaterial(myMaterialName);
    }
    
  } 
  catch (Ogre::Exception e)
  { 
    gzmsg(0) << "Unable to set Material[" << myMaterialName << "] to Geometry[" 
             << this->sceneNode->getName() << ". Object will appear white.\n";
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Set the transparency
void OgreVisual::SetTransparency( float trans )
{
  unsigned short i = 0, j=0;
  Ogre::ColourValue sc, dc;
  Ogre::Technique *t;

  this->transparency = std::min(std::max(trans, (float)0.0), (float)1.0);

  if (this->myMaterial.isNull())
  {
    gzmsg(0) << "The visual " << this->sceneNode->getName() << " can't set transparency for this geom without a material\n";
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

      if (this->transparency >0.0)
        passIt.peekNext ()->setDepthWriteEnabled (false);
      else
        passIt.peekNext ()->setDepthWriteEnabled (true);
        

      switch (this->sceneBlendType) 
      {
        case Ogre::SBT_ADD:
          dc = sc;
          dc.r -= sc.r * this->transparency;
          dc.g -= sc.g	 * this->transparency;
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
      
      passIt.moveNext ();

      ++j;
    }

    ++i;
  }

}

void OgreVisual::SetHighlight(bool highlight)
{
  /*
#include <OgreParticleSystem.h>
#include <iostream>
  Ogre::ParticleSystem *effect =OgreAdaptor::Instance()->sceneMgr->createParticleSystem(this->parentNode->getName(), "Gazebo/Aureola");
  OgreAdaptor::Instance()->sceneMgr->getRootSceneNode()->createChildSceneNode()->attachObject(effect);
  //this->sceneNode->createChildSceneNode()->attachObject(effect);
  Ogre::ParticleSystem::setDefaultNonVisibleUpdateTimeout(5);
std::cout << this->parentNode->getName() << std::endl;
*/

//FIXME:  Modifying selfIllumination is invasive to the material definition of the user
// Choose other effect.

  Ogre::Technique *t;
  Ogre::Material::TechniqueIterator techniqueIt = this->myMaterial->getTechniqueIterator();
  while ( techniqueIt.hasMoreElements() ) 
  {
    t = techniqueIt.getNext ();
    Ogre::Technique::PassIterator passIt = t->getPassIterator ();

    while (passIt.hasMoreElements ()) 
    {
      if (highlight)
      {
        passIt.peekNext ()->setSelfIllumination (1,1,1);
      }
      else
      {
        passIt.peekNext ()->setSelfIllumination (0,0,0);
      }       
      passIt.moveNext ();
    }
  }

}


////////////////////////////////////////////////////////////////////////////////
/// Set whether the visual should cast shadows
void OgreVisual::SetCastShadows(bool shadows)
{
  for (int i=0; i < this->sceneNode->numAttachedObjects(); i++)
  {
    Ogre::MovableObject *obj = this->sceneNode->getAttachedObject(i);
    obj->setCastShadows(shadows);
  }
  
}

////////////////////////////////////////////////////////////////////////////////
/// Set whether the visual is visible
void OgreVisual::SetVisible(bool visible, bool cascade)
{
  this->sceneNode->setVisible( visible, cascade );
}


////////////////////////////////////////////////////////////////////////////////
// Set the position of the visual
void OgreVisual::SetPosition( const Vector3 &pos)
{
  this->sceneNode->setPosition(pos.y, pos.z, pos.x);
}

////////////////////////////////////////////////////////////////////////////////
// Set the rotation of the visual
void OgreVisual::SetRotation( const Quatern &rot)
{
  this->sceneNode->setOrientation(rot.u, rot.y, rot.z, rot.x);
}

////////////////////////////////////////////////////////////////////////////////
// Set the pose of the visual
void OgreVisual::SetPose( const Pose3d &pose)
{
  this->SetPosition(pose.pos);
  this->SetRotation( pose.rot);
}

////////////////////////////////////////////////////////////////////////////////
// Set the position of the visual
Vector3 OgreVisual::GetPosition()
{
  Ogre::Vector3 vpos;
  Vector3 pos;
  vpos=this->sceneNode->getPosition();
  pos.x=vpos.z;
  pos.y=vpos.x;
  pos.z=vpos.y;
  return pos;
}

////////////////////////////////////////////////////////////////////////////////
// Get the rotation of the visual
Quatern OgreVisual::GetRotation( )
{
  Ogre::Quaternion vquatern;
  Quatern quatern;
  vquatern=this->sceneNode->getOrientation();
  quatern.u =vquatern.w;
  quatern.x=vquatern.z;
  quatern.y=vquatern.x;
  quatern.z=vquatern.y;
  return quatern;
}

////////////////////////////////////////////////////////////////////////////////
// Get the pose of the visual
Pose3d OgreVisual::GetPose()
{
  Pose3d pos;
  pos.pos=this->GetPosition();
  pos.rot=this->GetRotation();
  return pos;
}

////////////////////////////////////////////////////////////////////////////////
// Get this visual Ogre node 
Ogre::SceneNode * OgreVisual::GetSceneNode()
{
  return this->sceneNode;
}


////////////////////////////////////////////////////////////////////////////////
///  Create a bounding box for this visual
void OgreVisual::AttachBoundingBox(const Vector3 &min, const Vector3 &max)
{
  std::ostringstream nodeName;

  nodeName << this->sceneNode->getName()<<"_AABB_NODE";

  int i=0;
  while (this->sceneNode->getCreator()->hasSceneNode(nodeName.str()))
  {
    nodeName << "_" << i;
    i++;
  }

  this->boundingBoxNode = this->sceneNode->createChildSceneNode(nodeName.str()); 
  this->boundingBoxNode->setInheritScale(false);

  Ogre::MovableObject *odeObj = (Ogre::MovableObject*)(this->sceneNode->getCreator()->createEntity(nodeName.str()+"_OBJ", "unit_box"));

  this->boundingBoxNode->attachObject(odeObj);
  Vector3 diff = max-min;

  this->boundingBoxNode->setScale(diff.y, diff.z, diff.x);

  Ogre::Entity *ent = NULL;
  Ogre::SimpleRenderable *simple = NULL;

  ent = dynamic_cast<Ogre::Entity*>(odeObj);
  simple = dynamic_cast<Ogre::SimpleRenderable*>(odeObj);

  if (ent)
    ent->setMaterialName("Gazebo/TransparentTest");
  else if (simple)
    simple->setMaterial("Gazebo/TransparentTest");

  this->boundingBoxNode->setVisible(false);
  
}
