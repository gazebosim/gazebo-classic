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

OgreVisual::OgreVisual(Ogre::SceneNode *node)
{
  this->parentNode = node;

  this->sceneBlendType = Ogre::SBT_TRANSPARENT_ALPHA;
}

/// \brief Destructor
OgreVisual::~OgreVisual()
{
}

////////////////////////////////////////////////////////////////////////////////
// Load the visual
void OgreVisual::Load(XMLConfigNode *node)
{
  std::ostringstream stream;
  Pose3d pose;
  Vector3 size;
  Ogre::Vector3 meshSize;

  std::string meshName = node->GetString("mesh","",1);
  std::string materialName = node->GetString("material","",0);

  // Read the desired position and rotation of the mesh
  pose.pos = node->GetVector3("xyz", Vector3(0,0,0));
  pose.rot = node->GetRotation("rpy", Quatern());


  // Create a unique name for the scene node
  stream << this->parentNode->getName() << "_VISUAL_" << this->parentNode->numChildren();

  // Create the scene node
  this->sceneNode = this->parentNode->createChildSceneNode( stream.str() );

  // Create the entity
  stream << "_ENTITY";
  this->entity = this->sceneNode->getCreator()->createEntity(stream.str(), meshName);

  // Attach the entity to the node
  this->sceneNode->attachObject(this->entity);
  
  // Get the size of the mesh
  meshSize = this->entity->getBoundingBox().getSize();

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
  OgreAdaptor::Instance()->SetSceneNodePose(this->sceneNode, pose);

  // Set the material of the mesh
  this->SetMaterial(node->GetString("material","",0));
}

////////////////////////////////////////////////////////////////////////////////
// Set the material
void OgreVisual::SetMaterial(const std::string &materialName)
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
    if (this->entity)
      this->entity->setMaterialName(myMaterialName);
  } catch (Ogre::Exception e)
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

      if (this->transparency >0.0)
        passIt.peekNext ()->setDepthWriteEnabled (false);
      else
        passIt.peekNext ()->setDepthWriteEnabled (true);
        

      switch (this->sceneBlendType) 
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
      
      passIt.moveNext ();

      ++j;
    }

    ++i;
  }
}
