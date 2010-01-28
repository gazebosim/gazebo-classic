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

#include <boost/thread/recursive_mutex.hpp>

#include "RTShaderSystem.hh"
#include "MeshManager.hh"
#include "Simulator.hh"
#include "Entity.hh"
#include "GazeboMessage.hh"
#include "GazeboError.hh"
#include "XMLConfig.hh"
#include "OgreAdaptor.hh"
#include "OgreCreator.hh"
#include "Global.hh"
#include "OgreVisual.hh"

using namespace gazebo;

unsigned int OgreVisual::visualCounter = 0;

////////////////////////////////////////////////////////////////////////////////
// Constructor
OgreVisual::OgreVisual(OgreVisual *node, Entity *_owner)
  : Common()
{
  bool isStatic = false;
  Ogre::SceneNode *pnode = NULL;
  this->owner = _owner;

  if (Simulator::Instance()->GetRenderEngineEnabled())
  {
    if (!node)
      pnode = OgreAdaptor::Instance()->sceneMgr->getRootSceneNode();
    else
      pnode = node->GetSceneNode();
  }

  if (this->owner)
  {
    this->SetName(this->owner->GetName() + "_visual");
    isStatic = this->owner->IsStatic();
  }

  this->visible = true;
  this->ConstructorHelper(pnode, isStatic);
}

////////////////////////////////////////////////////////////////////////////////
/// Constructor
OgreVisual::OgreVisual (Ogre::SceneNode *node, bool isStatic)
{
  this->owner = NULL;
  this->ConstructorHelper(node, isStatic);
}

////////////////////////////////////////////////////////////////////////////////
// Helper for the contructor
void OgreVisual::ConstructorHelper(Ogre::SceneNode *node, bool isStatic)
{
  std::ostringstream stream;
  this->mutex = new boost::recursive_mutex();

  this->dirty = false;

  Param::Begin(&this->parameters);
  this->xyzP = new ParamT<Vector3>("xyz", Vector3(0,0,0), 0);
  this->xyzP->Callback( &OgreVisual::SetPosition, this );

  this->rpyP = new ParamT<Quatern>("rpy", Quatern(1,0,0,0), 0);
  this->rpyP->Callback( &OgreVisual::SetRotation, this );

  this->meshNameP = new ParamT<std::string>("mesh","",1);
  this->meshTileP = new ParamT< Vector2<double> >("uvTile", 
      Vector2<double>(1.0, 1.0), 0 );
 
  //default to Gazebo/White
  this->materialNameP = new ParamT<std::string>("material",
                                                std::string("none"),0);
  this->materialNameP->Callback( &OgreVisual::SetMaterial, this );

  this->castShadowsP = new ParamT<bool>("castShadows",true,0);
  this->castShadowsP->Callback( &OgreVisual::SetCastShadows, this );

  this->scaleP = new ParamT<Vector3>("scale", Vector3(1,1,1), 0);
  this->sizeP = new ParamT<Vector3>("size", Vector3(1,1,1), 0);
  Param::End();

  this->boundingBoxNode = NULL;

  this->ignorePoseUpdates = false;

  if (Simulator::Instance()->GetRenderEngineEnabled())
  {
    this->parentNode = node;
    this->sceneBlendType = Ogre::SBT_TRANSPARENT_ALPHA;

    // Create a unique name for the scene node
    //FIXME: what if we add the capability to delete and add new children?
    stream << this->parentNode->getName() << "_VISUAL_" << visualCounter++;

    // Create the scene node
    this->sceneNode = this->parentNode->createChildSceneNode( stream.str() );
  }

  this->isStatic = isStatic;

  /*if (this->isStatic)
    this->staticGeom = this->sceneNode->getCreator()->createStaticGeometry(
        this->GetName() + "_staticgeom");
  else
    this->staticGeom = NULL;
    */
  this->staticGeom = NULL;
}


////////////////////////////////////////////////////////////////////////////////
/// Destructor
OgreVisual::~OgreVisual()
{
  delete this->mutex;
  delete this->xyzP;
  delete this->rpyP;
  delete this->meshNameP;
  delete this->meshTileP;
  delete this->materialNameP;
  delete this->castShadowsP;

  // Having this chunk of code causes a segfault when closing the
  // application.
  /*if (this->parentNode && this->sceneNode)
  {
   this->parentNode->removeChild( this->sceneNode );
  }

  this->sceneNode->removeAndDestroyAllChildren();
  */

  for (int i=0; i < this->sceneNode->numAttachedObjects(); i++)
    RTShaderSystem::Instance()->DetachEntity(
        (Ogre::Entity*)(this->sceneNode->getAttachedObject(i)) );

  if (this->sceneNode)
    OgreAdaptor::Instance()->sceneMgr->destroySceneNode(this->sceneNode);
  if (this->boundingBoxNode)
    OgreAdaptor::Instance()->sceneMgr->destroySceneNode(this->boundingBoxNode);

}

////////////////////////////////////////////////////////////////////////////////
// Load the visual
void OgreVisual::Load(XMLConfigNode *node)
{
  boost::recursive_mutex::scoped_lock lock(*this->mutex);

  std::ostringstream stream;
  Pose3d pose;
  Vector3 size(0,0,0);
  Ogre::Vector3 meshSize(0,0,0);
  Ogre::MovableObject *obj = NULL;

  this->xyzP->Load(node);
  this->rpyP->Load(node);
  this->meshNameP->Load(node);
  this->meshTileP->Load(node);
  this->materialNameP->Load(node);
  this->castShadowsP->Load(node);

  // Read the desired position and rotation of the mesh
  pose.pos = this->xyzP->GetValue();
  pose.rot = this->rpyP->GetValue();

  // Stop here if the rendering engine has been disabled
  if (!Simulator::Instance()->GetRenderEngineEnabled())
    return;

  try
  {
    // Create the entity
    stream << "ENTITY_" << this->sceneNode->getName();
    std::string meshName = (**this->meshNameP);

    if ( meshName == "unit_box")
    {
      meshName += "_U" + 
        boost::lexical_cast<std::string>(this->meshTileP->GetValue().x) + "V" +
        boost::lexical_cast<std::string>(this->meshTileP->GetValue().y);

      if (!MeshManager::Instance()->HasMesh(meshName));
      {
        MeshManager::Instance()->CreateBox(meshName, Vector3(1,1,1), 
                                           **this->meshTileP);
      }
    }

    if (!MeshManager::Instance()->HasMesh(meshName))
      MeshManager::Instance()->Load(meshName);

    // Add the mesh into OGRE
    OgreCreator::InsertMesh( MeshManager::Instance()->GetMesh(meshName) );

    obj = (Ogre::MovableObject*)this->sceneNode->getCreator()->createEntity(
            stream.str(), meshName);
  }
  catch (Ogre::Exception e)
  {
    std::cerr << "Ogre Error:" << e.getFullDescription() << "\n";
    gzthrow("Unable to create a mesh from " + this->meshNameP->GetValue());
  }

  // Attach the entity to the node
  if (obj)
    this->AttachObject(obj);

  obj->setVisibilityFlags(GZ_ALL_CAMERA);

  // Set the pose of the scene node
  this->SetPose(pose);

  // Get the size of the mesh
  if (obj)
    meshSize = obj->getBoundingBox().getSize();

  // Get the desired size of the mesh
  if (node->GetChild("size") != NULL)
  {
    this->sizeP->Load(node);
  }
  else
    this->sizeP->SetValue( Vector3(meshSize.x, meshSize.y, meshSize.z) );

  // Get and set teh desired scale of the mesh
  if (node->GetChild("scale") != NULL)
  {
    this->scaleP->Load(node);
    Vector3 scale = this->scaleP->GetValue();
    this->sceneNode->setScale(scale.x, scale.y, scale.z);
  }
  else
  {
    Vector3 scale = this->sizeP->GetValue();
    scale.x /= meshSize.x;
    scale.y /= meshSize.y;
    scale.z /= meshSize.z;

    this->scaleP->SetValue( scale );
    this->sceneNode->setScale(scale.x, scale.y, scale.z);
  }

  // Set the material of the mesh
  if (**this->materialNameP != "none")
    this->SetMaterial(this->materialNameP->GetValue());

  // Allow the mesh to cast shadows
  this->SetCastShadows((**this->castShadowsP));
}


////////////////////////////////////////////////////////////////////////////////
// Save the visual in XML format
void OgreVisual::Save(std::string &prefix, std::ostream &stream)
{
  boost::recursive_mutex::scoped_lock lock(*this->mutex);
  stream << prefix << "<visual>\n";
  stream << prefix << "  " << *(this->xyzP) << "\n";
  stream << prefix << "  " << *(this->rpyP) << "\n";
  stream << prefix << "  " << *(this->meshNameP) << "\n";
  stream << prefix << "  " << *(this->materialNameP) << "\n";
  stream << prefix << "  " << *(this->castShadowsP) << "\n";
  stream << prefix << "  " << *(this->scaleP) << "\n";
  stream << prefix << "</visual>\n";
}

////////////////////////////////////////////////////////////////////////////////
/// Attach a renerable object to the visual
void OgreVisual::AttachObject( Ogre::MovableObject *obj)
{
  boost::recursive_mutex::scoped_lock lock(*this->mutex);

  // Stop here if the rendering engine has been disabled
  if (!Simulator::Instance()->GetRenderEngineEnabled())
    return;

  this->sceneNode->attachObject(obj);
  Ogre::Entity *ent = dynamic_cast<Ogre::Entity*>(obj);
  if (ent)
    RTShaderSystem::Instance()->AttachEntity(ent);

  obj->setUserAny( Ogre::Any(this) );
}

////////////////////////////////////////////////////////////////////////////////
/// Detach all objects
void OgreVisual::DetachObjects()
{
  boost::recursive_mutex::scoped_lock lock(*this->mutex);

  // Stop here if the rendering engine has been disabled
  if (!Simulator::Instance()->GetRenderEngineEnabled())
    return;

  this->sceneNode->detachAllObjects();
}

////////////////////////////////////////////////////////////////////////////////
/// Get the number of attached objects
unsigned short OgreVisual::GetNumAttached()
{
  boost::recursive_mutex::scoped_lock lock(*this->mutex);

  // Stop here if the rendering engine has been disabled
  if (!Simulator::Instance()->GetRenderEngineEnabled())
    return 0;

  return this->sceneNode->numAttachedObjects();
}

////////////////////////////////////////////////////////////////////////////////
/// Get an attached object
Ogre::MovableObject *OgreVisual::GetAttached(unsigned short num)
{
  boost::recursive_mutex::scoped_lock lock(*this->mutex);

  // Stop here if the rendering engine has been disabled
  if (!Simulator::Instance()->GetRenderEngineEnabled())
    return NULL;

  return this->sceneNode->getAttachedObject(num);
}

////////////////////////////////////////////////////////////////////////////////
// Attach a static object
void OgreVisual::MakeStatic()
{
  boost::recursive_mutex::scoped_lock lock(*this->mutex);

  // Stop here if the rendering engine has been disabled
  if (!Simulator::Instance()->GetRenderEngineEnabled())
    return;

  if (!this->staticGeom)
    this->staticGeom = OgreAdaptor::Instance()->sceneMgr->createStaticGeometry(this->sceneNode->getName() + "_Static");

  // Add the scene node to the static geometry
  this->staticGeom->addSceneNode(this->sceneNode);

  // Build the static geometry
  this->staticGeom->build();

  // Prevent double rendering
  this->sceneNode->setVisible(false);
}

////////////////////////////////////////////////////////////////////////////////
/// Attach a mesh to this visual by name
void OgreVisual::AttachMesh( const std::string &meshName )
{
  boost::recursive_mutex::scoped_lock lock(*this->mutex);

  // Stop here if the rendering engine has been disabled
  if (!Simulator::Instance()->GetRenderEngineEnabled())
    return;

  std::ostringstream stream;
  Ogre::MovableObject *obj;
  stream << this->sceneNode->getName() << "_ENTITY_" << meshName;

  // Add the mesh into OGRE
  if (!this->sceneNode->getCreator()->hasEntity(meshName) &&
      MeshManager::Instance()->HasMesh(meshName))
  {
    OgreCreator::InsertMesh( MeshManager::Instance()->GetMesh(meshName) );
  }

  obj = (Ogre::MovableObject*)(this->sceneNode->getCreator()->createEntity(stream.str(), meshName));

  this->AttachObject( obj );
}

////////////////////////////////////////////////////////////////////////////////
///  Set the scale
void OgreVisual::SetScale(const Vector3 &scale )
{
  boost::recursive_mutex::scoped_lock lock(*this->mutex);

  // Stop here if the rendering engine has been disabled
  if (!Simulator::Instance()->GetRenderEngineEnabled())
    return;

  Ogre::Vector3 vscale;
  vscale.x=scale.x;
  vscale.y=scale.y;
  vscale.z=scale.z;

  this->sceneNode->setScale(vscale);
}

////////////////////////////////////////////////////////////////////////////////
/// Get the scale
Vector3 OgreVisual::GetScale()
{
  boost::recursive_mutex::scoped_lock lock(*this->mutex);

  // Stop here if the rendering engine has been disabled
  if (!Simulator::Instance()->GetRenderEngineEnabled())
    return Vector3(0,0,0);

  Ogre::Vector3 vscale;
  vscale=this->sceneNode->getScale();
  return Vector3(vscale.x, vscale.y, vscale.z);
}


////////////////////////////////////////////////////////////////////////////////
// Set the material
void OgreVisual::SetMaterial(const std::string &materialName)
{
  boost::recursive_mutex::scoped_lock lock(*this->mutex);

  // Stop here if the rendering engine has been disabled
  if (!Simulator::Instance()->GetRenderEngineEnabled())
    return;

  if (materialName.empty())
    return;

  try
  {
    this->origMaterialName = materialName;
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
  this->myMaterialName = this->sceneNode->getName() + "_MATERIAL_" + materialName;

  // Clone the material. This will allow us to change the look of each geom
  // individually.
  if (Ogre::MaterialManager::getSingleton().resourceExists(this->myMaterialName))
    this->myMaterial = (Ogre::MaterialPtr)(Ogre::MaterialManager::getSingleton().getByName(this->myMaterialName));
  else
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
        ((Ogre::Entity*)obj)->setMaterialName(this->myMaterialName);
      else
        ((Ogre::SimpleRenderable*)obj)->setMaterial(this->myMaterialName);
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
  boost::recursive_mutex::scoped_lock lock(*this->mutex);

  // Stop here if the rendering engine has been disabled
  if (!Simulator::Instance()->GetRenderEngineEnabled())
    return;


  unsigned short i = 0, j=0;
  Ogre::ColourValue sc, dc;
  Ogre::Technique *t;

  this->transparency = std::min(std::max(trans, (float)0.0), (float)1.0);

  if (this->myMaterial.isNull())
  {
    gzmsg(0) << "The visual can't set transparency for this geom (" << /*geom name? << */ ") without a material"
             << " sceneNode(" << this->sceneNode->getName() << ")\n";
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
      sc = this->myMaterial->getTechnique(i)->getPass(j)->getDiffuse();

      if (this->transparency >0.0)
        passIt.peekNext ()->setDepthWriteEnabled (false);
      else
        passIt.peekNext ()->setDepthWriteEnabled (true);

      switch (this->sceneBlendType)
      {
        case Ogre::SBT_ADD:
          dc = sc;
          dc.r -= sc.r * this->transparency;
          dc.g -= sc.g  * this->transparency;
          dc.b -= sc.b * this->transparency;
          passIt.peekNext()->setAmbient(Ogre::ColourValue::Black);
          break;

        case Ogre::SBT_TRANSPARENT_ALPHA:
        default:
          dc = sc;
          //dc.a = sc.a * (1.0f - this->transparency);
          dc.a =  (1.0f - this->transparency);
          passIt.peekNext()->setAmbient(this->myMaterial->getTechnique(i)->getPass(j)->getAmbient());
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
  boost::recursive_mutex::scoped_lock lock(*this->mutex);

  // Stop here if the rendering engine has been disabled
  if (!Simulator::Instance()->GetRenderEngineEnabled())
    return;

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
/*
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
  */

}


////////////////////////////////////////////////////////////////////////////////
/// Set whether the visual should cast shadows
void OgreVisual::SetCastShadows(const bool &shadows)
{
  boost::recursive_mutex::scoped_lock lock(*this->mutex);

  // Stop here if the rendering engine has been disabled
  if (!Simulator::Instance()->GetRenderEngineEnabled())
    return;

  for (int i=0; i < this->sceneNode->numAttachedObjects(); i++)
  {
    Ogre::MovableObject *obj = this->sceneNode->getAttachedObject(i);
    obj->setCastShadows(shadows);
  }

  if (this->IsStatic() && this->staticGeom)
    this->staticGeom->setCastShadows(shadows);
}

////////////////////////////////////////////////////////////////////////////////
/// Set whether the visual is visible
void OgreVisual::SetVisible(bool visible, bool cascade)
{
  boost::recursive_mutex::scoped_lock lock(*this->mutex);

  // Stop here if the rendering engine has been disabled
  if (!Simulator::Instance()->GetRenderEngineEnabled())
    return;

  this->sceneNode->setVisible( visible, cascade );
  this->visible = visible;
}

////////////////////////////////////////////////////////////////////////////////
/// Get whether the visual is visible
bool OgreVisual::GetVisible() const
{
  return this->visible;
}

////////////////////////////////////////////////////////////////////////////////
// Set the position of the visual
void OgreVisual::SetPosition( const Vector3 &pos)
{
  boost::recursive_mutex::scoped_lock lock(*this->mutex);

  // Stop here if the rendering engine has been disabled
  if (!Simulator::Instance()->GetRenderEngineEnabled())
    return;

  if (this->ignorePoseUpdates)
    return;

  /*if (this->IsStatic() && this->staticGeom)
  {
    this->staticGeom->reset();
    delete this->staticGeom;
    this->staticGeom = NULL;
    //this->staticGeom->setOrigin( Ogre::Vector3(pos.x, pos.y, pos.z) );
  }*/

  this->sceneNode->setPosition(pos.x, pos.y, pos.z);

  /*if (this->IsStatic())
    this->MakeStatic();
    */
}

////////////////////////////////////////////////////////////////////////////////
// Set the rotation of the visual
void OgreVisual::SetRotation( const Quatern &rot)
{
  boost::recursive_mutex::scoped_lock lock(*this->mutex);

  // Stop here if the rendering engine has been disabled
  if (!Simulator::Instance()->GetRenderEngineEnabled())
    return;

  if (this->ignorePoseUpdates)
    return;

  this->sceneNode->setOrientation(rot.u, rot.x, rot.y, rot.z);
}

////////////////////////////////////////////////////////////////////////////////
// Set the pose of the visual
void OgreVisual::SetPose( const Pose3d &pose)
{
  boost::recursive_mutex::scoped_lock lock(*this->mutex);

  // Stop here if the rendering engine has been disabled
  if (!Simulator::Instance()->GetRenderEngineEnabled())
    return;

  if (this->ignorePoseUpdates)
    return;

  this->SetPosition( pose.pos );
  this->SetRotation( pose.rot);
}

////////////////////////////////////////////////////////////////////////////////
// Set the position of the visual
Vector3 OgreVisual::GetPosition() const
{
  boost::recursive_mutex::scoped_lock lock(*this->mutex);

  // Stop here if the rendering engine has been disabled
  if (!Simulator::Instance()->GetRenderEngineEnabled())
    return Vector3();

  Ogre::Vector3 vpos;
  Vector3 pos;
  vpos=this->sceneNode->getPosition();
  pos.x=vpos.x;
  pos.y=vpos.y;
  pos.z=vpos.z;
  return pos;
}

////////////////////////////////////////////////////////////////////////////////
// Get the rotation of the visual
Quatern OgreVisual::GetRotation( ) const
{
  boost::recursive_mutex::scoped_lock lock(*this->mutex);

  // Stop here if the rendering engine has been disabled
  if (!Simulator::Instance()->GetRenderEngineEnabled())
    return Quatern();

  Ogre::Quaternion vquatern;
  Quatern quatern;
  vquatern=this->sceneNode->getOrientation();
  quatern.u =vquatern.w;
  quatern.x=vquatern.x;
  quatern.y=vquatern.y;
  quatern.z=vquatern.z;
  return quatern;
}

////////////////////////////////////////////////////////////////////////////////
// Get the pose of the visual
Pose3d OgreVisual::GetPose() const
{
  boost::recursive_mutex::scoped_lock lock(*this->mutex);

  // Stop here if the rendering engine has been disabled
  if (!Simulator::Instance()->GetRenderEngineEnabled())
    return Pose3d();

  Pose3d pos;
  pos.pos=this->GetPosition();
  pos.rot=this->GetRotation();
  return pos;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the global pose of the node
Pose3d OgreVisual::GetAbsPose() const
{
  boost::recursive_mutex::scoped_lock lock(*this->mutex);

  // Stop here if the rendering engine has been disabled
  if (!Simulator::Instance()->GetRenderEngineEnabled())
    return Pose3d();

  Pose3d pose;

  Ogre::Vector3 vpos;
  Ogre::Quaternion vquatern;

  vpos=this->sceneNode->_getDerivedPosition();
  pose.pos.x=vpos.x;
  pose.pos.y=vpos.y;
  pose.pos.z=vpos.z;

  vquatern=this->sceneNode->getOrientation();
  pose.rot.u =vquatern.w;
  pose.rot.x=vquatern.x;
  pose.rot.y=vquatern.y;
  pose.rot.z=vquatern.z;


  return pose;
}


////////////////////////////////////////////////////////////////////////////////
// Get this visual Ogre node
Ogre::SceneNode * OgreVisual::GetSceneNode()
{
  boost::recursive_mutex::scoped_lock lock(*this->mutex);

  // Stop here if the rendering engine has been disabled
  if (!Simulator::Instance()->GetRenderEngineEnabled())
    return NULL;

  return this->sceneNode;
}


////////////////////////////////////////////////////////////////////////////////
///  Create a bounding box for this visual
void OgreVisual::AttachBoundingBox(const Vector3 &min, const Vector3 &max)
{
  boost::recursive_mutex::scoped_lock lock(*this->mutex);

  // Stop here if the rendering engine has been disabled
  if (!Simulator::Instance()->GetRenderEngineEnabled())
    return;

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

  if (!this->sceneNode->getCreator()->hasEntity("unit_box_U1V1"))
  {
    // Add the mesh into OGRE
    OgreCreator::InsertMesh(MeshManager::Instance()->GetMesh("unit_box_U1V1"));
  }

  Ogre::MovableObject *odeObj = (Ogre::MovableObject*)(this->sceneNode->getCreator()->createEntity(nodeName.str()+"_OBJ", "unit_box_U1V1"));

  this->boundingBoxNode->attachObject(odeObj);
  Vector3 diff = max-min;

  this->boundingBoxNode->setScale(diff.x, diff.y, diff.z);

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

////////////////////////////////////////////////////////////////////////////////
/// Get the entity that manages this visual
Entity *OgreVisual::GetOwner() const
{
  //boost::recursive_mutex::scoped_lock lock(*this->mutex);
  return this->owner;
}

////////////////////////////////////////////////////////////////////////////////
/// Set to true to show a white bounding box, used to indicate user selection
void OgreVisual::ShowSelectionBox( bool value )
{
  boost::recursive_mutex::scoped_lock lock(*this->mutex);

  // Stop here if the rendering engine has been disabled
  if (!Simulator::Instance()->GetRenderEngineEnabled())
    return;

  Ogre::SceneNode *node = this->sceneNode;

  while (node && node->numAttachedObjects() == 0)
    node = dynamic_cast<Ogre::SceneNode*>(node->getChild(0));

  if (node)
  {
    /*node = this->sceneNode;
    std::cout << "Num CHildren[" << node->numChildren() << "]\n";
    Ogre::AxisAlignedBox box = node->_getWorldAABB();
    box.scale( node->getScale() );
    Ogre::Vector3 min = box.getMinimum();
    Ogre::Vector3 max = box.getMaximum();
    printf("Scale[%f %f %f]\n", node->getScale().x, 
        node->getScale().y, node->getScale().z);
    printf("Box Min[%f %f %f] Max[%f %f %f]\n", min.x, min.y, min.z,
        max.x, max.y, max.z);
        */

    node->showBoundingBox(value);
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Set to true to discard all calls to "SetPose". This is useful for the 
/// visual node children that are part of a Geom
void OgreVisual::SetIgnorePoseUpdates( bool value )
{
  this->ignorePoseUpdates = value;
}

////////////////////////////////////////////////////////////////////////////////
/// Return true if the  visual is a static geometry
bool OgreVisual::IsStatic() const
{
  return this->isStatic;
}


////////////////////////////////////////////////////////////////////////////////
/// Set one visual to track/follow another
void OgreVisual::EnableTrackVisual( OgreVisual *vis )
{
  this->sceneNode->setAutoTracking(true, vis->GetSceneNode() );
}

////////////////////////////////////////////////////////////////////////////////
/// Disable tracking of a visual
void OgreVisual::DisableTrackVisual()
{
  this->sceneNode->setAutoTracking(false);
}
