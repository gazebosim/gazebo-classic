/*
 * Copyright 2011 Nate Koenig & Andrew Howard
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
/* Desc: Ogre Visual Class
 * Author: Nate Koenig
 * Date: 14 Dec 2007
 */

#include "rendering/ogre.h"
#include "sdf/sdf.h"

#include "msgs/msgs.h"
#include "common/Events.hh"

#include "rendering/Conversions.hh"
#include "rendering/DynamicLines.hh"
#include "rendering/Scene.hh"
#include "rendering/SelectionObj.hh"
#include "rendering/RTShaderSystem.hh"
#include "rendering/RenderEngine.hh"
#include "common/MeshManager.hh"
#include "common/Console.hh"
#include "common/Exception.hh"
#include "common/Mesh.hh"
#include "rendering/Material.hh"
#include "rendering/Visual.hh"

using namespace gazebo;
using namespace rendering;


SelectionObj *Visual::selectionObj = 0;
unsigned int Visual::visualCounter = 0;

//////////////////////////////////////////////////
Visual::Visual(const std::string &_name, VisualPtr _parent)
{
  this->SetName(_name);
  this->sceneNode = NULL;
  this->animState = NULL;

  Ogre::SceneNode *pnode = NULL;
  if (_parent)
    pnode = _parent->GetSceneNode();
  else
    gzerr << "Create a visual, invalid parent!!!\n";

  std::string uniqueName = this->GetName();
  int index = 0;
  while (pnode->getCreator()->hasSceneNode(uniqueName))
    uniqueName = this->GetName() + "_" +
                 boost::lexical_cast<std::string>(index++);

  this->SetName(uniqueName);

  this->sceneNode = pnode->createChildSceneNode(this->GetName());

  this->parent = _parent;
  this->scene = this->parent->GetScene();
  this->Init();
}

//////////////////////////////////////////////////
Visual::Visual(const std::string &_name, Scene *_scene)
{
  this->SetName(_name);
  this->sceneNode = NULL;
  this->animState = NULL;

  std::string uniqueName = this->GetName();
  int index = 0;
  while (_scene->GetManager()->hasSceneNode(uniqueName))
  {
    uniqueName = this->GetName() + "_" +
                 boost::lexical_cast<std::string>(index++);
  }

  this->scene = _scene;
  this->SetName(uniqueName);
  this->sceneNode =
    this->scene->GetManager()->getRootSceneNode()->createChildSceneNode(
        this->GetName());

  this->Init();
}

//////////////////////////////////////////////////
Visual::~Visual()
{
  RTShaderSystem::Instance()->DetachEntity(this);

  if (this->preRenderConnection)
    event::Events::DisconnectPreRender(this->preRenderConnection);

  // delete instance from lines vector
  for (std::list<DynamicLines*>::iterator iter = this->lines.begin();
       iter!= this->lines.end(); ++iter)
    delete *iter;
  this->lines.clear();


  if (this->sceneNode != NULL)
  {
    this->DestroyAllAttachedMovableObjects(this->sceneNode);
    this->sceneNode->removeAndDestroyAllChildren();
    this->scene->GetManager()->destroySceneNode(this->sceneNode->getName());
    this->sceneNode = NULL;
  }

  this->sdf->Reset();
  this->sdf.reset();
  this->parent.reset();
  this->children.clear();
}

void Visual::Fini()
{
  // Detach from the parent
  if (this->parent)
    this->parent->DetachVisual(this->GetName());

  // Detach all children
  std::vector<VisualPtr>::iterator iter;
  for (iter = this->children.begin(); iter != this->children.end(); ++iter)
  {
    this->sceneNode->removeChild((*iter)->GetSceneNode());
    (*iter)->parent.reset();
  }
  this->children.clear();

  if (this->sceneNode != NULL)
  {
    this->DestroyAllAttachedMovableObjects(this->sceneNode);
    this->sceneNode->removeAndDestroyAllChildren();
    this->sceneNode->detachAllObjects();

    this->scene->GetManager()->destroySceneNode(this->sceneNode);
    this->sceneNode = NULL;
  }

  RTShaderSystem::Instance()->DetachEntity(this);
}

void Visual::DestroyAllAttachedMovableObjects(Ogre::SceneNode* i_pSceneNode)
{
  if (!i_pSceneNode)
    return;

  // Destroy all the attached objects
  Ogre::SceneNode::ObjectIterator itObject =
    i_pSceneNode->getAttachedObjectIterator();

  while (itObject.hasMoreElements())
  {
    Ogre::Entity *ent = static_cast<Ogre::Entity*>(itObject.getNext());
    if (ent->getMovableType() != DynamicLines::GetMovableType())
      this->scene->GetManager()->destroyEntity(ent);
    else
      delete ent;
  }

  // Recurse to child SceneNodes
  Ogre::SceneNode::ChildNodeIterator itChild = i_pSceneNode->getChildIterator();

  while (itChild.hasMoreElements())
  {
    Ogre::SceneNode* pChildNode =
      static_cast<Ogre::SceneNode*>(itChild.getNext());
    this->DestroyAllAttachedMovableObjects(pChildNode);
  }
}

//////////////////////////////////////////////////
void Visual::Init()
{
  this->sdf.reset(new sdf::Element);
  sdf::initFile("/sdf/visual.sdf", this->sdf);


  this->transparency = 0.0;
  this->isStatic = false;
  this->visible = true;
  this->ribbonTrail = NULL;
  this->staticGeom = NULL;

  RTShaderSystem::Instance()->AttachEntity(this);
}

//////////////////////////////////////////////////
void Visual::LoadFromMsg(const boost::shared_ptr< msgs::Visual const> &_msg)
{
  sdf::ElementPtr geomElem = this->sdf->GetOrCreateElement("geometry");
  geomElem->ClearElements();

  if (_msg->has_geometry())
  {
    if (_msg->geometry().type() == msgs::Geometry::BOX)
    {
      sdf::ElementPtr elem = geomElem->AddElement("box");
      elem->GetAttribute("size")->Set(
          msgs::Convert(_msg->geometry().box().size()));
    }
    else if (_msg->geometry().type() == msgs::Geometry::SPHERE)
    {
      sdf::ElementPtr elem = geomElem->AddElement("sphere");
      elem->GetAttribute("radius")->Set(_msg->geometry().sphere().radius());
    }
    else if (_msg->geometry().type() == msgs::Geometry::CYLINDER)
    {
      sdf::ElementPtr elem = geomElem->AddElement("cylinder");
      elem->GetAttribute("radius")->Set(_msg->geometry().cylinder().radius());
      elem->GetAttribute("length")->Set(_msg->geometry().cylinder().length());
    }
    else if (_msg->geometry().type() == msgs::Geometry::PLANE)
    {
      math::Plane plane = msgs::Convert(_msg->geometry().plane());
      sdf::ElementPtr elem = geomElem->AddElement("plane");
      elem->GetAttribute("normal")->Set(plane.normal);
    }
    else if (_msg->geometry().type() == msgs::Geometry::MESH)
    {
      sdf::ElementPtr elem = geomElem->AddElement("mesh");
      elem->GetAttribute("filename")->Set(_msg->geometry().mesh().filename());
    }
  }

  if (_msg->has_pose())
  {
    sdf::ElementPtr elem = this->sdf->GetOrCreateElement("origin");
    math::Pose p(msgs::Convert(_msg->pose().position()),
                  msgs::Convert(_msg->pose().orientation()));

    elem->GetAttribute("pose")->Set(p);
  }

  if (_msg->has_material())
  {
    if (_msg->material().has_script())
    {
      sdf::ElementPtr elem = this->sdf->GetOrCreateElement("material");
      elem->GetAttribute("script")->Set(_msg->material().script());
    }

    if (_msg->material().has_ambient())
    {
      sdf::ElementPtr elem = this->sdf->GetOrCreateElement("material");
      elem->GetOrCreateElement("ambient")->GetAttribute("rgba")->Set(
          msgs::Convert(_msg->material().ambient()));
    }

    if (_msg->material().has_diffuse())
    {
      sdf::ElementPtr elem = this->sdf->GetOrCreateElement("material");
      elem->GetOrCreateElement("diffuse")->GetAttribute("rgba")->Set(
          msgs::Convert(_msg->material().diffuse()));
    }

    if (_msg->material().has_specular())
    {
      sdf::ElementPtr elem = this->sdf->GetOrCreateElement("material");
      elem->GetOrCreateElement("specular")->GetAttribute("rgba")->Set(
          msgs::Convert(_msg->material().specular()));
    }

    if (_msg->material().has_emissive())
    {
      sdf::ElementPtr elem = this->sdf->GetOrCreateElement("material");
      elem->GetOrCreateElement("emissive")->GetAttribute("rgba")->Set(
          msgs::Convert(_msg->material().emissive()));
    }
  }

  if (_msg->has_cast_shadows())
    this->sdf->GetAttribute("cast_shadows")->Set(_msg->cast_shadows());

  // if (msg->has_scale())
  // this->SetScale(msgs::Convert(msg->scale()));

  this->Load();
  this->UpdateFromMsg(_msg);
}

//////////////////////////////////////////////////
void Visual::Load(sdf::ElementPtr &_sdf)
{
  this->sdf = _sdf;
  this->Load();
}

//////////////////////////////////////////////////
void Visual::Load()
{
  std::ostringstream stream;
  math::Pose pose;
  Ogre::Vector3 meshSize(1, 1, 1);
  Ogre::MovableObject *obj = NULL;

  if (this->parent)
    this->parent->AttachVisual(shared_from_this());

  // Read the desired position and rotation of the mesh
  pose = this->sdf->GetOrCreateElement("origin")->GetValuePose("pose");

  std::string meshName = this->GetMeshName();

  if (!meshName.empty())
  {
    try
    {
      // Create the visual
      stream << "VISUAL_" << this->sceneNode->getName();

      const common::Mesh *mesh;
      if (!common::MeshManager::Instance()->HasMesh(meshName))
      {
        mesh = common::MeshManager::Instance()->Load(meshName);
        RenderEngine::Instance()->AddResourcePath(mesh->GetPath());
      }
      else
      {
        mesh = common::MeshManager::Instance()->GetMesh(meshName);
      }

      // Add the mesh into OGRE
      this->InsertMesh(mesh);

      Ogre::SceneManager *mgr = this->sceneNode->getCreator();
      if (mgr->hasEntity(stream.str()))
        obj = (Ogre::MovableObject*)mgr->getEntity(stream.str());
      else
        obj = (Ogre::MovableObject*)mgr->createEntity(stream.str(), meshName);
    }
    catch(Ogre::Exception &e)
    {
      gzerr << "Ogre Error:" << e.getFullDescription() << "\n";
      gzthrow("Unable to create a mesh from " + meshName);
    }
  }

  // Attach the entity to the node
  if (obj)
  {
    this->AttachObject(obj);
    obj->setVisibilityFlags(GZ_VISIBILITY_ALL);
  }

  // Set the pose of the scene node
  this->SetPose(pose);

  // Get the size of the mesh
  if (obj)
  {
    meshSize = obj->getBoundingBox().getSize();
  }

  math::Vector3 scale = this->GetScale();
  this->sceneNode->setScale(scale.x, scale.y, scale.z);

  // Set the material of the mesh
  if (this->sdf->HasElement("material"))
  {
    sdf::ElementPtr matElem = this->sdf->GetElement("material");
    std::string script = matElem->GetValueString("script");
    if (!script.empty())
      this->SetMaterial(script);
    else if (matElem->HasElement("ambient"))
      this->SetAmbient(matElem->GetElement("ambient")->GetValueColor("rgba"));
    else if (matElem->HasElement("diffuse"))
      this->SetDiffuse(matElem->GetElement("diffuse")->GetValueColor("rgba"));
    else if (matElem->HasElement("specular"))
      this->SetSpecular(matElem->GetElement("speclar")->GetValueColor("rgba"));
    else if (matElem->HasElement("emissive"))
      this->SetEmissive(matElem->GetElement("emissive")->GetValueColor("rgba"));
  }

  // Allow the mesh to cast shadows
  this->SetCastShadows(this->sdf->GetValueBool("cast_shadows"));
}

//////////////////////////////////////////////////
void Visual::Update()
{
  if (!this->visible)
    return;

  std::list<DynamicLines*>::iterator iter;

  // Update the lines
  for (iter = this->lines.begin(); iter != this->lines.end(); ++iter)
    (*iter)->Update();

  std::list< std::pair<DynamicLines*, unsigned int> >::iterator liter;
  for (liter = this->lineVertices.begin();
       liter != this->lineVertices.end(); ++liter)
  {
    liter->first->SetPoint(liter->second,
        Conversions::Convert(this->sceneNode->_getDerivedPosition()));
    liter->first->Update();
  }

  if (this->animState)
  {
    this->animState->addTime(0.01);
    if (this->animState->hasEnded())
    {
      this->animState = NULL;
      this->sceneNode->getCreator()->destroyAnimation(
          this->GetName() + "_animation");
      this->sceneNode->getCreator()->destroyAnimationState(
          this->GetName() + "_animation");
      event::Events::DisconnectPreRender(this->preRenderConnection);
    }
  }
}

//////////////////////////////////////////////////
void Visual::SetName(const std::string &name_)
{
  this->name = name_;
}

//////////////////////////////////////////////////
std::string Visual::GetName() const
{
  return this->name;
}

//////////////////////////////////////////////////
void Visual::AttachVisual(VisualPtr _vis)
{
  if (!_vis)
    gzerr << "Visual is null\n";
  else
  {
    if (_vis->GetSceneNode()->getParentSceneNode())
    {
      _vis->GetSceneNode()->getParentSceneNode()->removeChild(
          _vis->GetSceneNode());
    }
    this->sceneNode->addChild(_vis->GetSceneNode());
    this->children.push_back(_vis);
    _vis->parent = shared_from_this();
  }
}

//////////////////////////////////////////////////
void Visual::DetachVisual(VisualPtr _vis)
{
  this->DetachVisual(_vis->GetName());
}

//////////////////////////////////////////////////
void Visual::DetachVisual(const std::string &_name)
{
  std::vector<VisualPtr>::iterator iter;
  for (iter = this->children.begin(); iter != this->children.end(); ++iter)
  {
    if ((*iter)->GetName() == _name)
    {
      this->sceneNode->removeChild((*iter)->GetSceneNode());
      (*iter)->parent.reset();
      this->children.erase(iter);
      break;
    }
  }
}

//////////////////////////////////////////////////
void Visual::AttachObject(Ogre::MovableObject *_obj)
{
  // This code makes plane render before grids. This allows grids to overlay
  // planes, and then other elements to overlay both planes and grids.
  if (this->sdf->HasElement("geometry"))
    if (this->sdf->GetElement("geometry")->HasElement("plane"))
      _obj->setRenderQueueGroup(Ogre::RENDER_QUEUE_WORLD_GEOMETRY_1 - 2);

  if (!this->HasAttachedObject(_obj->getName()))
  {
    this->sceneNode->attachObject(_obj);
    RTShaderSystem::Instance()->UpdateShaders();
    _obj->setUserAny(Ogre::Any(this->GetName()));
  }
  else
    gzerr << "Visual[" << this->GetName() << "] already has object["
          << _obj->getName() << "] attached.";
}

//////////////////////////////////////////////////
bool Visual::HasAttachedObject(const std::string &_name)
{
  for (unsigned int i = 0; i < this->sceneNode->numAttachedObjects(); ++i)
  {
    if (this->sceneNode->getAttachedObject(i)->getName() == _name)
      return true;
  }

  return false;
}

//////////////////////////////////////////////////
void Visual::DetachObjects()
{
  this->sceneNode->detachAllObjects();
}

//////////////////////////////////////////////////
unsigned int Visual::GetChildCount()
{
  return this->children.size();
}

//////////////////////////////////////////////////
VisualPtr Visual::GetChild(unsigned int _num)
{
  if (_num < this->children.size())
    return this->children[_num];
  return VisualPtr();
}

//////////////////////////////////////////////////
void Visual::MakeStatic()
{
  /*if (!this->staticGeom)
    this->staticGeom =
    this->sceneNode->getCreator()->createStaticGeometry(
    this->sceneNode->getName() + "_Static");

  // Add the scene node to the static geometry
  this->staticGeom->addSceneNode(this->sceneNode);

  // Build the static geometry
  this->staticGeom->build();

  // Prevent double rendering
  this->sceneNode->setVisible(false);
  this->sceneNode->detachAllObjects();
  */
}

//////////////////////////////////////////////////
void Visual::AttachMesh(const std::string &meshName)
{
  std::ostringstream stream;
  Ogre::MovableObject *obj;
  stream << this->sceneNode->getName() << "_ENTITY_" << meshName;

  // Add the mesh into OGRE
  if (!this->sceneNode->getCreator()->hasEntity(meshName) &&
      common::MeshManager::Instance()->HasMesh(meshName))
  {
    const common::Mesh *mesh =
      common::MeshManager::Instance()->GetMesh(meshName);
    this->InsertMesh(mesh);
  }

  obj = (Ogre::MovableObject*)
    (this->sceneNode->getCreator()->createEntity(stream.str(), meshName));

  this->AttachObject(obj);
}

//////////////////////////////////////////////////
void Visual::SetScale(const math::Vector3 &_scale)
{
  sdf::ElementPtr geomElem = this->sdf->GetOrCreateElement("geometry");

  if (geomElem->HasElement("box"))
    geomElem->GetElement("box")->GetAttribute("size")->Set(_scale);
  else if (geomElem->HasElement("sphere"))
    geomElem->GetElement("sphere")->GetAttribute("radius")->Set(_scale.x);
  else if (geomElem->HasElement("cylinder"))
  {
    geomElem->GetElement("cylinder")->GetAttribute("radius")->Set(_scale.x);
    geomElem->GetElement("cylinder")->GetAttribute("length")->Set(_scale.y);
  }
  else if (geomElem->HasElement("mesh"))
    geomElem->GetElement("mesh")->GetAttribute("scale")->Set(_scale);

  this->sceneNode->setScale(Conversions::Convert(_scale));
}

//////////////////////////////////////////////////
math::Vector3 Visual::GetScale()
{
  math::Vector3 result(1, 1, 1);
  if (this->sdf->HasElement("geometry"))
  {
    sdf::ElementPtr geomElem = this->sdf->GetElement("geometry");

    if (geomElem->HasElement("box"))
    {
      result = geomElem->GetElement("box")->GetValueVector3("size");
    }
    else if (geomElem->HasElement("sphere"))
    {
      double r = geomElem->GetElement("sphere")->GetValueDouble("radius");
      result.Set(r, r, r);
    }
    else if (geomElem->HasElement("cylinder"))
    {
      double r = geomElem->GetElement("cylinder")->GetValueDouble("radius");
      double l = geomElem->GetElement("cylinder")->GetValueDouble("length");
      result.Set(r, r, l);
    }
    else if (geomElem->HasElement("plane"))
    {
      result.Set(1, 1, 1);
    }
    else if (geomElem->HasElement("mesh"))
    {
      result = geomElem->GetElement("mesh")->GetValueVector3("scale");
    }
  }

  return result;
}


//////////////////////////////////////////////////
void Visual::SetMaterial(const std::string &materialName)
{
  if (materialName.empty() || materialName == "__default__")
    return;

  // Create a custom material name
  std::string newMaterialName;
  newMaterialName = this->sceneNode->getName() + "_MATERIAL_" + materialName;

  if (this->GetMaterialName() == newMaterialName)
    return;

  this->myMaterialName = newMaterialName;

  Ogre::MaterialPtr origMaterial;

  try
  {
    this->origMaterialName = materialName;
    // Get the original material
    origMaterial =
      Ogre::MaterialManager::getSingleton().getByName(materialName);
  }
  catch(Ogre::Exception &e)
  {
    gzwarn << "Unable to get Material[" << materialName << "] for Geometry["
    << this->sceneNode->getName() << ". Object will appear white.\n";
    return;
  }

  if (origMaterial.isNull())
  {
    gzwarn << "Unable to get Material[" << materialName << "] for Geometry["
    << this->sceneNode->getName() << ". Object will appear white\n";
    return;
  }


  Ogre::MaterialPtr myMaterial;

  // Clone the material. This will allow us to change the look of each geom
  // individually.
  if (Ogre::MaterialManager::getSingleton().resourceExists(
        this->myMaterialName))
  {
    myMaterial =
      (Ogre::MaterialPtr)(Ogre::MaterialManager::getSingleton().getByName(
            this->myMaterialName));
  }
  else
  {
    myMaterial = origMaterial->clone(this->myMaterialName);
  }


  try
  {
    for (int i = 0; i < this->sceneNode->numAttachedObjects(); i++)
    {
      Ogre::MovableObject *obj = this->sceneNode->getAttachedObject(i);

      if (dynamic_cast<Ogre::Entity*>(obj))
        ((Ogre::Entity*)obj)->setMaterialName(this->myMaterialName);
      else
        ((Ogre::SimpleRenderable*)obj)->setMaterial(this->myMaterialName);
    }
  }
  catch(Ogre::Exception &e)
  {
    gzwarn << "Unable to set Material[" << this->myMaterialName
           << "] to Geometry["
           << this->sceneNode->getName() << ". Object will appear white.\n";
  }

  RTShaderSystem::Instance()->UpdateShaders();
}

/// Set the ambient color of the visual
void Visual::SetAmbient(const common::Color &_color)
{
  if (this->myMaterialName.empty())
  {
    std::string matName = this->GetName() + "_MATERIAL_";
    Ogre::MaterialManager::getSingleton().create(matName, "General");
    this->SetMaterial(matName);
  }

  for (unsigned int i = 0; i < this->children.size(); ++i)
  {
    this->children[i]->SetAmbient(_color);
  }

  for (unsigned int i = 0; i < this->sceneNode->numAttachedObjects(); ++i)
  {
    Ogre::Entity *entity = NULL;
    Ogre::MovableObject *obj = this->sceneNode->getAttachedObject(i);

    entity = dynamic_cast<Ogre::Entity*>(obj);

    if (!entity)
      continue;

    // For each ogre::entity
    for (unsigned int j = 0; j < entity->getNumSubEntities(); j++)
    {
      Ogre::SubEntity *subEntity = entity->getSubEntity(j);
      Ogre::MaterialPtr material = subEntity->getMaterial();

      unsigned int techniqueCount, passCount;
      Ogre::Technique *technique;
      Ogre::Pass *pass;
      Ogre::ColourValue dc;

      for (techniqueCount = 0; techniqueCount < material->getNumTechniques();
           techniqueCount++)
      {
        technique = material->getTechnique(techniqueCount);
        technique->setLightingEnabled(true);

        for (passCount = 0; passCount < technique->getNumPasses(); passCount++)
        {
          pass = technique->getPass(passCount);
          pass->setAmbient(Conversions::Convert(_color));
        }
      }
    }
  }

  for (unsigned int i = 0; i < this->children.size(); ++i)
  {
    this->children[i]->SetSpecular(_color);
  }
}

/// Set the diffuse color of the visual
void Visual::SetDiffuse(const common::Color &_color)
{
  if (this->myMaterialName.empty())
  {
    std::string matName = this->GetName() + "_MATERIAL_";
    Ogre::MaterialManager::getSingleton().create(matName, "General");
    this->SetMaterial(matName);
  }

  for (unsigned int i = 0; i < this->sceneNode->numAttachedObjects(); i++)
  {
    Ogre::Entity *entity = NULL;
    Ogre::MovableObject *obj = this->sceneNode->getAttachedObject(i);

    entity = dynamic_cast<Ogre::Entity*>(obj);

    if (!entity)
    {
      continue;
    }

    // For each ogre::entity
    for (unsigned int j = 0; j < entity->getNumSubEntities(); j++)
    {
      Ogre::SubEntity *subEntity = entity->getSubEntity(j);
      Ogre::MaterialPtr material = subEntity->getMaterial();

      unsigned int techniqueCount, passCount;
      Ogre::Technique *technique;
      Ogre::Pass *pass;
      Ogre::ColourValue dc;

      for (techniqueCount = 0; techniqueCount < material->getNumTechniques();
           techniqueCount++)
      {
        technique = material->getTechnique(techniqueCount);
        technique->setLightingEnabled(true);

        for (passCount = 0; passCount < technique->getNumPasses(); passCount++)
        {
          pass = technique->getPass(passCount);
          pass->setDiffuse(Conversions::Convert(_color));
        }
      }
    }
  }

  for (unsigned int i = 0; i < this->children.size(); ++i)
  {
    this->children[i]->SetDiffuse(_color);
  }
}

/// Set the specular color of the visual
void Visual::SetSpecular(const common::Color &_color)
{
  if (this->myMaterialName.empty())
  {
    std::string matName = this->GetName() + "_MATERIAL_";
    Ogre::MaterialManager::getSingleton().create(matName, "General");
    this->SetMaterial(matName);
  }

  for (unsigned int i = 0; i < this->sceneNode->numAttachedObjects(); i++)
  {
    Ogre::Entity *entity = NULL;
    Ogre::MovableObject *obj = this->sceneNode->getAttachedObject(i);

    entity = dynamic_cast<Ogre::Entity*>(obj);

    if (!entity)
      continue;

    // For each ogre::entity
    for (unsigned int j = 0; j < entity->getNumSubEntities(); j++)
    {
      Ogre::SubEntity *subEntity = entity->getSubEntity(j);
      Ogre::MaterialPtr material = subEntity->getMaterial();

      unsigned int techniqueCount, passCount;
      Ogre::Technique *technique;
      Ogre::Pass *pass;
      Ogre::ColourValue dc;

      for (techniqueCount = 0; techniqueCount < material->getNumTechniques();
           techniqueCount++)
      {
        technique = material->getTechnique(techniqueCount);
        technique->setLightingEnabled(true);

        for (passCount = 0; passCount < technique->getNumPasses(); passCount++)
        {
          pass = technique->getPass(passCount);
          pass->setSpecular(Conversions::Convert(_color));
        }
      }
    }
  }

  for (unsigned int i = 0; i < this->children.size(); ++i)
  {
    this->children[i]->SetSpecular(_color);
  }
}


void Visual::AttachAxes()
{
  std::ostringstream nodeName;

  nodeName << this->sceneNode->getName() << "_AXES_NODE";

  if (!this->sceneNode->getCreator()->hasEntity("axis_cylinder"))
    this->InsertMesh(common::MeshManager::Instance()->GetMesh("axis_cylinder"));

  Ogre::SceneNode *node = this->sceneNode->createChildSceneNode(nodeName.str());
  Ogre::SceneNode *x, *y, *z;

  x = node->createChildSceneNode(nodeName.str() + "_axisX");
  x->setInheritScale(true);
  x->translate(.25, 0, 0);
  x->yaw(Ogre::Radian(M_PI/2.0));

  y = node->createChildSceneNode(nodeName.str() + "_axisY");
  y->setInheritScale(true);
  y->translate(0, .25, 0);
  y->pitch(Ogre::Radian(M_PI/2.0));

  z = node->createChildSceneNode(nodeName.str() + "_axisZ");
  z->translate(0, 0, .25);
  z->setInheritScale(true);

  Ogre::MovableObject *xobj, *yobj, *zobj;

  xobj = (Ogre::MovableObject*)(node->getCreator()->createEntity(
        nodeName.str()+"X_AXIS", "axis_cylinder"));
  xobj->setCastShadows(false);
  ((Ogre::Entity*)xobj)->setMaterialName("Gazebo/Red");

  yobj = (Ogre::MovableObject*)(node->getCreator()->createEntity(
        nodeName.str()+"Y_AXIS", "axis_cylinder"));
  yobj->setCastShadows(false);
  ((Ogre::Entity*)yobj)->setMaterialName("Gazebo/Green");

  zobj = (Ogre::MovableObject*)(node->getCreator()->createEntity(
        nodeName.str()+"Z_AXIS", "axis_cylinder"));
  zobj->setCastShadows(false);
  ((Ogre::Entity*)zobj)->setMaterialName("Gazebo/Blue");

  x->attachObject(xobj);
  y->attachObject(yobj);
  z->attachObject(zobj);
}


//////////////////////////////////////////////////
void Visual::SetTransparency(float _trans)
{
  if (math::equal(_trans, this->transparency))
    return;

  this->transparency = std::min(
      std::max(_trans, static_cast<float>(0.0)), static_cast<float>(1.0));
  std::vector<VisualPtr>::iterator iter;
  for (iter = this->children.begin(); iter != this->children.end(); ++iter)
  {
    (*iter)->SetTransparency(_trans);
  }

  for (unsigned int i = 0; i < this->sceneNode->numAttachedObjects(); i++)
  {
    Ogre::Entity *entity = NULL;
    Ogre::MovableObject *obj = this->sceneNode->getAttachedObject(i);

    entity = dynamic_cast<Ogre::Entity*>(obj);

    if (!entity)
      continue;

    // For each ogre::entity
    for (unsigned int j = 0; j < entity->getNumSubEntities(); j++)
    {
      Ogre::SubEntity *subEntity = entity->getSubEntity(j);
      Ogre::MaterialPtr material = subEntity->getMaterial();

      unsigned int techniqueCount, passCount;
      Ogre::Technique *technique;
      Ogre::Pass *pass;
      Ogre::ColourValue dc;

      for (techniqueCount = 0; techniqueCount < material->getNumTechniques();
           techniqueCount++)
      {
        technique = material->getTechnique(techniqueCount);

        for (passCount = 0; passCount < technique->getNumPasses(); passCount++)
        {
          pass = technique->getPass(passCount);
          // Need to fix transparency
          if (!pass->isProgrammable() &&
              pass->getPolygonMode() == Ogre::PM_SOLID)
          {
            pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
          }

          if (this->transparency > 0.0)
          {
            pass->setDepthWriteEnabled(false);
            pass->setDepthCheckEnabled(false);
          }
          else
          {
            pass->setDepthWriteEnabled(true);
            pass->setDepthCheckEnabled(true);
          }


          dc = pass->getDiffuse();
          dc.a =(1.0f - this->transparency);
          pass->setDiffuse(dc);
        }
      }
    }
  }
  RTShaderSystem::Instance()->UpdateShaders();
}

//////////////////////////////////////////////////
void Visual::SetEmissive(const common::Color &_color)
{
  for (unsigned int i = 0; i < this->sceneNode->numAttachedObjects(); i++)
  {
    Ogre::Entity *entity = NULL;
    Ogre::MovableObject *obj = this->sceneNode->getAttachedObject(i);

    entity = dynamic_cast<Ogre::Entity*>(obj);

    if (!entity)
      continue;

    // For each ogre::entity
    for (unsigned int j = 0; j < entity->getNumSubEntities(); j++)
    {
      Ogre::SubEntity *subEntity = entity->getSubEntity(j);
      Ogre::MaterialPtr material = subEntity->getMaterial();

      unsigned int techniqueCount, passCount;
      Ogre::Technique *technique;
      Ogre::Pass *pass;
      Ogre::ColourValue dc;

      for (techniqueCount = 0; techniqueCount < material->getNumTechniques();
           techniqueCount++)
      {
        technique = material->getTechnique(techniqueCount);

        for (passCount = 0; passCount < technique->getNumPasses(); passCount++)
        {
          pass = technique->getPass(passCount);
          pass->setSelfIllumination(Conversions::Convert(_color));
        }
      }
    }
  }

  for (unsigned int i = 0; i < this->children.size(); ++i)
  {
    this->children[i]->SetSpecular(_color);
  }
}


//////////////////////////////////////////////////
float Visual::GetTransparency()
{
  return this->transparency;
}

//////////////////////////////////////////////////
void Visual::SetCastShadows(const bool &shadows)
{
  for (int i = 0; i < this->sceneNode->numAttachedObjects(); i++)
  {
    Ogre::MovableObject *obj = this->sceneNode->getAttachedObject(i);
    obj->setCastShadows(shadows);
  }

  if (this->IsStatic() && this->staticGeom)
    this->staticGeom->setCastShadows(shadows);
}

//////////////////////////////////////////////////
void Visual::SetVisible(bool visible_, bool cascade_)
{
  this->sceneNode->setVisible(visible_, cascade_);
  this->visible = visible_;
}

//////////////////////////////////////////////////
void Visual::ToggleVisible()
{
  this->SetVisible(!this->GetVisible());
}

//////////////////////////////////////////////////
bool Visual::GetVisible() const
{
  return this->visible;
}

//////////////////////////////////////////////////
void Visual::SetPosition(const math::Vector3 &_pos)
{
  /*if (this->IsStatic() && this->staticGeom)
  {
    this->staticGeom->reset();
    delete this->staticGeom;
    this->staticGeom = NULL;
    // this->staticGeom->setOrigin(Ogre::Vector3(pos.x, pos.y, pos.z));
  }*/
  this->sceneNode->setPosition(_pos.x, _pos.y, _pos.z);
}

//////////////////////////////////////////////////
void Visual::SetRotation(const math::Quaternion &_rot)
{
  this->sceneNode->setOrientation(
      Ogre::Quaternion(_rot.w, _rot.x, _rot.y, _rot.z));
}

//////////////////////////////////////////////////
void Visual::SetPose(const math::Pose &_pose)
{
  this->SetPosition(_pose.pos);
  this->SetRotation(_pose.rot);
}

//////////////////////////////////////////////////
math::Vector3 Visual::GetPosition() const
{
  return Conversions::Convert(this->sceneNode->getPosition());
}

//////////////////////////////////////////////////
math::Quaternion Visual::GetRotation() const
{
  return Conversions::Convert(this->sceneNode->getOrientation());
}

//////////////////////////////////////////////////
math::Pose Visual::GetPose() const
{
  math::Pose pos;
  pos.pos = this->GetPosition();
  pos.rot = this->GetRotation();
  return pos;
}

void Visual::SetWorldPose(const math::Pose _pose)
{
  this->SetWorldPosition(_pose.pos);
  this->SetWorldRotation(_pose.rot);
}

void Visual::SetWorldPosition(const math::Vector3 &_pos)
{
  this->sceneNode->_setDerivedPosition(Conversions::Convert(_pos));
}

void Visual::SetWorldRotation(const math::Quaternion &_q)
{
  Ogre::Quaternion vquatern(_q.w, _q.x, _q.y, _q.z);
  this->sceneNode->_setDerivedOrientation(vquatern);
}

//////////////////////////////////////////////////
math::Pose Visual::GetWorldPose() const
{
  math::Pose pose;

  Ogre::Vector3 vpos;
  Ogre::Quaternion vquatern;

  vpos = this->sceneNode->_getDerivedPosition();
  pose.pos.x = vpos.x;
  pose.pos.y = vpos.y;
  pose.pos.z = vpos.z;

  vquatern = this->sceneNode->getOrientation();
  pose.rot.w = vquatern.w;
  pose.rot.x = vquatern.x;
  pose.rot.y = vquatern.y;
  pose.rot.z = vquatern.z;


  return pose;
}


//////////////////////////////////////////////////
Ogre::SceneNode * Visual::GetSceneNode() const
{
  return this->sceneNode;
}


//////////////////////////////////////////////////
bool Visual::IsStatic() const
{
  return this->isStatic;
}

//////////////////////////////////////////////////
void Visual::EnableTrackVisual(Visual *vis)
{
  this->sceneNode->setAutoTracking(true, vis->GetSceneNode());
}

//////////////////////////////////////////////////
void Visual::DisableTrackVisual()
{
  this->sceneNode->setAutoTracking(false);
}

//////////////////////////////////////////////////
std::string Visual::GetNormalMap() const
{
  return this->sdf->GetOrCreateElement("material")->GetOrCreateElement(
      "shader")->GetOrCreateElement("normal_map")->GetValueString();
}

//////////////////////////////////////////////////
void Visual::SetNormalMap(const std::string &_nmap)
{
  this->sdf->GetOrCreateElement("material")->GetOrCreateElement(
      "shader")->GetOrCreateElement("normal_map")->GetValue()->Set(_nmap);
  RTShaderSystem::Instance()->UpdateShaders();
}

//////////////////////////////////////////////////
std::string Visual::GetShaderType() const
{
  return this->sdf->GetOrCreateElement("material")->GetOrCreateElement(
      "shader")->GetValueString("type");
}

//////////////////////////////////////////////////
void Visual::SetShaderType(const std::string &_type)
{
  this->sdf->GetOrCreateElement("material")->GetOrCreateElement(
      "shader")->GetAttribute("type")->Set(_type);
  RTShaderSystem::Instance()->UpdateShaders();
}


//////////////////////////////////////////////////
void Visual::SetRibbonTrail(bool value)
{
  if (this->ribbonTrail == NULL)
  {
    this->ribbonTrail =
      (Ogre::RibbonTrail*)this->sceneNode->getCreator()->createMovableObject(
          "RibbonTrail");
    this->ribbonTrail->setMaterialName("Gazebo/Red");
    this->ribbonTrail->setTrailLength(200);
    this->ribbonTrail->setMaxChainElements(1000);
    this->ribbonTrail->setNumberOfChains(1);
    this->ribbonTrail->setVisible(false);
    this->ribbonTrail->setInitialWidth(0, 0.05);
    this->sceneNode->attachObject(this->ribbonTrail);
  }


  if (value)
  {
    try
    {
      this->ribbonTrail->addNode(this->sceneNode);
    }
    catch(...)
    { }
  }
  else
  {
    this->ribbonTrail->removeNode(this->sceneNode);
    this->ribbonTrail->clearChain(0);
  }
  this->ribbonTrail->setVisible(value);
}

//////////////////////////////////////////////////
DynamicLines *Visual::CreateDynamicLine(RenderOpType type)
{
  this->preRenderConnection = event::Events::ConnectPreRender(
      boost::bind(&Visual::Update, this));

  DynamicLines *line = new DynamicLines(type);
  this->lines.push_back(line);
  this->AttachObject(line);
  return line;
}

//////////////////////////////////////////////////
void Visual::DeleteDynamicLine(DynamicLines *line)
{
  // delete instance from lines vector
  for (std::list<DynamicLines*>::iterator iter = this->lines.begin();
       iter!= this->lines.end(); ++iter)
  {
    if (*iter == line)
    {
      this->lines.erase(iter);
      break;
    }
  }
}

//////////////////////////////////////////////////
void Visual::AttachLineVertex(DynamicLines *_line, unsigned int _index)
{
  this->lineVertices.push_back(std::make_pair(_line, _index));
  _line->SetPoint(_index, this->GetWorldPose().pos);
}

//////////////////////////////////////////////////
std::string Visual::GetMaterialName() const
{
  return this->myMaterialName;
}

//////////////////////////////////////////////////
math::Box Visual::GetBoundingBox() const
{
  math::Box box;
  this->GetBoundsHelper(this->GetSceneNode(), box);
  return box;
}

//////////////////////////////////////////////////
void Visual::GetBoundsHelper(Ogre::SceneNode *node, math::Box &box) const
{
  node->_updateBounds();

  Ogre::SceneNode::ChildNodeIterator it = node->getChildIterator();

  for (int i = 0; i < node->numAttachedObjects(); i++)
  {
    Ogre::MovableObject *obj = node->getAttachedObject(i);
    if (obj->isVisible() && obj->getMovableType() != "gazebo::ogredynamiclines")
    {
      Ogre::Any any = obj->getUserAny();
      if (any.getType() == typeid(std::string))
      {
        std::string str = Ogre::any_cast<std::string>(any);
        if (str.substr(0, 3) == "rot" || str.substr(0, 5) == "trans")
          continue;
      }

      Ogre::AxisAlignedBox bb = obj->getWorldBoundingBox();
      Ogre::Vector3 min = bb.getMinimum();
      Ogre::Vector3 max = bb.getMaximum();

      box.Merge(math::Box(math::Vector3(min.x, min.y, min.z),
                          math::Vector3(max.x, max.y, max.z)));
    }
  }

  while (it.hasMoreElements())
  {
    Ogre::SceneNode *next = dynamic_cast<Ogre::SceneNode*>(it.getNext());
    this->GetBoundsHelper(next, box);
  }
}

//////////////////////////////////////////////////
void Visual::InsertMesh(const common::Mesh *mesh)
{
  Ogre::MeshPtr ogreMesh;

  if (mesh->GetSubMeshCount() == 0)
  {
    gzerr << "Visual::InsertMesh no submeshes, this is an invalid mesh\n";
    return;
  }

  try
  {
    // Create a new mesh specifically for manual definition.
    ogreMesh = Ogre::MeshManager::getSingleton().createManual(mesh->GetName(),
        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

    for (unsigned int i = 0; i < mesh->GetSubMeshCount(); i++)
    {
      Ogre::SubMesh *ogreSubMesh;
      Ogre::VertexData *vertexData;
      Ogre::VertexDeclaration* vertexDecl;
      Ogre::HardwareVertexBufferSharedPtr vBuf;
      Ogre::HardwareIndexBufferSharedPtr iBuf;
      float *vertices;
      uint16_t *indices;

      size_t currOffset = 0;

      const common::SubMesh *subMesh = mesh->GetSubMesh(i);

      ogreSubMesh = ogreMesh->createSubMesh();
      ogreSubMesh->useSharedVertices = false;
      if (subMesh->GetPrimitiveType() == common::SubMesh::TRIANGLES)
        ogreSubMesh->operationType = Ogre::RenderOperation::OT_TRIANGLE_LIST;
      else if (subMesh->GetPrimitiveType() == common::SubMesh::LINES)
        ogreSubMesh->operationType = Ogre::RenderOperation::OT_LINE_LIST;
      else if (subMesh->GetPrimitiveType() == common::SubMesh::LINESTRIPS)
        ogreSubMesh->operationType = Ogre::RenderOperation::OT_LINE_STRIP;
      else if (subMesh->GetPrimitiveType() == common::SubMesh::TRIFANS)
        ogreSubMesh->operationType = Ogre::RenderOperation::OT_TRIANGLE_FAN;
      else if (subMesh->GetPrimitiveType() == common::SubMesh::TRISTRIPS)
        ogreSubMesh->operationType = Ogre::RenderOperation::OT_TRIANGLE_STRIP;
      else
        gzerr << "Unknown primitive type["
              << subMesh->GetPrimitiveType() << "]\n";

      ogreSubMesh->vertexData = new Ogre::VertexData();
      vertexData = ogreSubMesh->vertexData;
      vertexDecl = vertexData->vertexDeclaration;

      // The vertexDecl should contain positions, blending weights, normals,
      // diffiuse colors, specular colors, tex coords. In that order.
      vertexDecl->addElement(0, currOffset, Ogre::VET_FLOAT3,
                             Ogre::VES_POSITION);
      currOffset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);

      // TODO: blending weights

      // normals
      if (subMesh->GetNormalCount() > 0)
      {
        vertexDecl->addElement(0, currOffset, Ogre::VET_FLOAT3,
                               Ogre::VES_NORMAL);
        currOffset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);
      }

      // TODO: diffuse colors

      // TODO: specular colors

      // two dimensional texture coordinates
      if (subMesh->GetTexCoordCount() > 0)
      {
        vertexDecl->addElement(0, currOffset, Ogre::VET_FLOAT2,
            Ogre::VES_TEXTURE_COORDINATES, 0);
        currOffset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT2);
      }

      // allocate the vertex buffer
      vertexData->vertexCount = subMesh->GetVertexCount();

      vBuf = Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(
                 vertexDecl->getVertexSize(0),
                 vertexData->vertexCount,
                 Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY,
                 false);

      vertexData->vertexBufferBinding->setBinding(0, vBuf);
      vertices = static_cast<float*>(vBuf->lock(
                      Ogre::HardwareBuffer::HBL_DISCARD));

      // allocate index buffer
      ogreSubMesh->indexData->indexCount = subMesh->GetIndexCount();

      ogreSubMesh->indexData->indexBuffer =
        Ogre::HardwareBufferManager::getSingleton().createIndexBuffer(
            Ogre::HardwareIndexBuffer::IT_16BIT,
            ogreSubMesh->indexData->indexCount,
            Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY,
            false);

      iBuf = ogreSubMesh->indexData->indexBuffer;
      indices = static_cast<uint16_t*>(
          iBuf->lock(Ogre::HardwareBuffer::HBL_DISCARD));

      unsigned int j;

      // Add all the vertices
      for (j = 0; j < subMesh->GetVertexCount(); j++)
      {
        *vertices++ = subMesh->GetVertex(j).x;
        *vertices++ = subMesh->GetVertex(j).y;
        *vertices++ = subMesh->GetVertex(j).z;

        if (subMesh->GetNormalCount() > 0)
        {
          *vertices++ = subMesh->GetNormal(j).x;
          *vertices++ = subMesh->GetNormal(j).y;
          *vertices++ = subMesh->GetNormal(j).z;
        }

        if (subMesh->GetTexCoordCount() > 0)
        {
          *vertices++ = subMesh->GetTexCoord(j).x;
          *vertices++ = subMesh->GetTexCoord(j).y;
        }
      }

      // Add all the indices
      for (j = 0; j < subMesh->GetIndexCount(); j++)
        *indices++ = subMesh->GetIndex(j);

      const common::Material *material;
      material = mesh->GetMaterial(subMesh->GetMaterialIndex());
      if (material)
      {
        rendering::Material::Update(material);
        ogreSubMesh->setMaterialName(material->GetName());
      }

      // Unlock
      vBuf->unlock();
      iBuf->unlock();
    }

    math::Vector3 max = mesh->GetMax();
    math::Vector3 min = mesh->GetMin();

    if (!max.IsFinite())
      gzthrow("Max bounding box is not finite[" << max << "]\n");

    if (!min.IsFinite())
      gzthrow("Min bounding box is not finite[" << min << "]\n");

    ogreMesh->_setBounds(Ogre::AxisAlignedBox(
          Ogre::Vector3(min.x, min.y, min.z),
          Ogre::Vector3(max.x, max.y, max.z)),
          false);

    // this line makes clear the mesh is loaded (avoids memory leaks)
    ogreMesh->load();
  }
  catch(Ogre::Exception &e)
  {
    gzerr << "Unable to insert mesh[" << e.getDescription() << std::endl;
  }
}

//////////////////////////////////////////////////
void Visual::UpdateFromMsg(const boost::shared_ptr< msgs::Visual const> &_msg)
{
  // TODO: Put back in, and check for performance improvements.
  /*if (msg->has_is_static() && msg->is_static())
    this->MakeStatic();
    */

  if (_msg->has_pose())
    this->SetWorldPose(msgs::Convert(_msg->pose()));

  if (_msg->has_visible())
    this->SetVisible(_msg->visible());

  if (_msg->has_transparency())
    this->SetTransparency(_msg->transparency());

  if (_msg->has_material())
  {
    if (_msg->material().has_script() && !_msg->material().script().empty())
      this->SetMaterial(_msg->material().script());

    if (_msg->material().has_ambient())
      this->SetAmbient(msgs::Convert(_msg->material().ambient()));

    if (_msg->material().has_diffuse())
      this->SetDiffuse(msgs::Convert(_msg->material().diffuse()));

    if (_msg->material().has_specular())
      this->SetSpecular(msgs::Convert(_msg->material().specular()));

    if (_msg->material().has_emissive())
      this->SetEmissive(msgs::Convert(_msg->material().emissive()));

    if (_msg->material().has_shader_type())
    {
      if (_msg->material().shader_type() == msgs::Material::VERTEX)
      {
        this->SetShaderType("vertex");
      }
      else if (_msg->material().shader_type() == msgs::Material::PIXEL)
      {
        this->SetShaderType("pixel");
      }
      else if (_msg->material().shader_type() ==
          msgs::Material::NORMAL_MAP_OBJECT_SPACE)
      {
        this->SetShaderType("normal_map_object_space");
      }
      else if (_msg->material().shader_type() ==
          msgs::Material::NORMAL_MAP_TANGENT_SPACE)
      {
        this->SetShaderType("normal_map_tangent_space");
      }

      if (_msg->material().has_normal_map())
        this->SetNormalMap(_msg->material().normal_map());
    }
  }

  if (_msg->has_geometry() && _msg->geometry().has_type())
  {
    math::Vector3 scale;

    if (_msg->geometry().type() == msgs::Geometry::BOX)
    {
      scale = msgs::Convert(_msg->geometry().box().size());
    }
    else if (_msg->geometry().type() == msgs::Geometry::CYLINDER)
    {
      scale.x = _msg->geometry().cylinder().radius() * 2.0;
      scale.y = _msg->geometry().cylinder().radius() * 2.0;
      scale.z = _msg->geometry().cylinder().length();
    }
    else if (_msg->geometry().type() == msgs::Geometry::SPHERE)
      scale.x = scale.y = scale.z = _msg->geometry().sphere().radius() * 2.0;
    else if (_msg->geometry().type() == msgs::Geometry::PLANE)
    {
      scale.x = scale.y = 1.0;
      if (_msg->geometry().plane().has_size())
      {
        scale.x = _msg->geometry().plane().size().x();
        scale.y = _msg->geometry().plane().size().y();
      }
      scale.z = 1.0;
    }
    else if (_msg->geometry().type() == msgs::Geometry::IMAGE)
      scale.x = scale.y = scale.z = _msg->geometry().image().scale();
    else if (_msg->geometry().type() == msgs::Geometry::HEIGHTMAP)
      scale = msgs::Convert(_msg->geometry().heightmap().size());
    else if (_msg->geometry().type() == msgs::Geometry::MESH)
      scale = msgs::Convert(_msg->geometry().mesh().scale());
    else
      gzerr << "Unknown geometry type[" << _msg->geometry().type() << "]\n";

    this->SetScale(scale);
  }

  /*if (msg->points.size() > 0)
  {
    DynamicLines *lines = this->AddDynamicLine(RENDERING_LINE_LIST);
    for (unsigned int i = 0; i < msg->points.size(); i++)
      lines->AddPoint(msg->points[i]);
  }
  */
}

//////////////////////////////////////////////////
VisualPtr Visual::GetParent() const
{
  return this->parent;
}

bool Visual::IsPlane() const
{
  if (this->sdf->HasElement("geometry"))
  {
    sdf::ElementPtr geomElem = this->sdf->GetElement("geometry");
    return geomElem->HasElement("plane");
  }
  return false;
}

std::string Visual::GetMeshName() const
{
  if (this->sdf->HasElement("geometry"))
  {
    sdf::ElementPtr geomElem = this->sdf->GetElement("geometry");
    if (geomElem->HasElement("box"))
      return "unit_box";
    else if (geomElem->HasElement("sphere"))
      return "unit_sphere";
    else if (geomElem->HasElement("cylinder"))
      return "unit_cylinder";
    else if (geomElem->HasElement("plane"))
      return "unit_plane";
    else if (geomElem->HasElement("mesh"))
      return geomElem->GetElement("mesh")->GetValueString("filename");
  }

  return std::string();
}

void Visual::MoveToPosition(const math::Vector3 &_end,
                             double _pitch, double _yaw, double _time)
{
  Ogre::TransformKeyFrame *key;
  math::Vector3 start = this->GetWorldPose().pos;

  math::Quaternion rotFinal(0, _pitch, _yaw);

  std::string animName = this->GetName() + "_animation";

  Ogre::Animation *anim =
    this->sceneNode->getCreator()->createAnimation(animName, _time);
  anim->setInterpolationMode(Ogre::Animation::IM_SPLINE);

  Ogre::NodeAnimationTrack *strack = anim->createNodeTrack(0, this->sceneNode);

  key = strack->createNodeKeyFrame(0);
  key->setTranslate(Ogre::Vector3(start.x, start.y, start.z));
  key->setRotation(this->sceneNode->getOrientation());

  key = strack->createNodeKeyFrame(_time);
  key->setTranslate(Ogre::Vector3(_end.x, _end.y, _end.z));
  key->setRotation(Conversions::Convert(rotFinal));

  this->animState =
    this->sceneNode->getCreator()->createAnimationState(animName);

  this->animState->setTimePosition(0);
  this->animState->setEnabled(true);
  this->animState->setLoop(false);

  this->preRenderConnection =
    event::Events::ConnectPreRender(boost::bind(&Visual::Update, this));
}

void Visual::ShowBoundingBox()
{
  this->sceneNode->showBoundingBox(true);
}

void Visual::SetScene(Scene *_scene)
{
  this->scene = _scene;
}

Scene *Visual::GetScene() const
{
  return this->scene;
}

void Visual::ShowCollision(bool _show)
{
  if (this->GetName().find("__COLLISION_VISUAL__") != std::string::npos)
    this->SetVisible(_show);

  std::vector<VisualPtr>::iterator iter;
  for (iter = this->children.begin(); iter != this->children.end(); ++iter)
  {
    (*iter)->ShowCollision(_show);
  }
}



