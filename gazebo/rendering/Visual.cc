/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
#include "gazebo/rendering/ogre_gazebo.h"

#include "gazebo/rendering/CameraVisual.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/common/Assert.hh"
#include "gazebo/common/Events.hh"
#include "gazebo/common/CommonIface.hh"

#include "gazebo/rendering/WireBox.hh"
#include "gazebo/rendering/Conversions.hh"
#include "gazebo/rendering/DynamicLines.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/RTShaderSystem.hh"
#include "gazebo/rendering/RenderEngine.hh"
#include "gazebo/common/MeshManager.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Exception.hh"
#include "gazebo/common/Mesh.hh"
#include "gazebo/common/Skeleton.hh"
#include "gazebo/rendering/Material.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/common/Plugin.hh"

using namespace gazebo;
using namespace rendering;

// Note: The value of GZ_UINT32_MAX is reserved as a flag.
uint32_t Visual::visualIdCount = GZ_UINT32_MAX - 1;

//////////////////////////////////////////////////
Visual::Visual(const std::string &_name, VisualPtr _parent, bool _useRTShader)
{
  this->id = this->visualIdCount--;
  this->boundingBox = NULL;
  this->useRTShader = _useRTShader;
  this->scale = math::Vector3::One;

  this->sdf.reset(new sdf::Element);
  sdf::initFile("visual.sdf", this->sdf);

  this->SetName(_name);
  this->sceneNode = NULL;
  this->animState = NULL;
  this->initialized = false;

  Ogre::SceneNode *pnode = NULL;
  if (_parent)
    pnode = _parent->GetSceneNode();
  else
  {
    gzerr << "Create a visual, invalid parent!!!\n";
    return;
  }

  if (!pnode)
  {
    gzerr << "Unable to get parent scene node\n";
    return;
  }

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
Visual::Visual(const std::string &_name, ScenePtr _scene, bool _useRTShader)
{
  this->id = this->visualIdCount--;
  this->boundingBox = NULL;
  this->useRTShader = _useRTShader;

  this->sdf.reset(new sdf::Element);
  sdf::initFile("visual.sdf", this->sdf);

  this->SetName(_name);
  this->sceneNode = NULL;
  this->animState = NULL;
  this->skeleton = NULL;
  this->initialized = false;

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

  delete this->boundingBox;

  // delete instance from lines vector
  /*for (std::list<DynamicLines*>::iterator iter = this->lines.begin();
       iter != this->lines.end(); ++iter)
    delete *iter;
    */
  this->lines.clear();

  if (this->sceneNode != NULL)
  {
    this->DestroyAllAttachedMovableObjects(this->sceneNode);
    this->sceneNode->removeAndDestroyAllChildren();
    this->scene->GetManager()->destroySceneNode(this->sceneNode->getName());
    this->sceneNode = NULL;
  }

  this->scene.reset();
  this->sdf->Reset();
  this->sdf.reset();
  this->parent.reset();
  this->children.clear();
}

/////////////////////////////////////////////////
void Visual::Fini()
{
  this->plugins.clear();

  // Detach from the parent
  if (this->parent)
    this->parent->DetachVisual(this->GetName());

  // Detach all children
  std::vector<VisualPtr>::iterator iter;
  for (iter = this->children.begin(); iter != this->children.end(); ++iter)
  {
    this->sceneNode->removeChild((*iter)->GetSceneNode());
    (*iter)->parent.reset();
    (*iter).reset();
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

/////////////////////////////////////////////////
VisualPtr Visual::Clone(const std::string &_name, VisualPtr _newParent)
{
  VisualPtr result(new Visual(_name, _newParent));
  result->Load(this->sdf);

  std::vector<VisualPtr>::iterator iter;
  for (iter = this->children.begin(); iter != this->children.end(); ++iter)
  {
    result->children.push_back((*iter)->Clone((*iter)->GetName(), result));
  }

  result->SetWorldPose(this->GetWorldPose());
  result->ShowCollision(false);

  return result;
}

/////////////////////////////////////////////////
void Visual::DestroyAllAttachedMovableObjects(Ogre::SceneNode *_sceneNode)
{
  if (!_sceneNode)
    return;

  // Destroy all the attached objects
  Ogre::SceneNode::ObjectIterator itObject =
    _sceneNode->getAttachedObjectIterator();

  while (itObject.hasMoreElements())
  {
    Ogre::Entity *ent = static_cast<Ogre::Entity*>(itObject.getNext());
    if (ent->getMovableType() != DynamicLines::GetMovableType())
      this->scene->GetManager()->destroyEntity(ent);
    else
      delete ent;
  }

  // Recurse to child SceneNodes
  Ogre::SceneNode::ChildNodeIterator itChild = _sceneNode->getChildIterator();

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
  this->transparency = 0.0;
  this->isStatic = false;
  this->visible = true;
  this->ribbonTrail = NULL;
  this->staticGeom = NULL;

  if (this->useRTShader)
    RTShaderSystem::Instance()->AttachEntity(this);

  this->initialized = true;
}

//////////////////////////////////////////////////
void Visual::LoadFromMsg(const boost::shared_ptr< msgs::Visual const> &_msg)
{
  sdf::ElementPtr geomElem = this->sdf->GetElement("geometry");
  geomElem->ClearElements();

  if (_msg->has_geometry())
  {
    if (_msg->geometry().type() == msgs::Geometry::BOX)
    {
      sdf::ElementPtr elem = geomElem->AddElement("box");
      elem->GetElement("size")->Set(
          msgs::Convert(_msg->geometry().box().size()));
    }
    else if (_msg->geometry().type() == msgs::Geometry::SPHERE)
    {
      sdf::ElementPtr elem = geomElem->AddElement("sphere");
      elem->GetElement("radius")->Set(_msg->geometry().sphere().radius());
    }
    else if (_msg->geometry().type() == msgs::Geometry::CYLINDER)
    {
      sdf::ElementPtr elem = geomElem->AddElement("cylinder");
      elem->GetElement("radius")->Set(_msg->geometry().cylinder().radius());
      elem->GetElement("length")->Set(_msg->geometry().cylinder().length());
    }
    else if (_msg->geometry().type() == msgs::Geometry::PLANE)
    {
      math::Plane plane = msgs::Convert(_msg->geometry().plane());
      sdf::ElementPtr elem = geomElem->AddElement("plane");
      elem->GetElement("normal")->Set(plane.normal);
      elem->GetElement("size")->Set(plane.size);
    }
    else if (_msg->geometry().type() == msgs::Geometry::MESH)
    {
      sdf::ElementPtr elem = geomElem->AddElement("mesh");
      elem->GetElement("uri")->Set(_msg->geometry().mesh().filename());

      if (_msg->geometry().mesh().has_submesh())
      {
        elem->GetElement("submesh")->GetElement("name")->Set(
            _msg->geometry().mesh().submesh());
      }

      if (_msg->geometry().mesh().has_center_submesh())
      {
        elem->GetElement("submesh")->GetElement("center")->Set(
            _msg->geometry().mesh().center_submesh());
      }
    }
  }

  if (_msg->has_pose())
  {
    sdf::ElementPtr elem = this->sdf->GetElement("pose");
    math::Pose p(msgs::Convert(_msg->pose().position()),
                  msgs::Convert(_msg->pose().orientation()));

    elem->Set(p);
  }

  if (_msg->has_material())
  {
    if (_msg->material().has_script())
    {
      sdf::ElementPtr elem =
        this->sdf->GetElement("material")->GetElement("script");
      elem->GetElement("name")->Set(_msg->material().script().name());
      for (int i = 0; i < _msg->material().script().uri_size(); ++i)
      {
        sdf::ElementPtr uriElem = elem->AddElement("uri");
        uriElem->Set(_msg->material().script().uri(i));
      }
    }

    if (_msg->material().has_ambient())
    {
      sdf::ElementPtr elem = this->sdf->GetElement("material");
      elem->GetElement("ambient")->Set(
          msgs::Convert(_msg->material().ambient()));
    }

    if (_msg->material().has_diffuse())
    {
      sdf::ElementPtr elem = this->sdf->GetElement("material");
      elem->GetElement("diffuse")->Set(
          msgs::Convert(_msg->material().diffuse()));
    }

    if (_msg->material().has_specular())
    {
      sdf::ElementPtr elem = this->sdf->GetElement("material");
      elem->GetElement("specular")->Set(
          msgs::Convert(_msg->material().specular()));
    }

    if (_msg->material().has_emissive())
    {
      sdf::ElementPtr elem = this->sdf->GetElement("material");
      elem->GetElement("emissive")->Set(
          msgs::Convert(_msg->material().emissive()));
    }
  }

  if (_msg->has_cast_shadows())
    this->sdf->GetElement("cast_shadows")->Set(_msg->cast_shadows());

  if (_msg->has_laser_retro())
    this->sdf->GetElement("laser_retro")->Set(_msg->laser_retro());

  if (_msg->has_plugin())
  {
    sdf::ElementPtr elem = this->sdf->GetElement("plugin");
    if (_msg->plugin().has_name())
      elem->GetAttribute("name")->Set(_msg->plugin().name());
    if (_msg->plugin().has_filename())
      elem->GetAttribute("filename")->Set(_msg->plugin().filename());
    if (_msg->plugin().has_innerxml())
    {
      TiXmlDocument innerXML;
      innerXML.Parse(_msg->plugin().innerxml().c_str());
      sdf::copyChildren(elem, innerXML.RootElement());
    }
  }

  this->Load();
  this->UpdateFromMsg(_msg);
}

//////////////////////////////////////////////////
void Visual::Load(sdf::ElementPtr _sdf)
{
  this->sdf->Copy(_sdf);
  this->Load();
  this->scene->AddVisual(shared_from_this());
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
  pose = this->sdf->Get<math::Pose>("pose");

  std::string meshName = this->GetMeshName();
  std::string subMeshName = this->GetSubMeshName();
  bool centerSubMesh = this->GetCenterSubMesh();

  if (!meshName.empty())
  {
    try
    {
      // Create the visual
      stream << "VISUAL_" << this->sceneNode->getName();
      obj = this->AttachMesh(meshName, subMeshName, centerSubMesh,
          stream.str());
    }
    catch(Ogre::Exception &e)
    {
      gzerr << "Ogre Error:" << e.getFullDescription() << "\n";
      gzerr << "Unable to create a mesh from " <<  meshName << "\n";
      return;
    }
  }

  Ogre::Entity *ent = static_cast<Ogre::Entity *>(obj);
  if (ent)
  {
    if (ent->hasSkeleton())
      this->skeleton = ent->getSkeleton();

    for (unsigned int i = 0; i < ent->getNumSubEntities(); i++)
    {
      ent->getSubEntity(i)->setCustomParameter(1, Ogre::Vector4(
          this->sdf->Get<double>("laser_retro"), 0.0, 0.0, 0.0));
    }
  }

  // Set the pose of the scene node
  this->SetPose(pose);

  // Get the size of the mesh
  if (obj)
    meshSize = obj->getBoundingBox().getSize();

  if (this->sdf->HasElement("geometry"))
  {
    sdf::ElementPtr geomElem = this->sdf->GetElement("geometry");

    if (geomElem->HasElement("box"))
    {
      this->scale = geomElem->GetElement("box")->Get<math::Vector3>("size");
    }
    else if (geomElem->HasElement("sphere"))
    {
      double r = geomElem->GetElement("sphere")->Get<double>("radius");
      this->scale.Set(r * 2.0, r * 2.0, r * 2.0);
    }
    else if (geomElem->HasElement("cylinder"))
    {
      double r = geomElem->GetElement("cylinder")->Get<double>("radius");
      double l = geomElem->GetElement("cylinder")->Get<double>("length");
      this->scale.Set(r * 2.0, r * 2.0, l);
    }
    else if (geomElem->HasElement("plane"))
    {
      math::Vector2d size =
        geomElem->GetElement("plane")->Get<math::Vector2d>("size");
      this->scale.Set(size.x, size.y, 1);
    }
    else if (geomElem->HasElement("mesh"))
    {
      this->scale = geomElem->GetElement("mesh")->Get<math::Vector3>("scale");
    }
  }

  this->sceneNode->setScale(this->scale.x, this->scale.y, this->scale.z);

  // Set the material of the mesh
  if (this->sdf->HasElement("material"))
  {
    sdf::ElementPtr matElem = this->sdf->GetElement("material");
    if (matElem->HasElement("script"))
    {
      sdf::ElementPtr scriptElem = matElem->GetElement("script");
      sdf::ElementPtr uriElem = scriptElem->GetElement("uri");

      // Add all the URI paths to the render engine
      while (uriElem)
      {
        std::string matUri = uriElem->Get<std::string>();
        if (!matUri.empty())
          RenderEngine::Instance()->AddResourcePath(matUri);
        uriElem = uriElem->GetNextElement("uri");
      }

      std::string matName = scriptElem->Get<std::string>("name");

      if (!matName.empty())
        this->SetMaterial(matName);
    }
    else if (matElem->HasElement("ambient"))
      this->SetAmbient(matElem->Get<common::Color>("ambient"));
    else if (matElem->HasElement("diffuse"))
      this->SetDiffuse(matElem->Get<common::Color>("diffuse"));
    else if (matElem->HasElement("specular"))
      this->SetSpecular(matElem->Get<common::Color>("specular"));
    else if (matElem->HasElement("emissive"))
      this->SetEmissive(matElem->Get<common::Color>("emissive"));
  }

  // Allow the mesh to cast shadows
  this->SetCastShadows(this->sdf->Get<bool>("cast_shadows"));
  this->LoadPlugins();
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
    this->animState->addTime(
        (common::Time::GetWallTime() - this->prevAnimTime).Double());
    this->prevAnimTime = common::Time::GetWallTime();
    if (this->animState->hasEnded())
    {
      this->animState = NULL;
      this->sceneNode->getCreator()->destroyAnimation(
          this->GetName() + "_animation");
      if (this->onAnimationComplete)
        this->onAnimationComplete();
      // event::Events::DisconnectPreRender(this->preRenderConnection);
    }
  }
}

//////////////////////////////////////////////////
void Visual::SetName(const std::string &_name)
{
  this->name = _name;
  this->sdf->GetAttribute("name")->Set(_name);
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
      VisualPtr childVis = (*iter);
      this->children.erase(iter);
      this->sceneNode->removeChild(childVis->GetSceneNode());
      childVis->GetParent().reset();
      break;
    }
  }
}

//////////////////////////////////////////////////
void Visual::AttachObject(Ogre::MovableObject *_obj)
{
  // This code makes plane render before grids. This allows grids to overlay
  // planes, and then other elements to overlay both planes and grids.
  // if (this->sdf->HasElement("geometry"))
  // if (this->sdf->GetElement("geometry")->HasElement("plane"))
  // _obj->setRenderQueueGroup(Ogre::RENDER_QUEUE_SKIES_EARLY+1);

  if (!this->HasAttachedObject(_obj->getName()))
  {
    this->sceneNode->attachObject(_obj);
    if (this->useRTShader && this->scene->GetInitialized() &&
      _obj->getName().find("__COLLISION_VISUAL__") == std::string::npos)
    {
      RTShaderSystem::Instance()->UpdateShaders();
    }
    _obj->setUserAny(Ogre::Any(this->GetName()));
  }
  else
    gzerr << "Visual[" << this->GetName() << "] already has object["
          << _obj->getName() << "] attached.";

  _obj->setVisibilityFlags(GZ_VISIBILITY_ALL);
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
unsigned int Visual::GetAttachedObjectCount() const
{
  return this->sceneNode->numAttachedObjects();
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
Ogre::MovableObject *Visual::AttachMesh(const std::string &_meshName,
                                        const std::string &_subMesh,
                                        bool _centerSubmesh,
                                        const std::string &_objName)
{
  if (_meshName.empty())
    return NULL;

  Ogre::MovableObject *obj;
  std::string objName = _objName;
  std::string meshName = _meshName;
  meshName += _subMesh.empty() ? "" : "::" + _subMesh;

  if (objName.empty())
    objName = this->sceneNode->getName() + "_ENTITY_" + meshName;

  this->InsertMesh(_meshName, _subMesh, _centerSubmesh);

  obj = (Ogre::MovableObject*)
      (this->sceneNode->getCreator()->createEntity(objName, meshName));

  this->AttachObject(obj);
  return obj;
}

//////////////////////////////////////////////////
void Visual::SetScale(const math::Vector3 &_scale)
{
  if (this->scale == _scale)
    return;

  math::Vector3 tmpScale = this->scale;
  this->scale = _scale;

  sdf::ElementPtr geomElem = this->sdf->GetElement("geometry");

  if (geomElem->HasElement("box"))
    geomElem->GetElement("box")->GetElement("size")->Set(_scale);
  else if (geomElem->HasElement("sphere"))
  {
    geomElem->GetElement("sphere")->GetElement("radius")->Set(_scale.x/2.0);
  }
  else if (geomElem->HasElement("cylinder"))
  {
    geomElem->GetElement("cylinder")->GetElement("radius")
        ->Set(_scale.x/2.0);
    geomElem->GetElement("cylinder")->GetElement("length")->Set(_scale.z);
  }
  else if (geomElem->HasElement("mesh"))
    geomElem->GetElement("mesh")->GetElement("scale")->Set(_scale);

  this->sceneNode->setScale(Conversions::Convert(this->scale));
}

//////////////////////////////////////////////////
math::Vector3 Visual::GetScale()
{
  return this->scale;
  /*math::Vector3 result(1, 1, 1);
  if (this->sdf->HasElement("geometry"))
  {
    sdf::ElementPtr geomElem = this->sdf->GetElement("geometry");

    if (geomElem->HasElement("box"))
    {
      result = geomElem->GetElement("box")->Get<math::Vector3>("size");
    }
    else if (geomElem->HasElement("sphere"))
    {
      double r = geomElem->GetElement("sphere")->Get<double>("radius");
      result.Set(r * 2.0, r * 2.0, r * 2.0);
    }
    else if (geomElem->HasElement("cylinder"))
    {
      double r = geomElem->GetElement("cylinder")->Get<double>("radius");
      double l = geomElem->GetElement("cylinder")->Get<double>("length");
      result.Set(r * 2.0, r * 2.0, l);
    }
    else if (geomElem->HasElement("plane"))
    {
      math::Vector2d size =
        geomElem->GetElement("plane")->Get<math::Vector2d>("size");
      result.Set(size.x, size.y, 1);
    }
    else if (geomElem->HasElement("mesh"))
    {
      result = geomElem->GetElement("mesh")->Get<math::Vector3>("scale");
    }
  }

  return result;*/
}


//////////////////////////////////////////////////
void Visual::SetMaterial(const std::string &_materialName, bool _unique)
{
  if (_materialName.empty() || _materialName == "__default__")
    return;

  if (_unique)
  {
    // Create a custom material name
    std::string newMaterialName;
    newMaterialName = this->sceneNode->getName() + "_MATERIAL_" + _materialName;

    if (this->GetMaterialName() == newMaterialName)
      return;

    this->myMaterialName = newMaterialName;

    Ogre::MaterialPtr origMaterial;
    try
    {
      this->origMaterialName = _materialName;
      // Get the original material
      origMaterial =
        Ogre::MaterialManager::getSingleton().getByName(_materialName);
    }
    catch(Ogre::Exception &e)
    {
      gzwarn << "Unable to get Material[" << _materialName << "] for Geometry["
        << this->sceneNode->getName() << ". Object will appear white.\n";
      return;
    }

    if (origMaterial.isNull())
    {
      gzwarn << "Unable to get Material[" << _materialName << "] for Geometry["
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
  }
  else
  {
    if ( this->myMaterialName == _materialName)
      return;
    this->myMaterialName = _materialName;
  }

  try
  {
    for (int i = 0; i < this->sceneNode->numAttachedObjects(); i++)
    {
      Ogre::MovableObject *obj = this->sceneNode->getAttachedObject(i);

      if (dynamic_cast<Ogre::Entity*>(obj))
        ((Ogre::Entity*)obj)->setMaterialName(this->myMaterialName);
      else if (dynamic_cast<Ogre::SimpleRenderable*>(obj))
        ((Ogre::SimpleRenderable*)obj)->setMaterial(this->myMaterialName);
    }

    // Apply material to all child scene nodes
    for (unsigned int i = 0; i < this->sceneNode->numChildren(); ++i)
    {
      Ogre::SceneNode *sn = (Ogre::SceneNode*)(this->sceneNode->getChild(i));
      for (int j = 0; j < sn->numAttachedObjects(); j++)
      {
        Ogre::MovableObject *obj = sn->getAttachedObject(j);

        if (dynamic_cast<Ogre::Entity*>(obj))
          ((Ogre::Entity*)obj)->setMaterialName(this->myMaterialName);
        else
          ((Ogre::SimpleRenderable*)obj)->setMaterial(this->myMaterialName);
      }
    }
  }
  catch(Ogre::Exception &e)
  {
    gzwarn << "Unable to set Material[" << this->myMaterialName
           << "] to Geometry["
           << this->sceneNode->getName() << ". Object will appear white.\n";
  }

  // Apply material to all child visuals
  for (std::vector<VisualPtr>::iterator iter = this->children.begin();
       iter != this->children.end(); ++iter)
  {
    (*iter)->SetMaterial(_materialName, _unique);
  }

  if (this->useRTShader && this->scene->GetInitialized() &&
    this->GetName().find("__COLLISION_VISUAL__") == std::string::npos)
  {
    RTShaderSystem::Instance()->UpdateShaders();
  }
}

/////////////////////////////////////////////////
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

/////////////////////////////////////////////////
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
void Visual::SetWireframe(bool _show)
{
  std::vector<VisualPtr>::iterator iter;
  for (iter = this->children.begin(); iter != this->children.end(); ++iter)
  {
    (*iter)->SetWireframe(_show);
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
      if (material.isNull())
        continue;

      unsigned int techniqueCount, passCount;
      Ogre::Technique *technique;
      Ogre::Pass *pass;

      for (techniqueCount = 0; techniqueCount < material->getNumTechniques();
           ++techniqueCount)
      {
        technique = material->getTechnique(techniqueCount);

        for (passCount = 0; passCount < technique->getNumPasses(); passCount++)
        {
          pass = technique->getPass(passCount);
          if (_show)
            pass->setPolygonMode(Ogre::PM_WIREFRAME);
          else
            pass->setPolygonMode(Ogre::PM_SOLID);
        }
      }
    }
  }
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

    if (entity->getName().find("__COLLISION_VISUAL__") != std::string::npos)
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
            pass->setDepthCheckEnabled(true);
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

  if (this->useRTShader && this->scene->GetInitialized())
    RTShaderSystem::Instance()->UpdateShaders();
}

//////////////////////////////////////////////////
void Visual::SetHighlighted(bool _highlighted)
{
  if (_highlighted)
  {
    // Create the bounding box if it's not already created.
    if (!this->boundingBox)
    {
      this->boundingBox = new WireBox(shared_from_this(),
                                      this->GetBoundingBox());
    }

    this->boundingBox->SetVisible(true);
  }
  else if (this->boundingBox)
  {
    this->boundingBox->SetVisible(false);
  }
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

        for (passCount = 0; passCount < technique->getNumPasses();
            passCount++)
        {
          pass = technique->getPass(passCount);
          pass->setSelfIllumination(Conversions::Convert(_color));
        }
      }
    }
  }

  for (unsigned int i = 0; i < this->children.size(); ++i)
  {
    this->children[i]->SetEmissive(_color);
  }
}

//////////////////////////////////////////////////
float Visual::GetTransparency()
{
  return this->transparency;
}

//////////////////////////////////////////////////
void Visual::SetCastShadows(bool shadows)
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
void Visual::SetVisible(bool _visible, bool _cascade)
{
  this->sceneNode->setVisible(_visible, _cascade);
  this->visible = _visible;
}

//////////////////////////////////////////////////
uint32_t Visual::GetVisibilityFlags()
{
  if (this->sceneNode->numAttachedObjects() > 0)
  {
    return this->sceneNode->getAttachedObject(0)->getVisibilityFlags();
  }

  return GZ_VISIBILITY_ALL;
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
  GZ_ASSERT(this->sceneNode, "Visual SceneNode is NULL");
  this->sceneNode->setPosition(_pos.x, _pos.y, _pos.z);
}

//////////////////////////////////////////////////
void Visual::SetRotation(const math::Quaternion &_rot)
{
  GZ_ASSERT(this->sceneNode, "Visual SceneNode is NULL");
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

//////////////////////////////////////////////////
void Visual::SetWorldPose(const math::Pose _pose)
{
  this->SetWorldPosition(_pose.pos);
  this->SetWorldRotation(_pose.rot);
}

//////////////////////////////////////////////////
void Visual::SetWorldPosition(const math::Vector3 &_pos)
{
  this->sceneNode->_setDerivedPosition(Conversions::Convert(_pos));
}

//////////////////////////////////////////////////
void Visual::SetWorldRotation(const math::Quaternion &_q)
{
  this->sceneNode->_setDerivedOrientation(Conversions::Convert(_q));
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

  vquatern = this->sceneNode->_getDerivedOrientation();
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
void Visual::EnableTrackVisual(VisualPtr _vis)
{
  this->sceneNode->setAutoTracking(true, _vis->GetSceneNode());
}

//////////////////////////////////////////////////
void Visual::DisableTrackVisual()
{
  this->sceneNode->setAutoTracking(false);
}

//////////////////////////////////////////////////
std::string Visual::GetNormalMap() const
{
  std::string file = this->sdf->GetElement("material")->GetElement(
      "shader")->GetElement("normal_map")->Get<std::string>();

  std::string uriFile = common::find_file(file);
  if (!uriFile.empty())
    file = uriFile;

  return file;
}

//////////////////////////////////////////////////
void Visual::SetNormalMap(const std::string &_nmap)
{
  this->sdf->GetElement("material")->GetElement(
      "shader")->GetElement("normal_map")->GetValue()->Set(_nmap);
  if (this->useRTShader && this->scene->GetInitialized())
    RTShaderSystem::Instance()->UpdateShaders();
}

//////////////////////////////////////////////////
std::string Visual::GetShaderType() const
{
  return this->sdf->GetElement("material")->GetElement(
      "shader")->Get<std::string>("type");
}

//////////////////////////////////////////////////
void Visual::SetShaderType(const std::string &_type)
{
  this->sdf->GetElement("material")->GetElement(
      "shader")->GetAttribute("type")->Set(_type);
  if (this->useRTShader && this->scene->GetInitialized())
    RTShaderSystem::Instance()->UpdateShaders();
}


//////////////////////////////////////////////////
void Visual::SetRibbonTrail(bool _value, const common::Color &_initialColor,
                            const common::Color &_changeColor)
{
  if (this->ribbonTrail == NULL)
  {
    this->ribbonTrail =
      this->scene->GetManager()->createRibbonTrail(this->GetName() +
                                                   "_RibbonTrail");
    this->ribbonTrail->setMaterialName("Gazebo/RibbonTrail");
    // this->ribbonTrail->setTrailLength(100);
    this->ribbonTrail->setMaxChainElements(10000);
    // this->ribbonTrail->setNumberOfChains(1);
    this->ribbonTrail->setVisible(false);
    this->ribbonTrail->setCastShadows(false);
    this->ribbonTrail->setInitialWidth(0, 0.05);
    this->scene->GetManager()->getRootSceneNode()->attachObject(
        this->ribbonTrail);

    this->ribbonTrail->setInitialColour(0, Conversions::Convert(_initialColor));
    this->ribbonTrail->setColourChange(0, Conversions::Convert(_changeColor));
  }


  if (_value)
  {
    try
    {
      this->ribbonTrail->addNode(this->sceneNode);
    }
    catch(...)
    {
      gzerr << "Unable to create ribbon trail\n";
    }
  }
  else
  {
    this->ribbonTrail->removeNode(this->sceneNode);
    this->ribbonTrail->clearChain(0);
  }
  this->ribbonTrail->setVisible(_value);
}

//////////////////////////////////////////////////
DynamicLines *Visual::CreateDynamicLine(RenderOpType _type)
{
  this->preRenderConnection = event::Events::ConnectPreRender(
      boost::bind(&Visual::Update, this));

  DynamicLines *line = new DynamicLines(_type);
  this->lines.push_back(line);
  this->AttachObject(line);
  return line;
}

//////////////////////////////////////////////////
void Visual::DeleteDynamicLine(DynamicLines *_line)
{
  // delete instance from lines vector
  for (std::list<DynamicLines*>::iterator iter = this->lines.begin();
       iter != this->lines.end(); ++iter)
  {
    if (*iter == _line)
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

    if (obj->isVisible() && obj->getMovableType() != "gazebo::dynamiclines"
        && obj->getVisibilityFlags() != GZ_VISIBILITY_GUI)
    {
      Ogre::Any any = obj->getUserAny();
      if (any.getType() == typeid(std::string))
      {
        std::string str = Ogre::any_cast<std::string>(any);
        if (str.substr(0, 3) == "rot" || str.substr(0, 5) == "trans")
          continue;
      }

      Ogre::AxisAlignedBox bb = obj->getBoundingBox();

      math::Vector3 min;
      math::Vector3 max;
      math::Quaternion rotDiff;
      math::Vector3 posDiff;

      rotDiff = Conversions::Convert(node->_getDerivedOrientation()) -
                this->GetWorldPose().rot;

      posDiff = Conversions::Convert(node->_getDerivedPosition()) -
                this->GetWorldPose().pos;

      // Ogre does not return a valid bounding box for lights.
      if (obj->getMovableType() == "Light")
      {
        min = math::Vector3(-0.5, -0.5, -0.5);
        max = math::Vector3(0.5, 0.5, 0.5);
      }
      else
      {
        min = rotDiff *
          Conversions::Convert(bb.getMinimum() * node->getScale()) + posDiff;
        max = rotDiff *
          Conversions::Convert(bb.getMaximum() * node->getScale()) + posDiff;
      }


      box.Merge(math::Box(min, max));
    }
  }

  while (it.hasMoreElements())
  {
    Ogre::SceneNode *next = dynamic_cast<Ogre::SceneNode*>(it.getNext());
    this->GetBoundsHelper(next, box);
  }
}

//////////////////////////////////////////////////
void Visual::InsertMesh(const std::string &_meshName,
                        const std::string &_subMesh,
                        bool _centerSubmesh)
{
  const common::Mesh *mesh;
  if (!common::MeshManager::Instance()->HasMesh(_meshName))
  {
    mesh = common::MeshManager::Instance()->Load(_meshName);
    if (!mesh)
    {
      gzerr << "Unable to create a mesh from " << _meshName << "\n";
      return;
    }
  }
  else
  {
    mesh = common::MeshManager::Instance()->GetMesh(_meshName);
  }

  this->InsertMesh(mesh, _subMesh, _centerSubmesh);

  // Add the mesh into OGRE
  /*if (!this->sceneNode->getCreator()->hasEntity(_meshName) &&
      common::MeshManager::Instance()->HasMesh(_meshName))
  {
    const common::Mesh *mesh =
      common::MeshManager::Instance()->GetMesh(_meshName);
    this->InsertMesh(mesh);
  }*/
}

//////////////////////////////////////////////////
void Visual::InsertMesh(const common::Mesh *_mesh, const std::string &_subMesh,
    bool _centerSubmesh)
{
  Ogre::MeshPtr ogreMesh;

  GZ_ASSERT(_mesh != NULL, "Unable to insert a NULL mesh");

  RenderEngine::Instance()->AddResourcePath(_mesh->GetPath());

  if (_mesh->GetSubMeshCount() == 0)
  {
    gzerr << "Visual::InsertMesh no submeshes, this is an invalid mesh\n";
    return;
  }

  // Don't re-add existing meshes
  if (Ogre::MeshManager::getSingleton().resourceExists(_mesh->GetName()))
  {
    return;
  }

  try
  {
    // Create a new mesh specifically for manual definition.
    if (_subMesh.empty())
    {
      ogreMesh = Ogre::MeshManager::getSingleton().createManual(
          _mesh->GetName(),
          Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    }
    else
    {
      ogreMesh = Ogre::MeshManager::getSingleton().createManual(
          _mesh->GetName() + "::" + _subMesh,
          Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    }

    Ogre::SkeletonPtr ogreSkeleton;

    if (_mesh->HasSkeleton())
    {
      common::Skeleton *skel = _mesh->GetSkeleton();
      ogreSkeleton = Ogre::SkeletonManager::getSingleton().create(
        _mesh->GetName() + "_skeleton",
        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
        true);

      for (unsigned int i = 0; i < skel->GetNumNodes(); i++)
      {
        common::SkeletonNode *node = skel->GetNodeByHandle(i);
        Ogre::Bone *bone = ogreSkeleton->createBone(node->GetName());

        if (node->GetParent())
          ogreSkeleton->getBone(node->GetParent()->GetName())->addChild(bone);

        math::Matrix4 trans = node->GetTransform();
        math::Vector3 pos = trans.GetTranslation();
        math::Quaternion q = trans.GetRotation();
        bone->setPosition(Ogre::Vector3(pos.x, pos.y, pos.z));
        bone->setOrientation(Ogre::Quaternion(q.w, q.x, q.y, q.z));
        bone->setInheritOrientation(true);
        bone->setManuallyControlled(true);
        bone->setInitialState();
      }
      ogreMesh->setSkeletonName(_mesh->GetName() + "_skeleton");
    }

    for (unsigned int i = 0; i < _mesh->GetSubMeshCount(); i++)
    {
      if (!_subMesh.empty() && _mesh->GetSubMesh(i)->GetName() != _subMesh)
        continue;

      Ogre::SubMesh *ogreSubMesh;
      Ogre::VertexData *vertexData;
      Ogre::VertexDeclaration* vertexDecl;
      Ogre::HardwareVertexBufferSharedPtr vBuf;
      Ogre::HardwareIndexBufferSharedPtr iBuf;
      float *vertices;
      uint32_t *indices;

      size_t currOffset = 0;

      // Copy the original submesh. We may need to modify the vertices, and
      // we don't want to change the original.
      common::SubMesh subMesh(_mesh->GetSubMesh(i));

      // Recenter the vertices if requested.
      if (_centerSubmesh)
        subMesh.Center();

      ogreSubMesh = ogreMesh->createSubMesh();
      ogreSubMesh->useSharedVertices = false;
      if (subMesh.GetPrimitiveType() == common::SubMesh::TRIANGLES)
        ogreSubMesh->operationType = Ogre::RenderOperation::OT_TRIANGLE_LIST;
      else if (subMesh.GetPrimitiveType() == common::SubMesh::LINES)
        ogreSubMesh->operationType = Ogre::RenderOperation::OT_LINE_LIST;
      else if (subMesh.GetPrimitiveType() == common::SubMesh::LINESTRIPS)
        ogreSubMesh->operationType = Ogre::RenderOperation::OT_LINE_STRIP;
      else if (subMesh.GetPrimitiveType() == common::SubMesh::TRIFANS)
        ogreSubMesh->operationType = Ogre::RenderOperation::OT_TRIANGLE_FAN;
      else if (subMesh.GetPrimitiveType() == common::SubMesh::TRISTRIPS)
        ogreSubMesh->operationType = Ogre::RenderOperation::OT_TRIANGLE_STRIP;
      else if (subMesh.GetPrimitiveType() == common::SubMesh::POINTS)
        ogreSubMesh->operationType = Ogre::RenderOperation::OT_POINT_LIST;
      else
        gzerr << "Unknown primitive type["
              << subMesh.GetPrimitiveType() << "]\n";

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
      if (subMesh.GetNormalCount() > 0)
      {
        vertexDecl->addElement(0, currOffset, Ogre::VET_FLOAT3,
                               Ogre::VES_NORMAL);
        currOffset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);
      }

      // TODO: diffuse colors

      // TODO: specular colors

      // two dimensional texture coordinates
      if (subMesh.GetTexCoordCount() > 0)
      {
        vertexDecl->addElement(0, currOffset, Ogre::VET_FLOAT2,
            Ogre::VES_TEXTURE_COORDINATES, 0);
        currOffset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT2);
      }

      // allocate the vertex buffer
      vertexData->vertexCount = subMesh.GetVertexCount();

      vBuf = Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(
                 vertexDecl->getVertexSize(0),
                 vertexData->vertexCount,
                 Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY,
                 false);

      vertexData->vertexBufferBinding->setBinding(0, vBuf);
      vertices = static_cast<float*>(vBuf->lock(
                      Ogre::HardwareBuffer::HBL_DISCARD));

      if (_mesh->HasSkeleton())
      {
        common::Skeleton *skel = _mesh->GetSkeleton();
        for (unsigned int j = 0; j < subMesh.GetNodeAssignmentsCount(); j++)
        {
          common::NodeAssignment na = subMesh.GetNodeAssignment(j);
          Ogre::VertexBoneAssignment vba;
          vba.vertexIndex = na.vertexIndex;
          vba.boneIndex = ogreSkeleton->getBone(skel->GetNodeByHandle(
                              na.nodeIndex)->GetName())->getHandle();
          vba.weight = na.weight;
          ogreSubMesh->addBoneAssignment(vba);
        }
      }

      // allocate index buffer
      ogreSubMesh->indexData->indexCount = subMesh.GetIndexCount();

      ogreSubMesh->indexData->indexBuffer =
        Ogre::HardwareBufferManager::getSingleton().createIndexBuffer(
            Ogre::HardwareIndexBuffer::IT_32BIT,
            ogreSubMesh->indexData->indexCount,
            Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY,
            false);

      iBuf = ogreSubMesh->indexData->indexBuffer;
      indices = static_cast<uint32_t*>(
          iBuf->lock(Ogre::HardwareBuffer::HBL_DISCARD));

      unsigned int j;

      // Add all the vertices
      for (j = 0; j < subMesh.GetVertexCount(); j++)
      {
        *vertices++ = subMesh.GetVertex(j).x;
        *vertices++ = subMesh.GetVertex(j).y;
        *vertices++ = subMesh.GetVertex(j).z;

        if (subMesh.GetNormalCount() > 0)
        {
          *vertices++ = subMesh.GetNormal(j).x;
          *vertices++ = subMesh.GetNormal(j).y;
          *vertices++ = subMesh.GetNormal(j).z;
        }

        if (subMesh.GetTexCoordCount() > 0)
        {
          *vertices++ = subMesh.GetTexCoord(j).x;
          *vertices++ = subMesh.GetTexCoord(j).y;
        }
      }

      // Add all the indices
      for (j = 0; j < subMesh.GetIndexCount(); j++)
        *indices++ = subMesh.GetIndex(j);

      const common::Material *material;
      material = _mesh->GetMaterial(subMesh.GetMaterialIndex());
      if (material)
      {
        rendering::Material::Update(material);
        ogreSubMesh->setMaterialName(material->GetName());
      }
      else
      {
        ogreSubMesh->setMaterialName("Gazebo/White");
      }

      // Unlock
      vBuf->unlock();
      iBuf->unlock();
    }

    math::Vector3 max = _mesh->GetMax();
    math::Vector3 min = _mesh->GetMin();

    if (_mesh->HasSkeleton())
    {
      min = math::Vector3(-1, -1, -1);
      max = math::Vector3(1, 1, 1);
    }

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
    gzerr << "Unable to insert mesh[" << e.getDescription() << "]" << std::endl;
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
    this->SetPose(msgs::Convert(_msg->pose()));

  if (_msg->has_visible())
    this->SetVisible(_msg->visible());

  if (_msg->has_transparency())
    this->SetTransparency(_msg->transparency());

  if (_msg->has_material())
  {
    if (_msg->material().has_script())
    {
      for (int i = 0; i < _msg->material().script().uri_size(); ++i)
      {
        RenderEngine::Instance()->AddResourcePath(
            _msg->material().script().uri(i));
      }
      this->SetMaterial(_msg->material().script().name());
    }

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

  if (_msg->has_scale())
    this->SetScale(msgs::Convert(_msg->scale()));

  // TODO: Make sure this isn't necessary
  if (_msg->has_geometry() && _msg->geometry().has_type())
  {
    math::Vector3 geomScale(1, 1, 1);

    if (_msg->geometry().type() == msgs::Geometry::BOX)
    {
      geomScale = msgs::Convert(_msg->geometry().box().size());
    }
    else if (_msg->geometry().type() == msgs::Geometry::CYLINDER)
    {
      geomScale.x = _msg->geometry().cylinder().radius() * 2.0;
      geomScale.y = _msg->geometry().cylinder().radius() * 2.0;
      geomScale.z = _msg->geometry().cylinder().length();
    }
    else if (_msg->geometry().type() == msgs::Geometry::SPHERE)
    {
      geomScale.x = geomScale.y = geomScale.z
          = _msg->geometry().sphere().radius() * 2.0;
    }
    else if (_msg->geometry().type() == msgs::Geometry::PLANE)
    {
      geomScale.x = geomScale.y = 1.0;
      if (_msg->geometry().plane().has_size())
      {
        geomScale.x = _msg->geometry().plane().size().x();
        geomScale.y = _msg->geometry().plane().size().y();
      }
      geomScale.z = 1.0;
    }
    else if (_msg->geometry().type() == msgs::Geometry::IMAGE)
    {
      geomScale.x = geomScale.y = geomScale.z
          = _msg->geometry().image().scale();
    }
    else if (_msg->geometry().type() == msgs::Geometry::HEIGHTMAP)
      geomScale = msgs::Convert(_msg->geometry().heightmap().size());
    else if (_msg->geometry().type() == msgs::Geometry::MESH)
    {
      if (_msg->geometry().mesh().has_scale())
        geomScale = msgs::Convert(_msg->geometry().mesh().scale());
      else
        geomScale.x = geomScale.y = geomScale.z = 1.0;
    }
    else if (_msg->geometry().type() == msgs::Geometry::EMPTY)
      geomScale.x = geomScale.y = geomScale.z = 1.0;
    else
      gzerr << "Unknown geometry type[" << _msg->geometry().type() << "]\n";

    this->SetScale(geomScale);
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

//////////////////////////////////////////////////
VisualPtr Visual::GetRootVisual()
{
  VisualPtr p = shared_from_this();
  while (p->GetParent() && p->GetParent()->GetName() != "__world_node__")
    p = p->GetParent();

  return p;
}

//////////////////////////////////////////////////
bool Visual::IsPlane() const
{
  if (this->sdf->HasElement("geometry"))
  {
    sdf::ElementPtr geomElem = this->sdf->GetElement("geometry");
    if (geomElem->HasElement("plane"))
      return true;
  }

  std::vector<VisualPtr>::const_iterator iter;
  for (iter = this->children.begin(); iter != this->children.end(); ++iter)
  {
    if ((*iter)->IsPlane())
      return true;
  }

  return false;
}

//////////////////////////////////////////////////
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
    else if (geomElem->HasElement("mesh") || geomElem->HasElement("heightmap"))
    {
      sdf::ElementPtr tmpElem = geomElem->GetElement("mesh");
      std::string filename;

      std::string uri = tmpElem->Get<std::string>("uri");
      if (uri.empty())
      {
        gzerr << "<uri> element missing for geometry element:\n";
        return std::string();
      }

      filename = common::find_file(uri);

      if (filename == "__default__" || filename.empty())
        gzerr << "No mesh specified\n";

      return filename;
    }
  }

  return std::string();
}

//////////////////////////////////////////////////
std::string Visual::GetSubMeshName() const
{
  std::string result;

  if (this->sdf->HasElement("geometry"))
  {
    sdf::ElementPtr geomElem = this->sdf->GetElement("geometry");
    if (geomElem->HasElement("mesh"))
    {
      sdf::ElementPtr tmpElem = geomElem->GetElement("mesh");
      if (tmpElem->HasElement("submesh"))
        result = tmpElem->GetElement("submesh")->Get<std::string>("name");
    }
  }

  return result;
}

//////////////////////////////////////////////////
bool Visual::GetCenterSubMesh() const
{
  bool result = false;

  if (this->sdf->HasElement("geometry"))
  {
    sdf::ElementPtr geomElem = this->sdf->GetElement("geometry");
    if (geomElem->HasElement("mesh"))
    {
      sdf::ElementPtr tmpElem = geomElem->GetElement("mesh");
      if (tmpElem->HasElement("submesh"))
        result = tmpElem->GetElement("submesh")->Get<bool>("center");
    }
  }

  return result;
}

//////////////////////////////////////////////////
void Visual::MoveToPositions(const std::vector<math::Pose> &_pts,
                             double _time,
                             boost::function<void()> _onComplete)
{
  Ogre::TransformKeyFrame *key;
  math::Vector3 start = this->GetWorldPose().pos;

  this->onAnimationComplete = _onComplete;

  std::string animName = this->GetName() + "_animation";

  Ogre::Animation *anim =
    this->sceneNode->getCreator()->createAnimation(animName, _time);
  anim->setInterpolationMode(Ogre::Animation::IM_SPLINE);

  Ogre::NodeAnimationTrack *strack = anim->createNodeTrack(0, this->sceneNode);

  key = strack->createNodeKeyFrame(0);
  key->setTranslate(Ogre::Vector3(start.x, start.y, start.z));
  key->setRotation(this->sceneNode->getOrientation());

  double dt = _time / (_pts.size()-1);
  double tt = 0;
  for (unsigned int i = 0; i < _pts.size(); i++)
  {
    key = strack->createNodeKeyFrame(tt);
    key->setTranslate(Ogre::Vector3(
          _pts[i].pos.x, _pts[i].pos.y, _pts[i].pos.z));
    key->setRotation(Conversions::Convert(_pts[i].rot));

    tt += dt;
  }

  this->animState =
    this->sceneNode->getCreator()->createAnimationState(animName);

  this->animState->setTimePosition(0);
  this->animState->setEnabled(true);
  this->animState->setLoop(false);
  this->prevAnimTime = common::Time::GetWallTime();

  if (!this->preRenderConnection)
    this->preRenderConnection =
      event::Events::ConnectPreRender(boost::bind(&Visual::Update, this));
}

//////////////////////////////////////////////////
void Visual::MoveToPosition(const math::Pose &_pose, double _time)
{
  Ogre::TransformKeyFrame *key;
  math::Vector3 start = this->GetWorldPose().pos;
  math::Vector3 rpy = _pose.rot.GetAsEuler();

  math::Quaternion rotFinal(0, rpy.y, rpy.z);

  std::string animName = this->GetName() + "_animation";

  Ogre::Animation *anim =
    this->sceneNode->getCreator()->createAnimation(animName, _time);
  anim->setInterpolationMode(Ogre::Animation::IM_SPLINE);

  Ogre::NodeAnimationTrack *strack = anim->createNodeTrack(0, this->sceneNode);

  key = strack->createNodeKeyFrame(0);
  key->setTranslate(Ogre::Vector3(start.x, start.y, start.z));
  key->setRotation(this->sceneNode->getOrientation());

  key = strack->createNodeKeyFrame(_time);
  key->setTranslate(Ogre::Vector3(_pose.pos.x, _pose.pos.y, _pose.pos.z));
  key->setRotation(Conversions::Convert(rotFinal));

  this->animState =
    this->sceneNode->getCreator()->createAnimationState(animName);

  this->animState->setTimePosition(0);
  this->animState->setEnabled(true);
  this->animState->setLoop(false);
  this->prevAnimTime = common::Time::GetWallTime();

  this->preRenderConnection =
    event::Events::ConnectPreRender(boost::bind(&Visual::Update, this));
}

//////////////////////////////////////////////////
void Visual::ShowBoundingBox()
{
  this->sceneNode->showBoundingBox(true);
}

//////////////////////////////////////////////////
void Visual::SetScene(ScenePtr _scene)
{
  this->scene = _scene;
}

//////////////////////////////////////////////////
ScenePtr Visual::GetScene() const
{
  return this->scene;
}

//////////////////////////////////////////////////
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

//////////////////////////////////////////////////
void Visual::ShowSkeleton(bool _show)
{
  double transp = 0.0;
  if (_show)
    transp = 0.5;

  ///  make the rest of the model transparent
  this->SetTransparency(transp);

  if (this->GetName().find("__SKELETON_VISUAL__") != std::string::npos)
    this->SetVisible(_show);

  std::vector<VisualPtr>::iterator iter;
  for (iter = this->children.begin(); iter != this->children.end(); ++iter)
  {
    (*iter)->ShowSkeleton(_show);
  }
}

//////////////////////////////////////////////////
void Visual::SetVisibilityFlags(uint32_t _flags)
{
  for (std::vector<VisualPtr>::iterator iter = this->children.begin();
       iter != this->children.end(); ++iter)
  {
    (*iter)->SetVisibilityFlags(_flags);
  }

  for (int i = 0; i < this->sceneNode->numAttachedObjects(); ++i)
  {
    this->sceneNode->getAttachedObject(i)->setVisibilityFlags(_flags);
  }

  for (unsigned int i = 0; i < this->sceneNode->numChildren(); ++i)
  {
    Ogre::SceneNode *sn = (Ogre::SceneNode*)(this->sceneNode->getChild(i));

    for (int j = 0; j < sn->numAttachedObjects(); ++j)
      sn->getAttachedObject(j)->setVisibilityFlags(_flags);
  }
}

//////////////////////////////////////////////////
void Visual::ShowJoints(bool _show)
{
  if (this->GetName().find("JOINT_VISUAL__") != std::string::npos)
    this->SetVisible(_show);

  std::vector<VisualPtr>::iterator iter;
  for (iter = this->children.begin(); iter != this->children.end(); ++iter)
  {
    (*iter)->ShowJoints(_show);
  }
}

//////////////////////////////////////////////////
void Visual::ShowCOM(bool _show)
{
  if (this->GetName().find("COM_VISUAL__") != std::string::npos)
    this->SetVisible(_show);

  std::vector<VisualPtr>::iterator iter;
  for (iter = this->children.begin(); iter != this->children.end(); ++iter)
  {
    (*iter)->ShowCOM(_show);
  }
}

//////////////////////////////////////////////////
void Visual::SetSkeletonPose(const msgs::PoseAnimation &_pose)
{
  if (!this->skeleton)
  {
    gzerr << "Visual " << this->GetName() << " has no skeleton.\n";
    return;
  }

  for (int i = 0; i < _pose.pose_size(); i++)
  {
    const msgs::Pose& bonePose = _pose.pose(i);
    if (!this->skeleton->hasBone(bonePose.name()))
      continue;
    Ogre::Bone *bone = this->skeleton->getBone(bonePose.name());
    Ogre::Vector3 p(bonePose.position().x(),
                    bonePose.position().y(),
                    bonePose.position().z());
    Ogre::Quaternion quat(Ogre::Quaternion(bonePose.orientation().w(),
                                           bonePose.orientation().x(),
                                           bonePose.orientation().y(),
                                           bonePose.orientation().z()));

    bone->setManuallyControlled(true);
    bone->setPosition(p);
    bone->setOrientation(quat);
  }
}


//////////////////////////////////////////////////
void Visual::LoadPlugins()
{
  if (this->sdf->HasElement("plugin"))
  {
    sdf::ElementPtr pluginElem = this->sdf->GetElement("plugin");
    while (pluginElem)
    {
      this->LoadPlugin(pluginElem);
      pluginElem = pluginElem->GetNextElement("plugin");
    }
  }


  for (std::vector<VisualPluginPtr>::iterator iter = this->plugins.begin();
       iter != this->plugins.end(); ++iter)
  {
    (*iter)->Init();
  }
}

//////////////////////////////////////////////////
void Visual::LoadPlugin(const std::string &_filename,
                       const std::string &_name,
                       sdf::ElementPtr _sdf)
{
  gazebo::VisualPluginPtr plugin = gazebo::VisualPlugin::Create(_filename,
                                                              _name);

  if (plugin)
  {
    if (plugin->GetType() != VISUAL_PLUGIN)
    {
      gzerr << "Visual[" << this->GetName() << "] is attempting to load "
            << "a plugin, but detected an incorrect plugin type. "
            << "Plugin filename[" << _filename << "] name[" << _name << "]\n";
      return;
    }
    plugin->Load(shared_from_this(), _sdf);
    this->plugins.push_back(plugin);

    if (this->initialized)
      plugin->Init();
  }
}

//////////////////////////////////////////////////
void Visual::RemovePlugin(const std::string &_name)
{
  std::vector<VisualPluginPtr>::iterator iter;
  for (iter = this->plugins.begin(); iter != this->plugins.end(); ++iter)
  {
    if ((*iter)->GetHandle() == _name)
    {
      this->plugins.erase(iter);
      break;
    }
  }
}

//////////////////////////////////////////////////
void Visual::LoadPlugin(sdf::ElementPtr _sdf)
{
  std::string pluginName = _sdf->Get<std::string>("name");
  std::string filename = _sdf->Get<std::string>("filename");
  this->LoadPlugin(filename, pluginName, _sdf);
}

//////////////////////////////////////////////////
uint32_t Visual::GetId() const
{
  return this->id;
}

//////////////////////////////////////////////////
void Visual::SetId(uint32_t _id)
{
  this->id = _id;
}
