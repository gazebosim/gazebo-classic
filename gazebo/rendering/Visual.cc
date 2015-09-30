/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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

#include "gazebo/msgs/msgs.hh"
#include "gazebo/math/Vector2d.hh"
#include "gazebo/common/Assert.hh"
#include "gazebo/common/Event.hh"
#include "gazebo/common/Events.hh"
#include "gazebo/common/CommonIface.hh"
#include "gazebo/common/MeshManager.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Exception.hh"
#include "gazebo/common/Mesh.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/common/Skeleton.hh"
#include "gazebo/rendering/RenderEvents.hh"
#include "gazebo/rendering/WireBox.hh"
#include "gazebo/rendering/Conversions.hh"
#include "gazebo/rendering/DynamicLines.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/RTShaderSystem.hh"
#include "gazebo/rendering/RenderEngine.hh"
#include "gazebo/rendering/Material.hh"
#include "gazebo/rendering/MovableText.hh"
#include "gazebo/rendering/VisualPrivate.hh"
#include "gazebo/rendering/Visual.hh"

using namespace gazebo;
using namespace rendering;

// Note: The value of GZ_UINT32_MAX is reserved as a flag.
uint32_t VisualPrivate::visualIdCount = GZ_UINT32_MAX - 1;

//////////////////////////////////////////////////
Visual::Visual(const std::string &_name, VisualPtr _parent, bool _useRTShader)
  : dataPtr(new VisualPrivate)
{
  this->Init(_name, _parent, _useRTShader);
}

//////////////////////////////////////////////////
Visual::Visual(const std::string &_name, ScenePtr _scene, bool _useRTShader)
  : dataPtr(new VisualPrivate)
{
  this->Init(_name, _scene, _useRTShader);
}

//////////////////////////////////////////////////
Visual::Visual(VisualPrivate &_dataPtr, const std::string &_name,
    VisualPtr _parent, bool _useRTShader)
    : dataPtr(&_dataPtr)
{
  this->Init(_name, _parent, _useRTShader);
}

//////////////////////////////////////////////////
Visual::Visual(VisualPrivate &_dataPtr, const std::string &_name,
    ScenePtr _scene,  bool _useRTShader)
    : dataPtr(&_dataPtr)
{
  this->Init(_name, _scene, _useRTShader);
}

//////////////////////////////////////////////////
void Visual::Init(const std::string &_name, ScenePtr _scene,
    bool _useRTShader)
{
  this->dataPtr->id = this->dataPtr->visualIdCount--;
  this->dataPtr->boundingBox = NULL;
  this->dataPtr->useRTShader = _useRTShader;
  this->dataPtr->visibilityFlags = GZ_VISIBILITY_ALL;

  this->dataPtr->sdf.reset(new sdf::Element);
  sdf::initFile("visual.sdf", this->dataPtr->sdf);

  this->SetName(_name);
  this->dataPtr->sceneNode = NULL;
  this->dataPtr->animState = NULL;
  this->dataPtr->skeleton = NULL;
  this->dataPtr->initialized = false;
  this->dataPtr->lighting = true;
  this->dataPtr->castShadows = true;
  this->dataPtr->visible = true;
  this->dataPtr->layer = -1;

  std::string uniqueName = this->GetName();
  int index = 0;
  while (_scene->GetManager()->hasSceneNode(uniqueName))
  {
    uniqueName = this->GetName() + "_" +
                 boost::lexical_cast<std::string>(index++);
  }

  this->dataPtr->scene = _scene;
  this->SetName(uniqueName);
  this->dataPtr->sceneNode =
    this->dataPtr->scene->GetManager()->getRootSceneNode()->
        createChildSceneNode(this->GetName());

  this->Init();
}

//////////////////////////////////////////////////
void Visual::Init(const std::string &_name, VisualPtr _parent,
    bool _useRTShader)
{
  this->dataPtr->id = this->dataPtr->visualIdCount--;
  this->dataPtr->boundingBox = NULL;
  this->dataPtr->useRTShader = _useRTShader;

  this->dataPtr->sdf.reset(new sdf::Element);
  sdf::initFile("visual.sdf", this->dataPtr->sdf);

  this->SetName(_name);
  this->dataPtr->sceneNode = NULL;
  this->dataPtr->animState = NULL;
  this->dataPtr->initialized = false;
  this->dataPtr->lighting = true;
  this->dataPtr->castShadows = true;
  this->dataPtr->visible = true;
  this->dataPtr->layer = -1;

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

  this->dataPtr->sceneNode = pnode->createChildSceneNode(this->GetName());

  this->dataPtr->parent = _parent;
  this->dataPtr->scene = this->dataPtr->parent->GetScene();
  this->Init();
}

//////////////////////////////////////////////////
Visual::~Visual()
{
  if (this->dataPtr->preRenderConnection)
    event::Events::DisconnectPreRender(this->dataPtr->preRenderConnection);

  delete this->dataPtr->boundingBox;

  // delete instance from lines vector
  /*for (std::list<DynamicLines*>::iterator iter = this->dataPtr->lines.begin();
       iter != this->dataPtr->lines.end(); ++iter)
    delete *iter;
    */
  this->dataPtr->lines.clear();

  if (this->dataPtr->sceneNode != NULL)
  {
    this->DestroyAllAttachedMovableObjects(this->dataPtr->sceneNode);
    this->dataPtr->sceneNode->removeAndDestroyAllChildren();
    this->dataPtr->scene->GetManager()->destroySceneNode(
        this->dataPtr->sceneNode->getName());
    this->dataPtr->sceneNode = NULL;
  }

  this->dataPtr->scene.reset();
  this->dataPtr->sdf->Reset();
  this->dataPtr->sdf.reset();
  this->dataPtr->parent.reset();
  this->dataPtr->children.clear();

  delete this->dataPtr;
  this->dataPtr = 0;
}

/////////////////////////////////////////////////
void Visual::Fini()
{
  this->dataPtr->plugins.clear();

  // Detach from the parent
  if (this->dataPtr->parent)
    this->dataPtr->parent->DetachVisual(this->GetName());

  if (this->dataPtr->sceneNode)
  {
    this->dataPtr->sceneNode->detachAllObjects();
    this->dataPtr->scene->GetManager()->destroySceneNode(
        this->dataPtr->sceneNode);
    this->dataPtr->sceneNode = NULL;
  }

  if (this->dataPtr->preRenderConnection)
  {
    event::Events::DisconnectPreRender(this->dataPtr->preRenderConnection);
    this->dataPtr->preRenderConnection.reset();
  }

  this->dataPtr->scene.reset();
}

/////////////////////////////////////////////////
VisualPtr Visual::Clone(const std::string &_name, VisualPtr _newParent)
{
  VisualPtr result(new Visual(_name, _newParent));
  result->Load(this->dataPtr->sdf);
  result->SetScale(this->dataPtr->scale);
  for (auto iter: this->dataPtr->children)
  {
    iter->Clone(iter->GetName(), result);
  }

  if (_newParent == this->dataPtr->scene->GetWorldVisual())
    result->SetWorldPose(this->GetWorldPose());
  result->ShowCollision(false);

  result->SetName(_name);
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
      this->dataPtr->scene->GetManager()->destroyEntity(ent);
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
  this->dataPtr->type = VT_ENTITY;
  this->dataPtr->transparency = 0.0;
  this->dataPtr->isStatic = false;
  this->dataPtr->visible = true;
  this->dataPtr->ribbonTrail = NULL;
  this->dataPtr->staticGeom = NULL;
  this->dataPtr->layer = -1;
  this->dataPtr->scale = ignition::math::Vector3d::One;

  this->dataPtr->initialized = true;
}

//////////////////////////////////////////////////
void Visual::LoadFromMsg(const boost::shared_ptr< msgs::Visual const> &_msg)
{
  this->dataPtr->sdf = msgs::VisualToSDF(*_msg.get());
  this->Load();
  this->UpdateFromMsg(_msg);
}

//////////////////////////////////////////////////
void Visual::Load(sdf::ElementPtr _sdf)
{
  this->dataPtr->sdf->Copy(_sdf);
  this->Load();
}

//////////////////////////////////////////////////
void Visual::Load()
{
  std::ostringstream stream;
  math::Pose pose;
  Ogre::Vector3 meshSize(1, 1, 1);
  Ogre::MovableObject *obj = NULL;

  if (this->dataPtr->parent)
    this->dataPtr->parent->AttachVisual(shared_from_this());

  // Read the desired position and rotation of the mesh
  pose = this->dataPtr->sdf->Get<math::Pose>("pose");

  std::string mesh = this->GetMeshName();
  std::string subMesh = this->GetSubMeshName();
  bool centerSubMesh = this->GetCenterSubMesh();

  if (!mesh.empty())
  {
    try
    {
      // Create the visual
      stream << "VISUAL_" << this->dataPtr->sceneNode->getName();
      obj = this->AttachMesh(mesh, subMesh, centerSubMesh,
          stream.str());
    }
    catch(Ogre::Exception &e)
    {
      gzerr << "Ogre Error:" << e.getFullDescription() << "\n";
      gzerr << "Unable to create a mesh from " <<  mesh << "\n";
      return;
    }
  }

  Ogre::Entity *ent = static_cast<Ogre::Entity *>(obj);
  if (ent)
  {
    if (ent->hasSkeleton())
      this->dataPtr->skeleton = ent->getSkeleton();

    for (unsigned int i = 0; i < ent->getNumSubEntities(); i++)
    {
      ent->getSubEntity(i)->setCustomParameter(1, Ogre::Vector4(
          this->dataPtr->sdf->Get<double>("laser_retro"), 0.0, 0.0, 0.0));
    }
  }

  // Set the pose of the scene node
  this->SetPose(pose);

  // Get the size of the mesh
  if (obj)
    meshSize = obj->getBoundingBox().getSize();

  if (this->dataPtr->sdf->HasElement("geometry"))
  {
    sdf::ElementPtr geomElem = this->dataPtr->sdf->GetElement("geometry");

    ignition::math::Vector3d geometrySize;
    bool hasGeom = true;
    if (geomElem->HasElement("box"))
    {
      geometrySize =
          geomElem->GetElement("box")->Get<ignition::math::Vector3d>("size");
    }
    else if (geomElem->HasElement("sphere"))
    {
      double r = geomElem->GetElement("sphere")->Get<double>("radius");
      geometrySize.Set(r * 2.0, r * 2.0, r * 2.0);
    }
    else if (geomElem->HasElement("cylinder"))
    {
      double r = geomElem->GetElement("cylinder")->Get<double>("radius");
      double l = geomElem->GetElement("cylinder")->Get<double>("length");
      geometrySize.Set(r * 2.0, r * 2.0, l);
    }
    else if (geomElem->HasElement("plane"))
    {
      math::Vector2d size =
        geomElem->GetElement("plane")->Get<math::Vector2d>("size");
      geometrySize.Set(size.x, size.y, 1);
    }
    else if (geomElem->HasElement("mesh"))
    {
      geometrySize =
          geomElem->GetElement("mesh")->Get<ignition::math::Vector3d>("scale");
    }
    else
    {
      hasGeom = false;
    }

    if (hasGeom)
    {
      // geom values give the absolute size so compute a scale that will
      // be mulitiply by the current scale to get to the geom size.
      ignition::math::Vector3d derivedScale = this->DerivedScale();
      ignition::math::Vector3d localScale =
          geometrySize / (derivedScale / this->dataPtr->scale);
      this->dataPtr->sceneNode->setScale(
          Conversions::Convert(math::Vector3(localScale)));
      this->dataPtr->scale = localScale;
      this->dataPtr->geomSize = geometrySize;
    }
  }

  // Set the material of the mesh
  if (this->dataPtr->sdf->HasElement("material"))
  {
    sdf::ElementPtr matElem =
        this->dataPtr->sdf->GetElement("material");

    // clone the material sdf to preserve the new values to be set
    // as updating the material name via SetMaterial can affect the
    // ambient/diffuse/specular/emissive color sdf elements.
    sdf::ElementPtr matElemClone = matElem->Clone();

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

    if (matElemClone->HasElement("ambient"))
      this->SetAmbient(matElemClone->Get<common::Color>("ambient"));
    if (matElemClone->HasElement("diffuse"))
      this->SetDiffuse(matElemClone->Get<common::Color>("diffuse"));
    if (matElemClone->HasElement("specular"))
      this->SetSpecular(matElemClone->Get<common::Color>("specular"));
    if (matElemClone->HasElement("emissive"))
      this->SetEmissive(matElemClone->Get<common::Color>("emissive"));

    if (matElem->HasElement("lighting"))
    {
      this->SetLighting(matElem->Get<bool>("lighting"));
    }
  }

  if (this->dataPtr->sdf->HasElement("transparency"))
  {
    this->SetTransparency(this->dataPtr->sdf->Get<float>("transparency"));
  }

  // Allow the mesh to cast shadows
  this->SetCastShadows(this->dataPtr->sdf->Get<bool>("cast_shadows"));
  this->LoadPlugins();
  this->dataPtr->scene->AddVisual(shared_from_this());

  // Set meta information
  if (this->dataPtr->sdf->HasElement("meta"))
  {
    if (this->dataPtr->sdf->GetElement("meta")->HasElement("layer"))
    {
      this->dataPtr->layer =
        this->dataPtr->sdf->GetElement("meta")->Get<int32_t>("layer");
      rendering::Events::newLayer(this->dataPtr->layer);
    }
  }
}

//////////////////////////////////////////////////
void Visual::Update()
{
  if (!this->dataPtr->visible)
    return;

  std::list<DynamicLines*>::iterator iter;

  // Update the lines
  for (iter = this->dataPtr->lines.begin(); iter != this->dataPtr->lines.end();
      ++iter)
  {
    (*iter)->Update();
  }

  std::list< std::pair<DynamicLines*, unsigned int> >::iterator liter;
  for (liter = this->dataPtr->lineVertices.begin();
       liter != this->dataPtr->lineVertices.end(); ++liter)
  {
    liter->first->SetPoint(liter->second,
        Conversions::Convert(this->dataPtr->sceneNode->_getDerivedPosition()));
    liter->first->Update();
  }

  if (this->dataPtr->animState)
  {
    this->dataPtr->animState->addTime(
        (common::Time::GetWallTime() - this->dataPtr->prevAnimTime).Double());
    this->dataPtr->prevAnimTime = common::Time::GetWallTime();
    if (this->dataPtr->animState->hasEnded())
    {
      this->dataPtr->animState = NULL;
      this->dataPtr->sceneNode->getCreator()->destroyAnimation(
          this->GetName() + "_animation");
      if (this->dataPtr->onAnimationComplete)
        this->dataPtr->onAnimationComplete();
      // event::Events::DisconnectPreRender(this->preRenderConnection);
    }
  }
}

//////////////////////////////////////////////////
void Visual::SetName(const std::string &_name)
{
  this->dataPtr->name = _name;
  this->dataPtr->sdf->GetAttribute("name")->Set(_name);
}

//////////////////////////////////////////////////
std::string Visual::GetName() const
{
  return this->dataPtr->name;
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
    this->dataPtr->sceneNode->addChild(_vis->GetSceneNode());
    this->dataPtr->children.push_back(_vis);
    _vis->dataPtr->parent = shared_from_this();
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
  for (iter = this->dataPtr->children.begin();
      iter != this->dataPtr->children.end(); ++iter)
  {
    if ((*iter)->GetName() == _name)
    {
      VisualPtr childVis = (*iter);
      this->dataPtr->children.erase(iter);
      if (this->dataPtr->sceneNode)
        this->dataPtr->sceneNode->removeChild(childVis->GetSceneNode());
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
  // if (this->dataPtr->sdf->HasElement("geometry"))
  // if (this->dataPtr->sdf->GetElement("geometry")->HasElement("plane"))
  // _obj->setRenderQueueGroup(Ogre::RENDER_QUEUE_SKIES_EARLY+1);

  if (!this->HasAttachedObject(_obj->getName()))
  {
    // update to use unique materials
    Ogre::Entity *entity = dynamic_cast<Ogre::Entity *>(_obj);
    if (entity)
    {
      for (unsigned j = 0; j < entity->getNumSubEntities(); ++j)
      {
        Ogre::SubEntity *subEntity = entity->getSubEntity(j);
        Ogre::MaterialPtr material = subEntity->getMaterial();
        if (!material.isNull() &&
            material->getName().find("_MATERIAL_") == std::string::npos)
        {
          std::string newMaterialName;
          newMaterialName = this->dataPtr->sceneNode->getName() +
              "_MATERIAL_" + material->getName();
          material = material->clone(newMaterialName);
          subEntity->setMaterial(material);
        }
      }
    }

    this->dataPtr->sceneNode->attachObject(_obj);
    if (this->dataPtr->useRTShader && this->dataPtr->scene->GetInitialized() &&
      _obj->getName().find("__COLLISION_VISUAL__") == std::string::npos)
    {
      RTShaderSystem::Instance()->UpdateShaders();
    }
    _obj->getUserObjectBindings().setUserAny(Ogre::Any(this->GetName()));
  }
  else
    gzerr << "Visual[" << this->GetName() << "] already has object["
          << _obj->getName() << "] attached.";

  _obj->setVisibilityFlags(GZ_VISIBILITY_ALL);
}

//////////////////////////////////////////////////
bool Visual::HasAttachedObject(const std::string &_name)
{
  for (unsigned int i = 0; i < this->dataPtr->sceneNode->numAttachedObjects();
      ++i)
  {
    if (this->dataPtr->sceneNode->getAttachedObject(i)->getName() == _name)
      return true;
  }

  return false;
}

//////////////////////////////////////////////////
unsigned int Visual::GetAttachedObjectCount() const
{
  return this->dataPtr->sceneNode->numAttachedObjects();
}

//////////////////////////////////////////////////
void Visual::DetachObjects()
{
  if (this->dataPtr->sceneNode)
    this->dataPtr->sceneNode->detachAllObjects();
  this->dataPtr->meshName = "";
  this->dataPtr->subMeshName = "";
  this->dataPtr->myMaterialName = "";
}

//////////////////////////////////////////////////
unsigned int Visual::GetChildCount()
{
  return this->dataPtr->children.size();
}

//////////////////////////////////////////////////
VisualPtr Visual::GetChild(unsigned int _num)
{
  if (_num < this->dataPtr->children.size())
    return this->dataPtr->children[_num];
  return VisualPtr();
}

//////////////////////////////////////////////////
void Visual::MakeStatic()
{
  /*if (!this->staticGeom)
    this->staticGeom =
    this->dataPtr->sceneNode->getCreator()->createStaticGeometry(
    this->dataPtr->sceneNode->getName() + "_Static");

  // Add the scene node to the static geometry
  this->staticGeom->addSceneNode(this->dataPtr->sceneNode);

  // Build the static geometry
  this->staticGeom->build();

  // Prevent double rendering
  this->dataPtr->sceneNode->setVisible(false);
  this->dataPtr->sceneNode->detachAllObjects();
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

  this->dataPtr->meshName = _meshName;
  this->dataPtr->subMeshName = _subMesh;

  Ogre::MovableObject *obj;
  std::string objName = _objName;
  std::string meshName = _meshName;
  meshName += _subMesh.empty() ? "" : "::" + _subMesh;

  if (objName.empty())
    objName = this->dataPtr->sceneNode->getName() + "_ENTITY_" + meshName;

  this->InsertMesh(_meshName, _subMesh, _centerSubmesh);

  if (this->dataPtr->sceneNode->getCreator()->hasEntity(objName))
  {
    obj = (Ogre::MovableObject*)
      (this->dataPtr->sceneNode->getCreator()->getEntity(objName));
  }
  else
  {
    obj = (Ogre::MovableObject*)
        (this->dataPtr->sceneNode->getCreator()->createEntity(objName,
        meshName));
  }

  this->AttachObject(obj);
  return obj;
}

//////////////////////////////////////////////////
void Visual::SetScale(const math::Vector3 &_scale)
{
  if (this->dataPtr->scale == _scale.Ign())
    return;

  // update geom size based on scale.
  this->UpdateGeomSize(
      this->DerivedScale() / this->dataPtr->scale * _scale.Ign());

  this->dataPtr->scale = _scale.Ign();

  this->dataPtr->sceneNode->setScale(
      Conversions::Convert(math::Vector3(this->dataPtr->scale)));
}

//////////////////////////////////////////////////
void Visual::UpdateGeomSize(const ignition::math::Vector3d &_scale)
{
  for (std::vector<VisualPtr>::iterator iter = this->dataPtr->children.begin();
       iter != this->dataPtr->children.end(); ++iter)
  {
    (*iter)->UpdateGeomSize(_scale * (*iter)->GetScale().Ign());
  }

  // update the same way as server - see Link::UpdateVisualGeomSDF()
  if (!this->dataPtr->sdf->HasElement("geometry"))
    return;

  sdf::ElementPtr geomElem = this->dataPtr->sdf->GetElement("geometry");
  if (geomElem->HasElement("box"))
  {
    ignition::math::Vector3d size =
        geomElem->GetElement("box")->Get<ignition::math::Vector3d>("size");
    ignition::math::Vector3d geomBoxSize = _scale/this->dataPtr->geomSize*size;

    geomElem->GetElement("box")->GetElement("size")->Set(
        geomBoxSize);
    this->dataPtr->geomSize = geomBoxSize;
  }
  else if (geomElem->HasElement("sphere"))
  {
    // update radius the same way as collision shapes
    double radius = geomElem->GetElement("sphere")->Get<double>("radius");
    double newRadius = _scale.Max();
    double oldRadius = this->dataPtr->geomSize.Max();
    double geomRadius = newRadius/oldRadius*radius;
    geomElem->GetElement("sphere")->GetElement("radius")->Set(geomRadius);
    this->dataPtr->geomSize = ignition::math::Vector3d(
        geomRadius*2.0, geomRadius*2.0, geomRadius*2.0);
  }
  else if (geomElem->HasElement("cylinder"))
  {
    // update radius the same way as collision shapes
    double radius = geomElem->GetElement("cylinder")->Get<double>("radius");
    double newRadius = std::max(_scale.X(), _scale.Y());
    double oldRadius = std::max(this->dataPtr->geomSize.X(),
        this->dataPtr->geomSize.Y());
    double length = geomElem->GetElement("cylinder")->Get<double>("length");
    double geomRadius = newRadius/oldRadius*radius;
    double geomLength = _scale.Z()/this->dataPtr->geomSize.Z()*length;
    geomElem->GetElement("cylinder")->GetElement("radius")->Set(
        geomRadius);
    geomElem->GetElement("cylinder")->GetElement("length")->Set(
        geomLength);

    this->dataPtr->geomSize =
        ignition::math::Vector3d(geomRadius*2.0, geomRadius*2.0, geomLength);
  }
  else if (geomElem->HasElement("mesh"))
  {
    geomElem->GetElement("mesh")->GetElement("scale")->Set(_scale);
    this->dataPtr->geomSize = _scale;
  }
}

/////////////////////////////////////////////////
ignition::math::Vector3d Visual::GetGeometrySize() const
{
  return this->dataPtr->geomSize;
}

//////////////////////////////////////////////////
math::Vector3 Visual::GetScale()
{
  return this->dataPtr->scale;
}

//////////////////////////////////////////////////
ignition::math::Vector3d Visual::DerivedScale() const
{
  ignition::math::Vector3d derivedScale = this->dataPtr->scale;

  VisualPtr worldVis = this->dataPtr->scene->GetWorldVisual();
  VisualPtr vis = this->GetParent();

  while (vis && vis != worldVis)
  {
    derivedScale = derivedScale * vis->GetScale().Ign();
    vis = vis->GetParent();
  }

  return derivedScale;
}

//////////////////////////////////////////////////
void Visual::SetLighting(bool _lighting)
{
  if (this->dataPtr->lighting == _lighting)
    return;

  this->dataPtr->lighting = _lighting;

  try
  {
    for (int i = 0; i < this->dataPtr->sceneNode->numAttachedObjects(); ++i)
    {
      Ogre::MovableObject *obj = this->dataPtr->sceneNode->getAttachedObject(i);

      Ogre::Entity *entity = dynamic_cast<Ogre::Entity*>(obj);
      if (entity)
      {
        for (unsigned j = 0; j < entity->getNumSubEntities(); ++j)
        {
          Ogre::MaterialPtr mat = entity->getSubEntity(j)->getMaterial();
          if (!mat.isNull())
          {
            mat->setLightingEnabled(this->dataPtr->lighting);
          }
        }
      }
    }

    // Apply lighting to all child scene nodes
    for (unsigned int i = 0; i < this->dataPtr->sceneNode->numChildren(); ++i)
    {
      Ogre::SceneNode *sn = dynamic_cast<Ogre::SceneNode *>(
          this->dataPtr->sceneNode->getChild(i));
      for (int j = 0; j < sn->numAttachedObjects(); j++)
      {
        Ogre::MovableObject *obj = sn->getAttachedObject(j);

        Ogre::Entity *entity = dynamic_cast<Ogre::Entity*>(obj);
        if (entity)
        {
          for (unsigned k = 0; k < entity->getNumSubEntities(); ++k)
          {
            Ogre::MaterialPtr mat = entity->getSubEntity(k)->getMaterial();
            if (!mat.isNull())
            {
              mat->setLightingEnabled(this->dataPtr->lighting);
            }
          }
        }
      }
    }
  }
  catch(Ogre::Exception &e)
  {
    gzwarn << "Unable to set lighting to Geometry["
           << this->dataPtr->sceneNode->getName() << ".\n";
  }

  // Apply lighting to all child visuals
  for (std::vector<VisualPtr>::iterator iter = this->dataPtr->children.begin();
       iter != this->dataPtr->children.end(); ++iter)
  {
    (*iter)->SetLighting(this->dataPtr->lighting);
  }

  this->dataPtr->sdf->GetElement("material")
      ->GetElement("lighting")->Set(this->dataPtr->lighting);
}

//////////////////////////////////////////////////
bool Visual::GetLighting() const
{
  return this->dataPtr->lighting;
}

//////////////////////////////////////////////////
void Visual::SetMaterial(const std::string &_materialName, bool _unique,
    const bool _cascade)
{
  if (_materialName.empty() || _materialName == "__default__")
    return;

  common::Color matAmbient;
  common::Color matDiffuse;
  common::Color matSpecular;
  common::Color matEmissive;
  bool matColor = rendering::Material::GetMaterialAsColor(
      _materialName, matAmbient, matDiffuse, matSpecular, matEmissive);

  if (_unique)
  {
    // Create a custom material name
    std::string newMaterialName;
    newMaterialName = this->dataPtr->sceneNode->getName() + "_MATERIAL_" +
        _materialName;

    if (this->GetMaterialName() == newMaterialName &&
        matAmbient == this->GetAmbient() &&
        matDiffuse == this->GetDiffuse() &&
        matSpecular == this->GetSpecular() &&
        matEmissive == this->GetEmissive())
      return;

    this->dataPtr->myMaterialName = newMaterialName;

    Ogre::MaterialPtr origMaterial;
    try
    {
      this->dataPtr->origMaterialName = _materialName;
      // Get the original material
      origMaterial =
        Ogre::MaterialManager::getSingleton().getByName(_materialName);
    }
    catch(Ogre::Exception &e)
    {
      gzwarn << "Unable to get Material[" << _materialName << "] for Geometry["
          << this->dataPtr->sceneNode->getName()
          << ". Object will appear white.\n";
      return;
    }

    if (origMaterial.isNull())
    {
      gzwarn << "Unable to get Material[" << _materialName << "] for Geometry["
        << this->dataPtr->sceneNode->getName()
        << ". Object will appear white\n";
      return;
    }

    Ogre::MaterialPtr myMaterial;

    // Clone the material. This will allow us to change the look of each geom
    // individually.
    if (Ogre::MaterialManager::getSingleton().resourceExists(
          this->dataPtr->myMaterialName))
    {
      myMaterial =
        (Ogre::MaterialPtr)(Ogre::MaterialManager::getSingleton().getByName(
              this->dataPtr->myMaterialName));
    }
    else
    {
      myMaterial = origMaterial->clone(this->dataPtr->myMaterialName);
    }
  }
  else
  {
    if ( this->dataPtr->myMaterialName == _materialName)
      return;
    this->dataPtr->myMaterialName = _materialName;
  }

  try
  {
    for (unsigned int i = 0;
        i < this->dataPtr->sceneNode->numAttachedObjects(); ++i)
    {
      Ogre::MovableObject *obj = this->dataPtr->sceneNode->getAttachedObject(i);
      Ogre::Entity *entity = dynamic_cast<Ogre::Entity *>(obj);
      if (entity)
        entity->setMaterialName(this->dataPtr->myMaterialName);
      else
      {
        Ogre::SimpleRenderable *simpleRenderable =
            dynamic_cast<Ogre::SimpleRenderable *>(obj);
        if (simpleRenderable)
          simpleRenderable->setMaterial(this->dataPtr->myMaterialName);
      }
    }

    // Apply material to all child scene nodes
    for (unsigned int i = 0; i < this->dataPtr->sceneNode->numChildren(); ++i)
    {
      Ogre::SceneNode *sn = dynamic_cast<Ogre::SceneNode *>(
          this->dataPtr->sceneNode->getChild(i));
      for (int j = 0; j < sn->numAttachedObjects(); ++j)
      {
        Ogre::MovableObject *obj = sn->getAttachedObject(j);

        MovableText *text = dynamic_cast<MovableText *>(obj);
        if (text)
        {
          common::Color ambient, diffuse, specular, emissive;
          bool matFound = rendering::Material::GetMaterialAsColor(
              this->dataPtr->myMaterialName, ambient, diffuse, specular,
              emissive);

          if (matFound)
          {
            text->SetColor(ambient);
          }
        }
        else if (dynamic_cast<Ogre::Entity *>(obj))
          ((Ogre::Entity *)obj)->setMaterialName(this->dataPtr->myMaterialName);
        else
        {
          ((Ogre::SimpleRenderable *)obj)->setMaterial(
              this->dataPtr->myMaterialName);
        }
      }
    }
  }
  catch(Ogre::Exception &e)
  {
    gzwarn << "Unable to set Material[" << this->dataPtr->myMaterialName
           << "] to Geometry["
           << this->dataPtr->sceneNode->getName()
           << ". Object will appear white.\n";
  }

  // check if material has color components, if so, set them.
  if (matColor)
  {
    this->SetAmbient(matAmbient, false);
    this->SetDiffuse(matDiffuse, false);
    this->SetSpecular(matSpecular, false);
    this->SetEmissive(matEmissive, false);
  }

  // Re-apply the transparency filter for the last known transparency value
  this->SetTransparencyInnerLoop(this->dataPtr->sceneNode);

  // Apply material to all child visuals
  if (_cascade)
  {
    for (auto &child : this->dataPtr->children)
    {
      child->SetMaterial(_materialName, _unique, _cascade);
    }
  }

  if (this->dataPtr->useRTShader && this->dataPtr->scene->GetInitialized()
      && this->dataPtr->lighting &&
      this->GetName().find("__COLLISION_VISUAL__") == std::string::npos)
  {
    RTShaderSystem::Instance()->UpdateShaders();
  }

  this->dataPtr->sdf->GetElement("material")->GetElement("script")
      ->GetElement("name")->Set(_materialName);
}

/////////////////////////////////////////////////
void Visual::SetAmbient(const common::Color &_color, const bool _cascade)
{
  if (!this->dataPtr->lighting)
    return;

  if (this->dataPtr->myMaterialName.empty())
  {
    std::string matName = this->GetName() + "_MATERIAL_";
    Ogre::MaterialManager::getSingleton().create(matName, "General");
    this->SetMaterial(matName);
  }

  for (unsigned int i = 0; i < this->dataPtr->sceneNode->numAttachedObjects();
      ++i)
  {
    Ogre::Entity *entity = NULL;
    Ogre::MovableObject *obj = this->dataPtr->sceneNode->getAttachedObject(i);

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

  if (_cascade)
  {
    for (auto &child : this->dataPtr->children)
    {
      child->SetAmbient(_color, _cascade);
    }
  }

  this->dataPtr->ambient = _color;

  this->dataPtr->sdf->GetElement("material")
      ->GetElement("ambient")->Set(_color);
}

/////////////////////////////////////////////////
void Visual::SetDiffuse(const common::Color &_color, const bool _cascade)
{
  if (!this->dataPtr->lighting)
    return;

  if (this->dataPtr->myMaterialName.empty())
  {
    std::string matName = this->GetName() + "_MATERIAL_";
    Ogre::MaterialManager::getSingleton().create(matName, "General");
    this->SetMaterial(matName);
  }

  for (unsigned int i = 0; i < this->dataPtr->sceneNode->numAttachedObjects();
      i++)
  {
    Ogre::Entity *entity = NULL;
    Ogre::MovableObject *obj = this->dataPtr->sceneNode->getAttachedObject(i);

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
          dc = Conversions::Convert(_color);
          pass->setDiffuse(dc);
          this->dataPtr->transparency = 1.0f - dc.a;
        }
      }
    }
  }

  if (_cascade)
  {
    for (auto &child : this->dataPtr->children)
    {
      child->SetDiffuse(_color, _cascade);
    }
  }

  this->dataPtr->diffuse = _color;

  this->dataPtr->sdf->GetElement("material")
      ->GetElement("diffuse")->Set(_color);
}

/////////////////////////////////////////////////
void Visual::SetSpecular(const common::Color &_color, const bool _cascade)
{
  if (!this->dataPtr->lighting)
    return;

  if (this->dataPtr->myMaterialName.empty())
  {
    std::string matName = this->GetName() + "_MATERIAL_";
    Ogre::MaterialManager::getSingleton().create(matName, "General");
    this->SetMaterial(matName);
  }

  for (unsigned int i = 0; i < this->dataPtr->sceneNode->numAttachedObjects();
      i++)
  {
    Ogre::Entity *entity = NULL;
    Ogre::MovableObject *obj = this->dataPtr->sceneNode->getAttachedObject(i);

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

  if (_cascade)
  {
    for (auto &child : this->dataPtr->children)
    {
      child->SetSpecular(_color, _cascade);
    }
  }

  this->dataPtr->specular = _color;

  this->dataPtr->sdf->GetElement("material")
      ->GetElement("specular")->Set(_color);
}

//////////////////////////////////////////////////
void Visual::SetEmissive(const common::Color &_color, const bool _cascade)
{
  for (unsigned int i = 0; i < this->dataPtr->sceneNode->numAttachedObjects();
      i++)
  {
    Ogre::Entity *entity = NULL;
    Ogre::MovableObject *obj = this->dataPtr->sceneNode->getAttachedObject(i);

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

  if (_cascade)
  {
    for (auto &child : this->dataPtr->children)
    {
      child->SetEmissive(_color, _cascade);
    }
  }

  this->dataPtr->emissive = _color;

  this->dataPtr->sdf->GetElement("material")
      ->GetElement("emissive")->Set(_color);
}

/////////////////////////////////////////////////
common::Color Visual::GetAmbient() const
{
  return this->dataPtr->ambient;
}

/////////////////////////////////////////////////
common::Color Visual::GetDiffuse() const
{
  return this->dataPtr->diffuse;
}

/////////////////////////////////////////////////
common::Color Visual::GetSpecular() const
{
  return this->dataPtr->specular;
}

/////////////////////////////////////////////////
common::Color Visual::GetEmissive() const
{
  return this->dataPtr->emissive;
}

//////////////////////////////////////////////////
void Visual::SetWireframe(bool _show)
{
  std::vector<VisualPtr>::iterator iter;
  for (iter = this->dataPtr->children.begin();
      iter != this->dataPtr->children.end(); ++iter)
  {
    (*iter)->SetWireframe(_show);
  }

  for (unsigned int i = 0; i < this->dataPtr->sceneNode->numAttachedObjects();
      i++)
  {
    Ogre::Entity *entity = NULL;
    Ogre::MovableObject *obj = this->dataPtr->sceneNode->getAttachedObject(i);

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
void Visual::SetTransparencyInnerLoop(Ogre::SceneNode *_sceneNode)
{
  for (unsigned int i = 0; i < _sceneNode->numAttachedObjects(); ++i)
  {
    Ogre::Entity *entity = NULL;
    Ogre::MovableObject *obj = _sceneNode->getAttachedObject(i);

    entity = dynamic_cast<Ogre::Entity*>(obj);

    if (!entity)
      continue;

    if (entity->getName().find("__COLLISION_VISUAL__") != std::string::npos)
      continue;

    // For each ogre::entity
    for (unsigned int j = 0; j < entity->getNumSubEntities(); ++j)
    {
      Ogre::SubEntity *subEntity = entity->getSubEntity(j);
      Ogre::MaterialPtr material = subEntity->getMaterial();

      unsigned int techniqueCount, passCount, unitStateCount;
      Ogre::Technique *technique;
      Ogre::Pass *pass;
      Ogre::ColourValue dc;

      for (techniqueCount = 0; techniqueCount < material->getNumTechniques();
           ++techniqueCount)
      {
        technique = material->getTechnique(techniqueCount);

        for (passCount = 0; passCount < technique->getNumPasses(); ++passCount)
        {
          pass = technique->getPass(passCount);

          // Need to fix transparency
          if (!pass->isProgrammable() &&
              pass->getPolygonMode() == Ogre::PM_SOLID)
          {
            pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
          }

          if (this->dataPtr->transparency > 0.0)
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
          dc.a = (1.0f - this->dataPtr->transparency);
          pass->setDiffuse(dc);
          this->dataPtr->diffuse = Conversions::Convert(dc);

          for (unitStateCount = 0; unitStateCount <
              pass->getNumTextureUnitStates(); ++unitStateCount)
          {
            auto textureUnitState = pass->getTextureUnitState(unitStateCount);

            if (textureUnitState->getColourBlendMode().operation ==
                Ogre::LBX_SOURCE1)
            {
              textureUnitState->setAlphaOperation(
                  Ogre::LBX_SOURCE1, Ogre::LBS_MANUAL, Ogre::LBS_CURRENT,
                  1.0 - this->dataPtr->transparency);
            }
          }
        }
      }
    }
  }
}

//////////////////////////////////////////////////
void Visual::SetTransparency(float _trans, const bool _cascade)
{
  if (math::equal(this->dataPtr->transparency, _trans))
    return;

  this->dataPtr->transparency = std::min(
      std::max(_trans, static_cast<float>(0.0)), static_cast<float>(1.0));

  if (_cascade)
  {
    for (auto &child : this->dataPtr->children)
    {
      // Don't change some visualizations when link changes
      if (!(this->GetType() == VT_LINK &&
          (child->GetType() == VT_GUI ||
           child->GetType() == VT_PHYSICS ||
           child->GetType() == VT_SENSOR)))
      {
        child->SetTransparency(_trans);
      }
    }
  }

  this->SetTransparencyInnerLoop(this->dataPtr->sceneNode);

  // For child nodes' scene nodes
  for (unsigned int i = 0; i < this->dataPtr->sceneNode->numChildren(); ++i)
  {
    Ogre::SceneNode *childSceneNode = dynamic_cast<Ogre::SceneNode*>(
        this->dataPtr->sceneNode->getChild(i));

    this->SetTransparencyInnerLoop(childSceneNode);
  }

  if (this->dataPtr->useRTShader && this->dataPtr->scene->GetInitialized())
    RTShaderSystem::Instance()->UpdateShaders();

  this->dataPtr->sdf->GetElement("transparency")->Set(_trans);
}

//////////////////////////////////////////////////
void Visual::SetHighlighted(bool _highlighted)
{
  if (_highlighted)
  {
    math::Box bbox = this->GetBoundingBox();

    // Create the bounding box if it's not already created.
    if (!this->dataPtr->boundingBox)
    {
      this->dataPtr->boundingBox = new WireBox(shared_from_this(), bbox);
    }
    else
    {
      this->dataPtr->boundingBox->Init(bbox);
    }
    this->dataPtr->boundingBox->SetVisible(true);
  }
  else if (this->dataPtr->boundingBox)
  {
    this->dataPtr->boundingBox->SetVisible(false);
  }

  // If this is a link, highlight frame visual
  if (this->GetType() == VT_LINK)
  {
    VisualPtr linkFrameVis;
    for (auto child : this->dataPtr->children)
    {
      if (child->GetName().find("LINK_FRAME_VISUAL__") != std::string::npos)
        child->SetHighlighted(_highlighted);
    }
  }
}

//////////////////////////////////////////////////
bool Visual::GetHighlighted() const
{
  if (this->dataPtr->boundingBox)
  {
    return this->dataPtr->boundingBox->GetVisible();
  }
  return false;
}

//////////////////////////////////////////////////
float Visual::GetTransparency()
{
  return this->dataPtr->transparency;
}

//////////////////////////////////////////////////
void Visual::SetCastShadows(bool _shadows)
{
  for (int i = 0; i < this->dataPtr->sceneNode->numAttachedObjects(); i++)
  {
    Ogre::MovableObject *obj = this->dataPtr->sceneNode->getAttachedObject(i);
    obj->setCastShadows(_shadows);
  }

  if (this->IsStatic() && this->dataPtr->staticGeom)
    this->dataPtr->staticGeom->setCastShadows(_shadows);

  this->dataPtr->castShadows = _shadows;
  this->dataPtr->sdf->GetElement("cast_shadows")->Set(_shadows);
}

//////////////////////////////////////////////////
bool Visual::GetCastShadows() const
{
  return this->dataPtr->castShadows;
}

//////////////////////////////////////////////////
void Visual::SetVisible(bool _visible, bool _cascade)
{
  if (this->dataPtr->sceneNode)
    this->dataPtr->sceneNode->setVisible(_visible, _cascade);

  if (_cascade)
  {
    for (auto child: this->dataPtr->children)
      child->SetVisible(_visible);
  }

  this->dataPtr->visible = _visible;
}

//////////////////////////////////////////////////
uint32_t Visual::GetVisibilityFlags()
{
  return this->dataPtr->visibilityFlags;
}

//////////////////////////////////////////////////
void Visual::ToggleVisible()
{
  this->SetVisible(!this->GetVisible(), true);
}

//////////////////////////////////////////////////
bool Visual::GetVisible() const
{
  return this->dataPtr->visible;
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
  GZ_ASSERT(this->dataPtr->sceneNode, "Visual SceneNode is NULL");
  this->dataPtr->sceneNode->setPosition(_pos.x, _pos.y, _pos.z);

  this->dataPtr->sdf->GetElement("pose")->Set(this->GetPose());
}

//////////////////////////////////////////////////
void Visual::SetRotation(const math::Quaternion &_rot)
{
  GZ_ASSERT(this->dataPtr->sceneNode, "Visual SceneNode is NULL");
  this->dataPtr->sceneNode->setOrientation(
      Ogre::Quaternion(_rot.w, _rot.x, _rot.y, _rot.z));

  this->dataPtr->sdf->GetElement("pose")->Set(this->GetPose());
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
  return Conversions::Convert(this->dataPtr->sceneNode->getPosition());
}

//////////////////////////////////////////////////
math::Quaternion Visual::GetRotation() const
{
  return Conversions::Convert(this->dataPtr->sceneNode->getOrientation());
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
void Visual::SetWorldPose(const math::Pose &_pose)
{
  this->SetWorldPosition(_pose.pos);
  this->SetWorldRotation(_pose.rot);
}

//////////////////////////////////////////////////
void Visual::SetWorldPosition(const math::Vector3 &_pos)
{
  this->dataPtr->sceneNode->_setDerivedPosition(Conversions::Convert(_pos));
}

//////////////////////////////////////////////////
void Visual::SetWorldRotation(const math::Quaternion &_q)
{
  this->dataPtr->sceneNode->_setDerivedOrientation(Conversions::Convert(_q));
}

//////////////////////////////////////////////////
math::Pose Visual::GetWorldPose() const
{
  math::Pose pose;

  Ogre::Vector3 vpos;
  Ogre::Quaternion vquatern;

  if (this->dataPtr->sceneNode)
  {
    vpos = this->dataPtr->sceneNode->_getDerivedPosition();
    pose.pos.x = vpos.x;
    pose.pos.y = vpos.y;
    pose.pos.z = vpos.z;

    vquatern = this->dataPtr->sceneNode->_getDerivedOrientation();
    pose.rot.w = vquatern.w;
    pose.rot.x = vquatern.x;
    pose.rot.y = vquatern.y;
    pose.rot.z = vquatern.z;
  }

  return pose;
}


//////////////////////////////////////////////////
Ogre::SceneNode * Visual::GetSceneNode() const
{
  return this->dataPtr->sceneNode;
}


//////////////////////////////////////////////////
bool Visual::IsStatic() const
{
  return this->dataPtr->isStatic;
}

//////////////////////////////////////////////////
void Visual::EnableTrackVisual(VisualPtr _vis)
{
  this->dataPtr->sceneNode->setAutoTracking(true, _vis->GetSceneNode());
}

//////////////////////////////////////////////////
void Visual::DisableTrackVisual()
{
  this->dataPtr->sceneNode->setAutoTracking(false);
}

//////////////////////////////////////////////////
std::string Visual::GetNormalMap() const
{
  std::string file = this->dataPtr->sdf->GetElement("material")->GetElement(
      "shader")->GetElement("normal_map")->Get<std::string>();

  std::string uriFile = common::find_file(file);
  if (!uriFile.empty())
    file = uriFile;

  return file;
}

//////////////////////////////////////////////////
void Visual::SetNormalMap(const std::string &_nmap)
{
  this->dataPtr->sdf->GetElement("material")->GetElement(
      "shader")->GetElement("normal_map")->GetValue()->Set(_nmap);
  if (this->dataPtr->useRTShader && this->dataPtr->scene->GetInitialized())
    RTShaderSystem::Instance()->UpdateShaders();
}

//////////////////////////////////////////////////
std::string Visual::GetShaderType() const
{
  return this->dataPtr->sdf->GetElement("material")->GetElement(
      "shader")->Get<std::string>("type");
}

//////////////////////////////////////////////////
void Visual::SetShaderType(const std::string &_type)
{
  this->dataPtr->sdf->GetElement("material")->GetElement(
      "shader")->GetAttribute("type")->Set(_type);
  if (this->dataPtr->useRTShader && this->dataPtr->scene->GetInitialized())
    RTShaderSystem::Instance()->UpdateShaders();
}


//////////////////////////////////////////////////
void Visual::SetRibbonTrail(bool _value, const common::Color &_initialColor,
                            const common::Color &_changeColor)
{
  if (this->dataPtr->ribbonTrail == NULL)
  {
    this->dataPtr->ribbonTrail =
        this->dataPtr->scene->GetManager()->createRibbonTrail(
        this->GetName() + "_RibbonTrail");
    this->dataPtr->ribbonTrail->setMaterialName("Gazebo/RibbonTrail");
    // this->dataPtr->ribbonTrail->setTrailLength(100);
    this->dataPtr->ribbonTrail->setMaxChainElements(10000);
    // this->dataPtr->ribbonTrail->setNumberOfChains(1);
    this->dataPtr->ribbonTrail->setVisible(false);
    this->dataPtr->ribbonTrail->setCastShadows(false);
    this->dataPtr->ribbonTrail->setInitialWidth(0, 0.05);
    this->dataPtr->scene->GetManager()->getRootSceneNode()->attachObject(
        this->dataPtr->ribbonTrail);

    this->dataPtr->ribbonTrail->setInitialColour(0,
        Conversions::Convert(_initialColor));
    this->dataPtr->ribbonTrail->setColourChange(0,
        Conversions::Convert(_changeColor));
  }

  if (_value)
  {
    try
    {
      this->dataPtr->ribbonTrail->addNode(this->dataPtr->sceneNode);
    }
    catch(...)
    {
      gzerr << "Unable to create ribbon trail\n";
    }
  }
  else
  {
    this->dataPtr->ribbonTrail->removeNode(this->dataPtr->sceneNode);
    this->dataPtr->ribbonTrail->clearChain(0);
  }
  this->dataPtr->ribbonTrail->setVisible(_value);
}

//////////////////////////////////////////////////
DynamicLines *Visual::CreateDynamicLine(RenderOpType _type)
{
  this->dataPtr->preRenderConnection = event::Events::ConnectPreRender(
      boost::bind(&Visual::Update, this));

  DynamicLines *line = new DynamicLines(_type);
  this->dataPtr->lines.push_back(line);
  this->AttachObject(line);
  return line;
}

//////////////////////////////////////////////////
void Visual::DeleteDynamicLine(DynamicLines *_line)
{
  // delete instance from lines vector
  for (std::list<DynamicLines*>::iterator iter = this->dataPtr->lines.begin();
       iter != this->dataPtr->lines.end(); ++iter)
  {
    if (*iter == _line)
    {
      delete *iter;
      this->dataPtr->lines.erase(iter);
      break;
    }
  }
}

//////////////////////////////////////////////////
void Visual::AttachLineVertex(DynamicLines *_line, unsigned int _index)
{
  this->dataPtr->lineVertices.push_back(std::make_pair(_line, _index));
  _line->SetPoint(_index, this->GetWorldPose().pos);
}

//////////////////////////////////////////////////
std::string Visual::GetMaterialName() const
{
  return this->dataPtr->myMaterialName;
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
  node->_update(false, true);

  Ogre::Matrix4 invTransform =
      this->dataPtr->sceneNode->_getFullTransform().inverse();

  Ogre::SceneNode::ChildNodeIterator it = node->getChildIterator();

  for (int i = 0; i < node->numAttachedObjects(); i++)
  {
    Ogre::MovableObject *obj = node->getAttachedObject(i);

    if (obj->isVisible() && obj->getMovableType() != "gazebo::dynamiclines"
        && obj->getMovableType() != "BillboardSet"
        && obj->getVisibilityFlags() != GZ_VISIBILITY_GUI)
    {
      Ogre::Any any = obj->getUserObjectBindings().getUserAny();
      if (any.getType() == typeid(std::string))
      {
        std::string str = Ogre::any_cast<std::string>(any);
        if (str.substr(0, 3) == "rot" || str.substr(0, 5) == "trans"
            || str.substr(0, 5) == "scale" ||
            str.find("_APPLY_WRENCH_") != std::string::npos)
          continue;
      }

      Ogre::AxisAlignedBox bb = obj->getBoundingBox();

      math::Vector3 min;
      math::Vector3 max;

      // Ogre does not return a valid bounding box for lights.
      if (obj->getMovableType() == "Light")
      {
        min = math::Vector3(-0.5, -0.5, -0.5);
        max = math::Vector3(0.5, 0.5, 0.5);
      }
      else
      {
        // Get transform to be applied to the current node.
        Ogre::Matrix4 transform = invTransform * node->_getFullTransform();
        // Correct precision error which makes ogre's isAffine check fail.
        transform[3][0] = transform[3][1] = transform[3][2] = 0;
        transform[3][3] = 1;
        // get oriented bounding box in object's local space
        bb.transformAffine(transform);

        min = Conversions::Convert(bb.getMinimum());
        max = Conversions::Convert(bb.getMaximum());
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
  /*if (!this->dataPtr->sceneNode->getCreator()->hasEntity(_meshName) &&
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

        ignition::math::Matrix4d trans = node->Transform();
        ignition::math::Vector3d pos = trans.Translation();
        ignition::math::Quaterniond q = trans.Rotation();
        bone->setPosition(Ogre::Vector3(pos.X(), pos.Y(), pos.Z()));
        bone->setOrientation(Ogre::Quaternion(q.W(), q.X(), q.Y(), q.Z()));
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
        subMesh.Center(ignition::math::Vector3d::Zero);

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
        *vertices++ = subMesh.Vertex(j).X();
        *vertices++ = subMesh.Vertex(j).Y();
        *vertices++ = subMesh.Vertex(j).Z();

        if (subMesh.GetNormalCount() > 0)
        {
          *vertices++ = subMesh.Normal(j).X();
          *vertices++ = subMesh.Normal(j).Y();
          *vertices++ = subMesh.Normal(j).Z();
        }

        if (subMesh.GetTexCoordCount() > 0)
        {
          *vertices++ = subMesh.TexCoord(j).X();
          *vertices++ = subMesh.TexCoord(j).Y();
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

    ignition::math::Vector3d max = _mesh->Max();
    ignition::math::Vector3d min = _mesh->Min();

    if (_mesh->HasSkeleton())
    {
      min = ignition::math::Vector3d(-1, -1, -1);
      max = ignition::math::Vector3d(1, 1, 1);
    }

    if (!max.IsFinite())
      gzthrow("Max bounding box is not finite[" << max << "]\n");

    if (!min.IsFinite())
      gzthrow("Min bounding box is not finite[" << min << "]\n");

    ogreMesh->_setBounds(Ogre::AxisAlignedBox(
          Ogre::Vector3(min.X(), min.Y(), min.Z()),
          Ogre::Vector3(max.X(), max.Y(), max.Z())),
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

  // Set meta information
  if (_msg->has_meta())
  {
    if (_msg->meta().has_layer())
    {
      this->dataPtr->layer = _msg->meta().layer();
      rendering::Events::newLayer(this->dataPtr->layer);
    }
  }

  if (_msg->has_pose())
    this->SetPose(msgs::ConvertIgn(_msg->pose()));

  if (_msg->has_visible())
    this->SetVisible(_msg->visible());

  if (_msg->has_scale())
    this->SetScale(msgs::ConvertIgn(_msg->scale()));

  if (_msg->has_geometry() && _msg->geometry().has_type())
  {
    std::string newGeometryType =
        msgs::ConvertGeometryType(_msg->geometry().type());

    std::string geometryType = this->GetGeometryType();
    std::string geometryName = this->GetMeshName();

    std::string newGeometryName = geometryName;
    if (_msg->geometry().has_mesh() && _msg->geometry().mesh().has_filename())
        newGeometryName = _msg->geometry().mesh().filename();

    if (newGeometryType != geometryType ||
        (newGeometryType == "mesh" && newGeometryName != geometryName))
    {
      std::string origMaterial = this->dataPtr->myMaterialName;
      float origTransparency = this->dataPtr->transparency;

      sdf::ElementPtr geomElem = this->dataPtr->sdf->GetElement("geometry");
      geomElem->ClearElements();

      this->DetachObjects();

      if (newGeometryType == "box" || newGeometryType == "cylinder" ||
          newGeometryType == "sphere" || newGeometryType == "plane")
      {
        this->AttachMesh("unit_" + newGeometryType);
        sdf::ElementPtr shapeElem = geomElem->AddElement(newGeometryType);
        if (newGeometryType == "sphere" || newGeometryType == "cylinder")
          shapeElem->GetElement("radius")->Set(0.5);
      }
      else if (newGeometryType == "mesh")
      {
        std::string filename = _msg->geometry().mesh().filename();
        std::string meshName = common::find_file(filename);
        std::string submeshName;
        bool centerSubmesh = false;

        if (meshName.empty())
        {
          meshName = "unit_box";
          gzerr << "No mesh found, setting mesh to a unit box" << std::endl;
        }
        else
        {
          if (_msg->geometry().mesh().has_submesh())
            submeshName= _msg->geometry().mesh().submesh();
          if (_msg->geometry().mesh().has_center_submesh())
            centerSubmesh= _msg->geometry().mesh().center_submesh();
        }

        this->AttachMesh(meshName, submeshName, centerSubmesh);

        sdf::ElementPtr meshElem = geomElem->AddElement(newGeometryType);
        if (!filename.empty())
          meshElem->GetElement("uri")->Set(filename);
        if (!submeshName.empty())
        {
          sdf::ElementPtr submeshElem = meshElem->GetElement("submesh");
          submeshElem->GetElement("name")->Set(submeshName);
          submeshElem->GetElement("center")->Set(centerSubmesh);
        }
      }
      this->SetTransparency(origTransparency);
      this->SetMaterial(origMaterial);
    }

    math::Vector3 geomScale(1, 1, 1);

    if (_msg->geometry().type() == msgs::Geometry::BOX)
    {
      geomScale = msgs::ConvertIgn(_msg->geometry().box().size());
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
      if (_msg->geometry().plane().has_size())
      {
        geomScale.x = _msg->geometry().plane().size().x();
        geomScale.y = _msg->geometry().plane().size().y();
      }
    }
    else if (_msg->geometry().type() == msgs::Geometry::IMAGE)
    {
      geomScale.x = geomScale.y = geomScale.z
          = _msg->geometry().image().scale();
    }
    else if (_msg->geometry().type() == msgs::Geometry::HEIGHTMAP)
    {
      geomScale = msgs::ConvertIgn(_msg->geometry().heightmap().size());
    }
    else if (_msg->geometry().type() == msgs::Geometry::MESH)
    {
      if (_msg->geometry().mesh().has_scale())
      {
        geomScale = msgs::ConvertIgn(_msg->geometry().mesh().scale());
      }
    }
    else if (_msg->geometry().type() == msgs::Geometry::EMPTY ||
        _msg->geometry().type() == msgs::Geometry::POLYLINE)
    {
      // do nothing for now - keep unit scale.
    }
    else
      gzerr << "Unknown geometry type[" << _msg->geometry().type() << "]\n";

    this->SetScale(geomScale * this->dataPtr->scale / this->DerivedScale());
  }

  if (_msg->has_material())
  {
    if (_msg->material().has_lighting())
    {
      this->SetLighting(_msg->material().lighting());
    }

    if (_msg->material().has_script())
    {
      for (int i = 0; i < _msg->material().script().uri_size(); ++i)
      {
        RenderEngine::Instance()->AddResourcePath(
            _msg->material().script().uri(i));
      }
      if (_msg->material().script().has_name() &&
          !_msg->material().script().name().empty())
      {
        this->SetMaterial(_msg->material().script().name());
      }
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
      else
      {
        gzerr << "Unrecognized shader type" << std::endl;
      }

      if (_msg->material().has_normal_map())
        this->SetNormalMap(_msg->material().normal_map());
    }
  }

  if (_msg->has_transparency())
  {
    this->SetTransparency(_msg->transparency());
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
  return this->dataPtr->parent;
}

//////////////////////////////////////////////////
VisualPtr Visual::GetRootVisual()
{
  VisualPtr p = shared_from_this();
  while (p->GetParent() &&
      p->GetParent() != this->dataPtr->scene->GetWorldVisual())
  {
    p = p->GetParent();
  }

  return p;
}

//////////////////////////////////////////////////
VisualPtr Visual::GetNthAncestor(unsigned int _n)
{
  // Get visual's depth
  unsigned int depth = this->GetDepth();

  // Must be deeper than ancestor
  if (depth < _n)
    return NULL;

  // Get ancestor
  VisualPtr p = shared_from_this();
  while (p->GetParent() && depth != _n)
  {
    p = p->GetParent();
    --depth;
  }

  return p;
}

/////////////////////////////////////////////////
bool Visual::IsAncestorOf(const rendering::VisualPtr _visual) const
{
  if (!_visual || !this->dataPtr->scene)
    return false;

  rendering::VisualPtr world = this->dataPtr->scene->GetWorldVisual();
  rendering::VisualPtr vis = _visual->GetParent();
  while (vis)
  {
    if (vis->GetName() == this->GetName())
      return true;
    vis = vis->GetParent();
  }

  return false;
}

/////////////////////////////////////////////////
bool Visual::IsDescendantOf(const rendering::VisualPtr _visual) const
{
  if (!_visual || !this->dataPtr->scene)
    return false;

  rendering::VisualPtr world = this->dataPtr->scene->GetWorldVisual();
  rendering::VisualPtr vis = this->GetParent();
  while (vis)
  {
    if (vis->GetName() == _visual->GetName())
      return true;
    vis = vis->GetParent();
  }

  return false;
}

//////////////////////////////////////////////////
unsigned int Visual::GetDepth() const
{
  boost::shared_ptr<Visual const> p = shared_from_this();
  unsigned int depth = 0;
  while (p->GetParent())
  {
    p = p->GetParent();
    ++depth;
  }
  return depth;
}

//////////////////////////////////////////////////
bool Visual::IsPlane() const
{
  if (this->dataPtr->sdf->HasElement("geometry"))
  {
    sdf::ElementPtr geomElem = this->dataPtr->sdf->GetElement("geometry");
    if (geomElem->HasElement("plane"))
      return true;
  }

  std::vector<VisualPtr>::const_iterator iter;
  for (iter = this->dataPtr->children.begin();
      iter != this->dataPtr->children.end(); ++iter)
  {
    if ((*iter)->IsPlane())
      return true;
  }

  return false;
}

//////////////////////////////////////////////////
std::string Visual::GetGeometryType() const
{
  if (this->dataPtr->sdf->HasElement("geometry"))
  {
    sdf::ElementPtr geomElem = this->dataPtr->sdf->GetElement("geometry");
    if (geomElem->HasElement("box"))
      return "box";
    else if (geomElem->HasElement("sphere"))
      return "sphere";
    else if (geomElem->HasElement("cylinder"))
      return "cylinder";
    else if (geomElem->HasElement("plane"))
      return "plane";
    else if (geomElem->HasElement("image"))
      return "image";
    else if (geomElem->HasElement("polyline"))
      return "polyline";
    else if (geomElem->HasElement("mesh"))
      return "mesh";
    else if (geomElem->HasElement("heightmap"))
      return "heightmap";
  }
  return "";
}

//////////////////////////////////////////////////
std::string Visual::GetMeshName() const
{
  if (!this->dataPtr->meshName.empty())
  {
    return this->dataPtr->meshName;
  }

  if (this->dataPtr->sdf->HasElement("geometry"))
  {
    sdf::ElementPtr geomElem = this->dataPtr->sdf->GetElement("geometry");
    if (geomElem->HasElement("box"))
      return "unit_box";
    else if (geomElem->HasElement("sphere"))
      return "unit_sphere";
    else if (geomElem->HasElement("cylinder"))
      return "unit_cylinder";
    else if (geomElem->HasElement("plane"))
      return "unit_plane";
    else if (geomElem->HasElement("polyline"))
    {
      std::string polyLineName = this->GetName();
      common::MeshManager *meshManager = common::MeshManager::Instance();

      if (!meshManager->IsValidFilename(polyLineName))
      {
        sdf::ElementPtr polylineElem = geomElem->GetElement("polyline");

        std::vector<std::vector<ignition::math::Vector2d> > polylines;
        while (polylineElem)
        {
          std::vector<ignition::math::Vector2d> vertices;
          sdf::ElementPtr pointElem = polylineElem->GetElement("point");
          while (pointElem)
          {
            ignition::math::Vector2d point =
              pointElem->Get<ignition::math::Vector2d>();
            vertices.push_back(point);
            pointElem = pointElem->GetNextElement("point");
          }
          polylineElem = polylineElem->GetNextElement("polyline");
          polylines.push_back(vertices);
        }

        meshManager->CreateExtrudedPolyline(polyLineName, polylines,
            geomElem->GetElement("polyline")->Get<double>("height"));
      }
      return polyLineName;
    }
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
  if (!this->dataPtr->subMeshName.empty())
  {
    return this->dataPtr->subMeshName;
  }

  std::string result;

  if (this->dataPtr->sdf->HasElement("geometry"))
  {
    sdf::ElementPtr geomElem = this->dataPtr->sdf->GetElement("geometry");
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

  if (this->dataPtr->sdf->HasElement("geometry"))
  {
    sdf::ElementPtr geomElem = this->dataPtr->sdf->GetElement("geometry");
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

  this->dataPtr->onAnimationComplete = _onComplete;

  std::string animName = this->GetName() + "_animation";

  Ogre::Animation *anim =
    this->dataPtr->sceneNode->getCreator()->createAnimation(animName, _time);
  anim->setInterpolationMode(Ogre::Animation::IM_SPLINE);

  Ogre::NodeAnimationTrack *strack = anim->createNodeTrack(0,
      this->dataPtr->sceneNode);

  key = strack->createNodeKeyFrame(0);
  key->setTranslate(Ogre::Vector3(start.x, start.y, start.z));
  key->setRotation(this->dataPtr->sceneNode->getOrientation());

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

  this->dataPtr->animState =
    this->dataPtr->sceneNode->getCreator()->createAnimationState(animName);

  this->dataPtr->animState->setTimePosition(0);
  this->dataPtr->animState->setEnabled(true);
  this->dataPtr->animState->setLoop(false);
  this->dataPtr->prevAnimTime = common::Time::GetWallTime();

  if (!this->dataPtr->preRenderConnection)
  {
    this->dataPtr->preRenderConnection =
      event::Events::ConnectPreRender(boost::bind(&Visual::Update, this));
  }
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
    this->dataPtr->sceneNode->getCreator()->createAnimation(animName, _time);
  anim->setInterpolationMode(Ogre::Animation::IM_SPLINE);

  Ogre::NodeAnimationTrack *strack =
      anim->createNodeTrack(0, this->dataPtr->sceneNode);

  key = strack->createNodeKeyFrame(0);
  key->setTranslate(Ogre::Vector3(start.x, start.y, start.z));
  key->setRotation(this->dataPtr->sceneNode->getOrientation());

  key = strack->createNodeKeyFrame(_time);
  key->setTranslate(Ogre::Vector3(_pose.pos.x, _pose.pos.y, _pose.pos.z));
  key->setRotation(Conversions::Convert(rotFinal));

  this->dataPtr->animState =
    this->dataPtr->sceneNode->getCreator()->createAnimationState(animName);

  this->dataPtr->animState->setTimePosition(0);
  this->dataPtr->animState->setEnabled(true);
  this->dataPtr->animState->setLoop(false);
  this->dataPtr->prevAnimTime = common::Time::GetWallTime();

  this->dataPtr->preRenderConnection =
    event::Events::ConnectPreRender(boost::bind(&Visual::Update, this));
}

//////////////////////////////////////////////////
void Visual::ShowBoundingBox()
{
  this->dataPtr->sceneNode->showBoundingBox(true);
}

//////////////////////////////////////////////////
void Visual::SetScene(ScenePtr _scene)
{
  this->dataPtr->scene = _scene;
}

//////////////////////////////////////////////////
ScenePtr Visual::GetScene() const
{
  return this->dataPtr->scene;
}

//////////////////////////////////////////////////
void Visual::ShowCollision(bool _show)
{
  if (this->GetName().find("__COLLISION_VISUAL__") != std::string::npos)
    this->SetVisible(_show);

  std::vector<VisualPtr>::iterator iter;
  for (iter = this->dataPtr->children.begin();
      iter != this->dataPtr->children.end(); ++iter)
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
  for (iter = this->dataPtr->children.begin();
      iter != this->dataPtr->children.end(); ++iter)
  {
    (*iter)->ShowSkeleton(_show);
  }
}

//////////////////////////////////////////////////
void Visual::SetVisibilityFlags(uint32_t _flags)
{
  for (std::vector<VisualPtr>::iterator iter = this->dataPtr->children.begin();
       iter != this->dataPtr->children.end(); ++iter)
  {
    (*iter)->SetVisibilityFlags(_flags);
  }

  for (int i = 0; i < this->dataPtr->sceneNode->numAttachedObjects(); ++i)
  {
    this->dataPtr->sceneNode->getAttachedObject(i)->setVisibilityFlags(_flags);
  }

  for (unsigned int i = 0; i < this->dataPtr->sceneNode->numChildren(); ++i)
  {
    Ogre::SceneNode *sn =
        (Ogre::SceneNode*)(this->dataPtr->sceneNode->getChild(i));

    for (int j = 0; j < sn->numAttachedObjects(); ++j)
      sn->getAttachedObject(j)->setVisibilityFlags(_flags);
  }

  this->dataPtr->visibilityFlags = _flags;
}

//////////////////////////////////////////////////
void Visual::ShowJoints(bool _show)
{
  if (this->GetName().find("JOINT_VISUAL__") != std::string::npos)
    this->SetVisible(_show);

  for (auto &child : this->dataPtr->children)
  {
    child->ShowJoints(_show);
  }
}

//////////////////////////////////////////////////
void Visual::ShowCOM(bool _show)
{
  if (this->GetName().find("COM_VISUAL__") != std::string::npos)
    this->SetVisible(_show);

  for (auto &child : this->dataPtr->children)
  {
    child->ShowCOM(_show);
  }
}

//////////////////////////////////////////////////
void Visual::ShowInertia(bool _show)
{
  if (this->GetName().find("INERTIA_VISUAL__") != std::string::npos)
    this->SetVisible(_show);

  for (auto &child : this->dataPtr->children)
  {
    child->ShowInertia(_show);
  }
}

//////////////////////////////////////////////////
void Visual::ShowLinkFrame(bool _show)
{
  if (this->GetName().find("LINK_FRAME_VISUAL__") != std::string::npos)
    this->SetVisible(_show);

  for (auto &child : this->dataPtr->children)
  {
    child->ShowLinkFrame(_show);
  }
}

//////////////////////////////////////////////////
void Visual::SetSkeletonPose(const msgs::PoseAnimation &_pose)
{
  if (!this->dataPtr->skeleton)
  {
    gzerr << "Visual " << this->GetName() << " has no skeleton.\n";
    return;
  }

  for (int i = 0; i < _pose.pose_size(); i++)
  {
    const msgs::Pose& bonePose = _pose.pose(i);
    if (!this->dataPtr->skeleton->hasBone(bonePose.name()))
      continue;
    Ogre::Bone *bone = this->dataPtr->skeleton->getBone(bonePose.name());
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
  if (this->dataPtr->sdf->HasElement("plugin"))
  {
    sdf::ElementPtr pluginElem = this->dataPtr->sdf->GetElement("plugin");
    while (pluginElem)
    {
      this->LoadPlugin(pluginElem);
      pluginElem = pluginElem->GetNextElement("plugin");
    }
  }


  for (std::vector<VisualPluginPtr>::iterator iter =
      this->dataPtr->plugins.begin();
      iter != this->dataPtr->plugins.end(); ++iter)
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
    this->dataPtr->plugins.push_back(plugin);

    if (this->dataPtr->initialized)
      plugin->Init();
  }
}

//////////////////////////////////////////////////
void Visual::RemovePlugin(const std::string &_name)
{
  std::vector<VisualPluginPtr>::iterator iter;
  for (iter = this->dataPtr->plugins.begin();
      iter != this->dataPtr->plugins.end(); ++iter)
  {
    if ((*iter)->GetHandle() == _name)
    {
      this->dataPtr->plugins.erase(iter);
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
  return this->dataPtr->id;
}

//////////////////////////////////////////////////
void Visual::SetId(uint32_t _id)
{
  if (this->dataPtr->id == _id)
    return;

  // set new id and also let the scene know that the id has changed.
  this->dataPtr->scene->SetVisualId(shared_from_this(), _id);
  this->dataPtr->id = _id;
}

//////////////////////////////////////////////////
sdf::ElementPtr Visual::GetSDF() const
{
  return this->dataPtr->sdf;
}

//////////////////////////////////////////////////
Visual::VisualType Visual::GetType() const
{
  return this->dataPtr->type;
}

//////////////////////////////////////////////////
void Visual::SetType(const Visual::VisualType _type)
{
  this->dataPtr->type = _type;
}

//////////////////////////////////////////////////
void Visual::ToggleLayer(const int32_t _layer)
{
  // Visuals with negative layers are always visible
  if (this->dataPtr->layer < 0)
    return;

  if (this->dataPtr->layer == _layer)
  {
    this->ToggleVisible();
  }
}

//////////////////////////////////////////////////
Visual::VisualType Visual::ConvertVisualType(const msgs::Visual::Type &_type)
{
  Visual::VisualType visualType = Visual::VT_ENTITY;

  switch (_type)
  {
    case msgs::Visual::ENTITY:
      visualType = Visual::VT_ENTITY;
      break;
    case msgs::Visual::MODEL:
      visualType = Visual::VT_MODEL;
      break;
    case msgs::Visual::LINK:
      visualType = Visual::VT_LINK;
      break;
    case msgs::Visual::VISUAL:
      visualType = Visual::VT_VISUAL;
      break;
    case msgs::Visual::COLLISION:
      visualType = Visual::VT_COLLISION;
      break;
    case msgs::Visual::SENSOR:
      visualType = Visual::VT_SENSOR;
      break;
    case msgs::Visual::GUI:
      visualType = Visual::VT_GUI;
      break;
    case msgs::Visual::PHYSICS:
      visualType = Visual::VT_PHYSICS;
      break;
    default:
      gzerr << "Cannot convert visual type. Defaults to 'VT_ENTITY'"
          << std::endl;
      break;
  }
  return visualType;
}

//////////////////////////////////////////////////
msgs::Visual::Type Visual::ConvertVisualType(const Visual::VisualType &_type)
{
  msgs::Visual::Type visualType = msgs::Visual::ENTITY;

  switch (_type)
  {
    case Visual::VT_ENTITY:
      visualType = msgs::Visual::ENTITY;
      break;
    case Visual::VT_MODEL:
      visualType = msgs::Visual::MODEL;
      break;
    case Visual::VT_LINK:
      visualType = msgs::Visual::LINK;
      break;
    case Visual::VT_VISUAL:
      visualType = msgs::Visual::VISUAL;
      break;
    case Visual::VT_COLLISION:
      visualType = msgs::Visual::COLLISION;
      break;
    case Visual::VT_SENSOR:
      visualType = msgs::Visual::SENSOR;
      break;
    case Visual::VT_GUI:
      visualType = msgs::Visual::GUI;
      break;
    case Visual::VT_PHYSICS:
      visualType = msgs::Visual::PHYSICS;
      break;
    default:
      gzerr << "Cannot convert visual type. Defaults to 'msgs::Visual::ENTITY'"
          << std::endl;
      break;
  }
  return visualType;
}
