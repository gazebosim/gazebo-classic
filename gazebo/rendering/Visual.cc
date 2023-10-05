/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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
#include <boost/bind/bind.hpp>
#include <boost/function.hpp>
#include <boost/lexical_cast.hpp>

#include <ignition/common/Profiler.hh>
#include <ignition/math/AxisAlignedBox.hh>
#include <ignition/math/Color.hh>
#include <ignition/math/Helpers.hh>
#include <ignition/math/Matrix4.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector2.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Vector4.hh>
#include <ignition/transport/Node.hh>
#include <ignition/transport/TopicUtils.hh>

#include "gazebo/msgs/msgs.hh"

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

#include "gazebo/rendering/COMVisual.hh"
#include "gazebo/rendering/Conversions.hh"
#include "gazebo/rendering/DynamicLines.hh"
#include "gazebo/rendering/InertiaVisual.hh"
#include "gazebo/rendering/JointVisual.hh"
#include "gazebo/rendering/LinkFrameVisual.hh"
#include "gazebo/rendering/Material.hh"
#include "gazebo/rendering/MovableText.hh"
#include "gazebo/rendering/ogre_gazebo.h"
#include "gazebo/rendering/RenderEngine.hh"
#include "gazebo/rendering/RenderEvents.hh"
#include "gazebo/rendering/RTShaderSystem.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/SelectionObj.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/rendering/VisualPrivate.hh"
#include "gazebo/rendering/WireBox.hh"

using namespace gazebo;
using namespace rendering;

// Note: The value of ignition::math::MAX_UI32 is reserved as a flag.
uint32_t VisualPrivate::visualIdCount = ignition::math::MAX_UI32 - 1;

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
  this->dataPtr->boundingBox = nullptr;
  this->dataPtr->useRTShader = _useRTShader;
  this->dataPtr->visibilityFlags = GZ_VISIBILITY_ALL;

  this->dataPtr->sdf.reset(new sdf::Element);
  sdf::initFile("visual.sdf", this->dataPtr->sdf);

  this->SetName(_name);
  this->dataPtr->sceneNode = nullptr;
  this->dataPtr->animState = nullptr;
  this->dataPtr->skeleton = nullptr;
  this->dataPtr->initialized = false;
  this->dataPtr->lighting = true;
  this->dataPtr->castShadows = true;
  this->dataPtr->visible = true;
  this->dataPtr->layer = -1;
  this->dataPtr->inheritTransparency = true;

  std::string uniqueName = this->Name();
  int index = 0;
  while (_scene->OgreSceneManager()->hasSceneNode(uniqueName))
  {
    uniqueName = this->Name() + "_" +
                 boost::lexical_cast<std::string>(index++);
  }

  this->dataPtr->scene = _scene;
  this->SetName(uniqueName);
  this->dataPtr->sceneNode =
    this->dataPtr->scene->OgreSceneManager()->getRootSceneNode()->
        createChildSceneNode(this->Name());

  this->Init();
}

//////////////////////////////////////////////////
void Visual::Init(const std::string &_name, VisualPtr _parent,
    bool _useRTShader)
{
  this->dataPtr->id = this->dataPtr->visualIdCount--;
  this->dataPtr->boundingBox = nullptr;
  this->dataPtr->useRTShader = _useRTShader;

  this->dataPtr->sdf.reset(new sdf::Element);
  sdf::initFile("visual.sdf", this->dataPtr->sdf);

  this->SetName(_name);
  this->dataPtr->sceneNode = nullptr;
  this->dataPtr->animState = nullptr;
  this->dataPtr->initialized = false;
  this->dataPtr->lighting = true;
  this->dataPtr->castShadows = true;
  this->dataPtr->visible = true;
  this->dataPtr->layer = -1;
  this->dataPtr->inheritTransparency = true;

  Ogre::SceneNode *pnode = nullptr;
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

  std::string uniqueName = this->Name();
  int index = 0;
  while (pnode->getCreator()->hasSceneNode(uniqueName))
    uniqueName = this->Name() + "_" +
                 boost::lexical_cast<std::string>(index++);

  this->SetName(uniqueName);

  this->dataPtr->sceneNode = pnode->createChildSceneNode(this->Name());

  this->dataPtr->parent = _parent;
  this->dataPtr->scene = this->dataPtr->parent->GetScene();
  this->Init();
}

//////////////////////////////////////////////////
Visual::~Visual()
{
  this->Fini();

  delete this->dataPtr->typeMsg;

  delete this->dataPtr;
  this->dataPtr = nullptr;
}

/////////////////////////////////////////////////
void Visual::Fini()
{
  // Terminate callbacks before clearing other pointers
  this->dataPtr->preRenderConnection.reset();

  // Plugins might have callbacks
  this->dataPtr->plugins.clear();

  while (!this->dataPtr->children.empty())
  {
    VisualPtr childVis = this->dataPtr->children.front();
    this->DetachVisual(childVis);
    childVis->Fini();
  }
  this->dataPtr->children.clear();

  // Detach from the parent
  if (this->dataPtr->parent)
    this->dataPtr->parent->DetachVisual(this->Name());
  this->dataPtr->parent.reset();

  if (this->dataPtr->boundingBox)
    delete this->dataPtr->boundingBox;
  this->dataPtr->boundingBox = nullptr;

  this->dataPtr->lines.clear();

  if (this->dataPtr->sceneNode)
  {
    this->DestroyAllAttachedMovableObjects(this->dataPtr->sceneNode);
    this->dataPtr->scene->OgreSceneManager()->destroySceneNode(
        this->dataPtr->sceneNode);
  }
  this->dataPtr->sceneNode = nullptr;

  if (this->dataPtr->scene &&
      this->dataPtr->scene->GetVisual(this->dataPtr->id))
  {
    this->dataPtr->scene->RemoveVisual(this->dataPtr->id);
  }
  this->dataPtr->scene.reset();

  dataPtr->poseElem.reset();

  if (this->dataPtr->sdf)
    this->dataPtr->sdf->Reset();
  this->dataPtr->sdf.reset();
}

/////////////////////////////////////////////////
VisualPtr Visual::Clone(const std::string &_name, VisualPtr _newParent)
{
  VisualPtr result(new Visual(_name, _newParent));
  result->Load(this->dataPtr->sdf);
  result->SetScale(this->dataPtr->scale);
  result->SetVisibilityFlags(this->dataPtr->visibilityFlags);
  std::string visName = this->Name();
  for (auto iter: this->dataPtr->children)
  {
    // give a unique name by prefixing child visuals with the new clone name
    std::string childName = iter->Name();
    std::string newName = childName;
    size_t pos = childName.find(visName);
    if (pos == 0)
      newName = _name + childName.substr(pos+visName.size());
    iter->Clone(newName, result);
  }

  if (_newParent == this->dataPtr->scene->WorldVisual())
    result->SetWorldPose(this->WorldPose());
  result->ShowCollision(false);
  result->SetInheritTransparency(this->InheritTransparency());

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
    // Remove dynamic lines and entities in Visual
    // Other objects such as cameras, lights, and projectors
    // should have their own class for handling the deletion
    // of these ogre objects
    Ogre::MovableObject *obj = itObject.getNext();
    if (obj->getMovableType() == DynamicLines::GetMovableType())
      delete obj;
    else
    {
      Ogre::Entity *ent = dynamic_cast<Ogre::Entity *>(obj);
      if (!ent)
        continue;
      this->dataPtr->scene->OgreSceneManager()->destroyEntity(ent);
    }
  }
  this->dataPtr->lines.clear();

  // only recurse if there are no child visuals otherwise let them clean up
  // after themselves in Fini()
  if (this->dataPtr->children.empty())
  {
    // Recurse to child SceneNodes
    Ogre::SceneNode::ChildNodeIterator itChild = _sceneNode->getChildIterator();
    while (itChild.hasMoreElements())
    {
      Ogre::SceneNode* pChildNode =
          static_cast<Ogre::SceneNode*>(itChild.getNext());
      this->DestroyAllAttachedMovableObjects(pChildNode);
    }
  }
  _sceneNode->detachAllObjects();
}

//////////////////////////////////////////////////
void Visual::Init()
{
  this->dataPtr->type = VT_ENTITY;
  this->dataPtr->transparency = 0.0;
  this->dataPtr->isStatic = false;
  this->dataPtr->visible = true;
  this->dataPtr->ribbonTrail = nullptr;
  this->dataPtr->staticGeom = nullptr;
  this->dataPtr->layer = -1;
  this->dataPtr->wireframe = false;
  this->dataPtr->inheritTransparency = true;
  this->dataPtr->scale = ignition::math::Vector3d::One;

  this->dataPtr->initialized = true;
}

//////////////////////////////////////////////////
void Visual::LoadFromMsg(const boost::shared_ptr<msgs::Visual const> &_msg)
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
  if (this->dataPtr->sdf->HasElement("material"))
  {
    // Get shininess value from physics::World
    ignition::transport::Node node;
    msgs::Any rep;

    const std::string visualName = this->Name();

    const std::string serviceName = "/shininess";

    const std::string validServiceName =
        ignition::transport::TopicUtils::AsValidTopic(serviceName);

    bool tryServiceCall = true;
    if (validServiceName.empty())
    {
        gzerr << "Service name [" << serviceName << "] not valid" << std::endl;
        tryServiceCall = false;
    }

    {
      std::vector<ignition::transport::ServicePublisher> publishers;
      if (!node.ServiceInfo(validServiceName, publishers))
      {
        gzerr << "Service name [" << validServiceName << "] not advertised, "
              << "not attempting to load shininess for visual with name ["
              << this->Name() << "]."
              << std::endl;
        tryServiceCall = false;
      }
    }

    if (tryServiceCall)
    {
      ignition::msgs::StringMsg req;
      req.set_data(visualName);

      bool result;
      unsigned int timeout = 5000;
      bool executed = node.Request(validServiceName, req, timeout, rep, result);

      if (executed)
      {
        if (result)
          this->dataPtr->shininess = rep.double_value();
        else
          gzerr << "Service call [" << validServiceName << "] failed"
                << std::endl;
      }
      else
      {
        gzerr << "Service call [" << validServiceName << "] timed out"
              << std::endl;
      }
    }
  }

  std::ostringstream stream;
  ignition::math::Pose3d pose;
  Ogre::MovableObject *obj = nullptr;

  if (this->dataPtr->parent)
    this->dataPtr->parent->AttachVisual(shared_from_this());

  // Read the desired position and rotation of the mesh
  pose = this->dataPtr->sdf->Get<ignition::math::Pose3d>("pose");

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
  this->dataPtr->poseElem.reset();
  this->SetPose(pose);
  this->dataPtr->initialRelativePose = pose;

  // Keep transparency to set after setting material
  double sdfTransparency = -1;
  if (this->dataPtr->sdf->HasElement("transparency"))
  {
    sdfTransparency = this->dataPtr->sdf->Get<float>("transparency");
  }

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
      ignition::math::Vector2d size =
          geomElem->GetElement("plane")->Get<ignition::math::Vector2d>("size");
      geometrySize.Set(size.X(), size.Y(), 1);
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
      this->dataPtr->sceneNode->setScale(Conversions::Convert(localScale));
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
        auto matUri = common::asFullPath(uriElem->Get<std::string>(),
            scriptElem->FilePath());
        if (!matUri.empty())
          RenderEngine::Instance()->AddResourcePath(matUri);
        uriElem = uriElem->GetNextElement("uri");
      }

      std::string matName = scriptElem->Get<std::string>("name");

      if (!matName.empty())
        this->SetMaterial(matName);
    }

    if (matElemClone->HasElement("ambient"))
      this->SetAmbient(matElemClone->Get<ignition::math::Color>("ambient"));
    if (matElemClone->HasElement("diffuse"))
      this->SetDiffuse(matElemClone->Get<ignition::math::Color>("diffuse"));
    if (matElemClone->HasElement("specular"))
      this->SetSpecular(matElemClone->Get<ignition::math::Color>("specular"));
    if (matElemClone->HasElement("emissive"))
      this->SetEmissive(matElemClone->Get<ignition::math::Color>("emissive"));

    if (matElem->HasElement("lighting"))
    {
      this->SetLighting(matElem->Get<bool>("lighting"));
    }
  }

  if (sdfTransparency > -1)
  {
    this->SetTransparency(sdfTransparency);
  }
  // Update transparency the first time the visual is loaded. This is needed
  // when attaching to a parent who is semi-transparent.
  if (!ignition::math::equal(this->DerivedTransparency(),
      this->dataPtr->transparency))
    this->UpdateTransparency(true);

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

  // Set invisible if this visual's layer is not active
  if (!this->dataPtr->scene->LayerState(this->dataPtr->layer))
    this->SetVisible(false);
}

//////////////////////////////////////////////////
void Visual::Update()
{
  IGN_PROFILE("rendering::Visual::Update");
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
        Conversions::ConvertIgn(
          this->dataPtr->sceneNode->_getDerivedPosition()));
    liter->first->Update();
  }

  if (this->dataPtr->animState)
  {
    this->dataPtr->animState->addTime(
        (common::Time::GetWallTime() - this->dataPtr->prevAnimTime).Double());
    this->dataPtr->prevAnimTime = common::Time::GetWallTime();
    if (this->dataPtr->animState->hasEnded())
    {
      this->dataPtr->animState = nullptr;
      this->dataPtr->sceneNode->getCreator()->destroyAnimation(
          this->Name() + "_animation");
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
std::string Visual::Name() const
{
  return this->dataPtr->name;
}

//////////////////////////////////////////////////
void Visual::AttachVisual(VisualPtr _vis)
{
  if (!_vis)
    gzerr << "Visual is null" << std::endl;
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
  this->DetachVisual(_vis->Name());
}

//////////////////////////////////////////////////
void Visual::DetachVisual(const std::string &_name)
{
  for (auto iter = this->dataPtr->children.begin();
      iter != this->dataPtr->children.end(); ++iter)
  {
    if ((*iter)->Name() == _name)
    {
      VisualPtr childVis = (*iter);
      this->dataPtr->children.erase(iter);
      if (this->dataPtr->sceneNode && childVis->GetSceneNode())
        this->dataPtr->sceneNode->removeChild(childVis->GetSceneNode());
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

          // keep a pointer to the original submesh material so it can be used
          // to restore material state when setting transparency
          this->dataPtr->submeshMaterials[newMaterialName] = material;

          material = material->clone(newMaterialName);
          subEntity->setMaterial(material);
        }
      }
    }

    this->dataPtr->sceneNode->attachObject(_obj);
    if (this->dataPtr->useRTShader && this->dataPtr->scene->Initialized() &&
      _obj->getName().find("__COLLISION_VISUAL__") == std::string::npos)
    {
      RTShaderSystem::Instance()->UpdateShaders();
    }
    _obj->getUserObjectBindings().setUserAny(Ogre::Any(this->Name()));
  }
  else
    gzerr << "Visual[" << this->Name() << "] already has object["
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
  this->dataPtr->skeleton = nullptr;
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
    return nullptr;

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
    // build tangent vectors if normal mapping in tangent space is specified
    if (this->GetShaderType() == "normal_map_tangent_space")
    {
      Ogre::MeshPtr ogreMesh = Ogre::MeshManager::getSingleton().getByName(
        _meshName);
      if (!ogreMesh.isNull())
      {
        try
        {
          uint16_t src, dest;
          if (!ogreMesh->suggestTangentVectorBuildParams(
              Ogre::VES_TANGENT, src, dest))
          {
            ogreMesh->buildTangentVectors(Ogre::VES_TANGENT, src, dest);
          }
        }
        catch(Ogre::Exception &e)
        {
          gzwarn << "Problem generating tangent vectors for " << _meshName
                 << ". Normal map will not work: " << e.what() << std::endl;
        }
      }
    }

    obj = (Ogre::MovableObject*)
        (this->dataPtr->sceneNode->getCreator()->createEntity(objName,
        meshName));
  }

  this->AttachObject(obj);
  return obj;
}

//////////////////////////////////////////////////
void Visual::SetScale(const ignition::math::Vector3d &_scale)
{
  if (this->dataPtr->scale == _scale)
    return;

  // update geom size based on scale.
  this->UpdateGeomSize(this->DerivedScale() / this->dataPtr->scale * _scale);

  this->dataPtr->scale = _scale;

  if (!ignition::math::isnan(this->dataPtr->scale.X())
      && !ignition::math::isnan(this->dataPtr->scale.Y())
      && !ignition::math::isnan(this->dataPtr->scale.Z()))
  {
    this->dataPtr->sceneNode->setScale(
        Conversions::Convert(this->dataPtr->scale));
  }
  else
  {
    gzerr << Name() << ": Size of the collision contains one or several zeros."
      << " Collisions may not visualize properly." << std::endl;
  }
  // Scale selection object in case we have one attached. Other children were
  // scaled from UpdateGeomSize
  for (auto child : this->dataPtr->children)
  {
    auto selectionObj = std::dynamic_pointer_cast<SelectionObj>(child);

    if (selectionObj)
    {
      selectionObj->UpdateSize();
      break;
    }
  }
}

//////////////////////////////////////////////////
void Visual::UpdateGeomSize(const ignition::math::Vector3d &_scale)
{
  for (std::vector<VisualPtr>::iterator iter = this->dataPtr->children.begin();
       iter != this->dataPtr->children.end(); ++iter)
  {
    (*iter)->UpdateGeomSize(_scale * (*iter)->Scale());
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
ignition::math::Vector3d Visual::Scale() const
{
  return this->dataPtr->scale;
}

//////////////////////////////////////////////////
ignition::math::Vector3d Visual::DerivedScale() const
{
  ignition::math::Vector3d derivedScale = this->dataPtr->scale;

  VisualPtr worldVis = this->dataPtr->scene->WorldVisual();
  VisualPtr vis = this->GetParent();

  while (vis && vis != worldVis)
  {
    derivedScale = derivedScale * vis->Scale();
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

  ignition::math::Color matAmbient;
  ignition::math::Color matDiffuse;
  ignition::math::Color matSpecular;
  ignition::math::Color matEmissive;
  bool matColor = rendering::Material::MaterialAsColor(
      _materialName, matAmbient, matDiffuse, matSpecular, matEmissive);

  if (_unique)
  {
    // Create a custom material name
    std::string newMaterialName;
    newMaterialName = this->dataPtr->sceneNode->getName() + "_MATERIAL_" +
        _materialName;

    if (this->GetMaterialName() == newMaterialName &&
        matAmbient == this->Ambient() &&
        matDiffuse == this->Diffuse() &&
        matSpecular == this->Specular() &&
        matEmissive == this->Emissive())
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

  // check if material has color components, if so, set them.
  if (matColor)
  {
    this->dataPtr->ambient = matAmbient;
    this->dataPtr->diffuse = matDiffuse;
    this->dataPtr->specular = matSpecular;
    this->dataPtr->emissive = matEmissive;
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
          GZ_OGRE_SET_MATERIAL_BY_NAME(simpleRenderable,
              this->dataPtr->myMaterialName);
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

  // Apply material to all child visuals
  if (_cascade)
  {
    for (auto &child : this->dataPtr->children)
    {
      child->SetMaterial(_materialName, _unique, _cascade);
    }
  }

  if (this->dataPtr->useRTShader && this->dataPtr->scene->Initialized()
      && this->dataPtr->lighting &&
      this->Name().find("__COLLISION_VISUAL__") == std::string::npos)
  {
    RTShaderSystem::Instance()->UpdateShaders();
  }

  this->dataPtr->sdf->GetElement("material")->GetElement("script")
      ->GetElement("name")->Set(_materialName);
}

/////////////////////////////////////////////////
void Visual::SetMaterialShaderParam(const std::string &_paramName,
    const std::string &_shaderType, const std::string &_value)
{
  // currently only vertex and fragment shaders are supported
  if (_shaderType != "vertex" && _shaderType != "fragment")
  {
    gzerr << "Shader type: '" << _shaderType << "' is not supported"
          << std::endl;
    return;
  }

  // set the parameter based name and type defined in material script
  // and shaders
  auto setNamedParam = [](Ogre::GpuProgramParametersSharedPtr _params,
      const std::string &_name, const std::string &_v)
  {
    auto paramDef = _params->_findNamedConstantDefinition(_name);
    if (!paramDef)
      return;

    switch (paramDef->constType)
    {
      case Ogre::GCT_INT1:
      {
        int value = Ogre::StringConverter::parseInt(_v);
        _params->setNamedConstant(_name, value);
        break;
      }
      case Ogre::GCT_FLOAT1:
      {
        Ogre::Real value = Ogre::StringConverter::parseReal(_v);
        _params->setNamedConstant(_name, value);
        break;
      }
#if (OGRE_VERSION >= ((1 << 16) | (9 << 8) | 0))
      case Ogre::GCT_INT2:
      case Ogre::GCT_FLOAT2:
      {
        Ogre::Vector2 value = Ogre::StringConverter::parseVector2(_v);
        _params->setNamedConstant(_name, value);
        break;
      }
#endif
      case Ogre::GCT_INT3:
      case Ogre::GCT_FLOAT3:
      {
        Ogre::Vector3 value = Ogre::StringConverter::parseVector3(_v);
        _params->setNamedConstant(_name, value);
        break;
      }
      case Ogre::GCT_INT4:
      case Ogre::GCT_FLOAT4:
      {
        Ogre::Vector4 value = Ogre::StringConverter::parseVector4(_v);
        _params->setNamedConstant(_name, value);
        break;
      }
      case Ogre::GCT_MATRIX_4X4:
      {
        Ogre::Matrix4 value = Ogre::StringConverter::parseMatrix4(_v);
        _params->setNamedConstant(_name, value);
        break;
      }
      default:
        break;
    }
  };

  // loop through material techniques and passes to find the param
  Ogre::MaterialPtr mat = Ogre::MaterialManager::getSingleton().getByName(
      this->dataPtr->myMaterialName);
  if (mat.isNull())
  {
    gzerr << "Failed to find material: '" << this->dataPtr->myMaterialName
          << std::endl;
    return;
  }
  for (unsigned int i = 0; i < mat->getNumTechniques(); ++i)
  {
    Ogre::Technique *technique = mat->getTechnique(i);
    if (!technique)
      continue;
    for (unsigned int j = 0; j < technique->getNumPasses(); ++j)
    {
      Ogre::Pass *pass = technique->getPass(j);
      if (!pass)
        continue;

      // check if pass is programmable, ie if they are using shaders
      if (!pass->isProgrammable())
        continue;

      if (_shaderType == "vertex" && pass->hasVertexProgram())
      {
        setNamedParam(pass->getVertexProgramParameters(), _paramName, _value);
      }
      else if (_shaderType == "fragment" && pass->hasFragmentProgram())
      {
        setNamedParam(pass->getFragmentProgramParameters(), _paramName, _value);
      }
      else
      {
        gzerr << "Failed to retrieve shaders for material: '"
              << this->dataPtr->myMaterialName << "', technique: '"
              << technique->getName() << "', pass: '" << pass->getName() << "'"
              << std::endl;
        continue;
      }
    }
  }
}

/////////////////////////////////////////////////
void Visual::SetAmbient(const ignition::math::Color &_color,
    const bool _cascade)
{
  if (!this->dataPtr->lighting)
    return;

  if (this->dataPtr->myMaterialName.empty())
  {
    std::string matName = this->Name() + "_MATERIAL_";
    Ogre::MaterialManager::getSingleton().create(matName, "General");
    this->SetMaterial(matName);
  }

  for (unsigned int i = 0; i < this->dataPtr->sceneNode->numAttachedObjects();
      ++i)
  {
    Ogre::Entity *entity = nullptr;
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
void Visual::SetDiffuse(const ignition::math::Color &_color,
    const bool _cascade)
{
  if (!this->dataPtr->lighting)
    return;

  if (this->dataPtr->myMaterialName.empty())
  {
    std::string matName = this->Name() + "_MATERIAL_";
    Ogre::MaterialManager::getSingleton().create(matName, "General");
    this->SetMaterial(matName);
  }

  for (unsigned int i = 0; i < this->dataPtr->sceneNode->numAttachedObjects();
      i++)
  {
    Ogre::Entity *entity = nullptr;
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
  this->UpdateTransparency();

  this->dataPtr->sdf->GetElement("material")
      ->GetElement("diffuse")->Set(_color);
}

//////////////////////////////////////////////////
void Visual::SetSpecular(const ignition::math::Color &_color,
    const bool _cascade)
{
  if (!this->dataPtr->lighting)
    return;

  if (this->dataPtr->myMaterialName.empty())
  {
    std::string matName = this->Name() + "_MATERIAL_";
    Ogre::MaterialManager::getSingleton().create(matName, "General");
    this->SetMaterial(matName);
  }

  for (unsigned int i = 0; i < this->dataPtr->sceneNode->numAttachedObjects();
      i++)
  {
    Ogre::Entity *entity = nullptr;
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
          pass->setShininess(this->dataPtr->shininess);
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
void Visual::SetEmissive(const ignition::math::Color &_color,
    const bool _cascade)
{
  for (unsigned int i = 0; i < this->dataPtr->sceneNode->numAttachedObjects();
      i++)
  {
    Ogre::Entity *entity = nullptr;
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
ignition::math::Color Visual::Ambient() const
{
  return this->dataPtr->ambient;
}

/////////////////////////////////////////////////
ignition::math::Color Visual::Diffuse() const
{
  return this->dataPtr->diffuse;
}

/////////////////////////////////////////////////
ignition::math::Color Visual::Specular() const
{
  return this->dataPtr->specular;
}

/////////////////////////////////////////////////
ignition::math::Color Visual::Emissive() const
{
  return this->dataPtr->emissive;
}

/////////////////////////////////////////////////
double Visual::Shininess() const
{
  return this->dataPtr->shininess;
}

//////////////////////////////////////////////////
void Visual::SetWireframe(bool _show)
{
  if (this->dataPtr->type == VT_GUI || this->dataPtr->type == VT_PHYSICS ||
      this->dataPtr->type == VT_SENSOR)
    return;

  for (auto &iter : this->dataPtr->children)
  {
    iter->SetWireframe(_show);
  }

  if (this->dataPtr->wireframe == _show)
    return;

  this->dataPtr->wireframe = _show;
  for (unsigned int i = 0; i < this->dataPtr->sceneNode->numAttachedObjects();
      i++)
  {
    Ogre::Entity *entity = nullptr;
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
bool Visual::Wireframe() const
{
  return this->dataPtr->wireframe;
}

//////////////////////////////////////////////////
void Visual::SetTransparencyInnerLoop(Ogre::SceneNode *_sceneNode)
{
  float derivedTransparency = this->dataPtr->inheritTransparency ?
      this->DerivedTransparency() : this->dataPtr->transparency;

  for (unsigned int i = 0; i < _sceneNode->numAttachedObjects(); ++i)
  {
    Ogre::Entity *entity = nullptr;
    Ogre::MovableObject *obj = _sceneNode->getAttachedObject(i);

    entity = dynamic_cast<Ogre::Entity*>(obj);

    if (!entity)
      continue;

    // we may not need to check for this string anymore now that each visual
    // has a type
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

      // see if the original ogre material associated with this sub-entity
      // exists or not
      Ogre::MaterialPtr origMat;
      auto it = this->dataPtr->submeshMaterials.find(material->getName());
      if (it != this->dataPtr->submeshMaterials.end())
        origMat = it->second;

      for (techniqueCount = 0; techniqueCount < material->getNumTechniques();
           ++techniqueCount)
      {
        technique = material->getTechnique(techniqueCount);

        // get original material technique
        Ogre::Technique *origTechnique = nullptr;
        if (!origMat.isNull()
            && (techniqueCount < origMat->getNumTechniques()))
        {
          origTechnique = origMat->getTechnique(techniqueCount);
        }

        for (passCount = 0; passCount < technique->getNumPasses(); ++passCount)
        {
          pass = technique->getPass(passCount);

          // get original material pass
          Ogre::Pass *origPass = nullptr;
          if (origTechnique && passCount < origTechnique->getNumPasses())
          {
            origPass = origTechnique->getPass(passCount);
          }

          // account for the diffuse alpha value in the ogre material script
          // in addtion to the <transparency> value in sdf.
          float origPassAlpha = 1.0;
          if (origPass)
          {
            Ogre::ColourValue origPassDiffuse = origPass->getDiffuse();
            origPassAlpha = origPassDiffuse.a;
          }
          float passDerivedTransparency = 1.0f -
              (1.0f - derivedTransparency) * origPassAlpha;

          if (passDerivedTransparency > 0.0)
          {
            // set up ogre material pass to render transparent objects
            pass->setDepthWriteEnabled(false);
            if (!pass->isProgrammable() &&
                pass->getPolygonMode() == Ogre::PM_SOLID)
            {
              pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
            }
          }
          else
          {
            // restore original ogre material pass properties when transparency
            // is turned off
            bool depthWrite = true;
            if (origPass)
            {
              pass->setSceneBlending(origPass->getSourceBlendFactor(),
                  origPass->getDestBlendFactor());
              depthWrite = origPass->getDepthWriteEnabled();
            }
            pass->setDepthWriteEnabled(depthWrite);
          }

          dc = pass->getDiffuse();
          dc.a = (1.0f - passDerivedTransparency);
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
                  1.0 - passDerivedTransparency);
            }
          }
        }
      }
    }
  }
}

//////////////////////////////////////////////////
void Visual::UpdateTransparency(const bool _cascade)
{
  this->SetTransparencyInnerLoop(this->dataPtr->sceneNode);

  if (_cascade)
  {
    for (auto &child : this->dataPtr->children)
    {
      // Don't change some visualizations when link changes
      if (child->InheritTransparency() && !(this->GetType() == VT_LINK &&
          (child->GetType() == VT_GUI ||
           child->GetType() == VT_PHYSICS ||
           child->GetType() == VT_SENSOR)))
      {
        child->UpdateTransparency(_cascade);
      }
    }
  }

  if (this->dataPtr->useRTShader && this->dataPtr->scene->Initialized())
    RTShaderSystem::Instance()->UpdateShaders();

  this->dataPtr->sdf->GetElement("transparency")->Set(
      this->dataPtr->transparency);
}

//////////////////////////////////////////////////
void Visual::SetTransparency(float _trans)
{
  if (ignition::math::equal(this->dataPtr->transparency, _trans))
    return;

  this->dataPtr->transparency = std::min(
      std::max(_trans, static_cast<float>(0.0)), static_cast<float>(1.0));

  this->UpdateTransparency(true);
}

//////////////////////////////////////////////////
void Visual::SetInheritTransparency(const bool _inherit)
{
  this->dataPtr->inheritTransparency = _inherit;
}

//////////////////////////////////////////////////
bool Visual::InheritTransparency() const
{
  return this->dataPtr->inheritTransparency;
}

//////////////////////////////////////////////////
void Visual::SetHighlighted(bool _highlighted)
{
  if (_highlighted)
  {
    auto bbox = this->BoundingBox();

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
    for (auto child : this->dataPtr->children)
    {
      if (child->Name().find("LINK_FRAME_VISUAL__") != std::string::npos)
        child->SetHighlighted(_highlighted);
    }
  }
}

//////////////////////////////////////////////////
bool Visual::GetHighlighted() const
{
  if (this->dataPtr->boundingBox)
  {
    return this->dataPtr->boundingBox->Visible();
  }
  return false;
}

//////////////////////////////////////////////////
float Visual::GetTransparency()
{
  return this->dataPtr->transparency;
}

//////////////////////////////////////////////////
float Visual::DerivedTransparency() const
{
  if (!this->InheritTransparency())
    return this->dataPtr->transparency;

  float derivedTransparency = this->dataPtr->transparency;

  VisualPtr worldVis = this->dataPtr->scene->WorldVisual();
  VisualPtr vis = this->GetParent();

  while (vis && vis != worldVis)
  {
    derivedTransparency = 1 - ((1 - derivedTransparency) *
        (1 - vis->GetTransparency()));
    if (!vis->InheritTransparency())
      break;
    vis = vis->GetParent();
  }

  return derivedTransparency;
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
void Visual::SetPosition(const ignition::math::Vector3d &_pos)
{
  GZ_ASSERT(this->dataPtr->sceneNode, "Visual SceneNode is NULL");
  this->dataPtr->sceneNode->setPosition(_pos.X(), _pos.Y(), _pos.Z());

  this->dataPtr->sdf->GetElement("pose")->Set(this->Pose());
}

//////////////////////////////////////////////////
void Visual::SetRotation(const ignition::math::Quaterniond &_rot)
{
  GZ_ASSERT(this->dataPtr->sceneNode, "Visual SceneNode is null");
  this->dataPtr->sceneNode->setOrientation(
      Ogre::Quaternion(_rot.W(), _rot.X(), _rot.Y(), _rot.Z()));

  if(!dataPtr->poseElem)
  {
    dataPtr->poseElem = this->dataPtr->sdf->GetElement("pose");
  }

  dataPtr->poseElem->Set(this->Pose());
}

//////////////////////////////////////////////////
void Visual::SetPose(const ignition::math::Pose3d &_pose)
{
  this->dataPtr->sceneNode->setPosition(_pose.Pos().X(), _pose.Pos().Y(), _pose.Pos().Z());
  this->dataPtr->sceneNode->setOrientation(
      Ogre::Quaternion(_pose.Rot().W(), _pose.Rot().X(), _pose.Rot().Y(), _pose.Rot().Z()));
  if(!dataPtr->poseElem)
  {
    dataPtr->poseElem = this->dataPtr->sdf->GetElement("pose");
  }
  dataPtr->poseElem->Set(this->Pose());
}

//////////////////////////////////////////////////
ignition::math::Vector3d Visual::Position() const
{
  if (!this->dataPtr->sceneNode)
    return ignition::math::Vector3d::Zero;
  return Conversions::ConvertIgn(this->dataPtr->sceneNode->getPosition());
}

//////////////////////////////////////////////////
ignition::math::Quaterniond Visual::Rotation() const
{
  if (!this->dataPtr->sceneNode)
    return ignition::math::Quaterniond::Identity;
  return Conversions::ConvertIgn(this->dataPtr->sceneNode->getOrientation());
}

//////////////////////////////////////////////////
ignition::math::Pose3d Visual::Pose() const
{
  ignition::math::Pose3d pos;
  pos.Pos() = this->Position();
  pos.Rot() = this->Rotation();
  return pos;
}

//////////////////////////////////////////////////
ignition::math::Pose3d Visual::InitialRelativePose() const
{
  return this->dataPtr->initialRelativePose;
}

//////////////////////////////////////////////////
void Visual::SetWorldPose(const ignition::math::Pose3d &_pose)
{
  this->SetWorldPosition(_pose.Pos());
  this->SetWorldRotation(_pose.Rot());
}

//////////////////////////////////////////////////
void Visual::SetWorldPosition(const ignition::math::Vector3d &_pos)
{
  if (!this->dataPtr->sceneNode)
    return;
  this->dataPtr->sceneNode->_setDerivedPosition(Conversions::Convert(_pos));
}

//////////////////////////////////////////////////
void Visual::SetWorldRotation(const ignition::math::Quaterniond &_q)
{
  if (!this->dataPtr->sceneNode)
    return;
  this->dataPtr->sceneNode->_setDerivedOrientation(Conversions::Convert(_q));
}

//////////////////////////////////////////////////
ignition::math::Pose3d Visual::WorldPose() const
{
  ignition::math::Pose3d pose;

  Ogre::Vector3 vpos;
  Ogre::Quaternion vquatern;

  if (this->dataPtr->sceneNode)
  {
    vpos = this->dataPtr->sceneNode->_getDerivedPosition();
    pose.Pos().Set(vpos.x, vpos.y, vpos.z);

    vquatern = this->dataPtr->sceneNode->_getDerivedOrientation();
    pose.Rot().Set(vquatern.w, vquatern.x, vquatern.y, vquatern.z);
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
  if (this->dataPtr->useRTShader && this->dataPtr->scene->Initialized())
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
  if (this->dataPtr->useRTShader && this->dataPtr->scene->Initialized())
    RTShaderSystem::Instance()->UpdateShaders();
}


//////////////////////////////////////////////////
void Visual::SetRibbonTrail(bool _value,
    const ignition::math::Color &_initialColor,
    const ignition::math::Color &_changeColor)
{
  if (this->dataPtr->ribbonTrail == nullptr)
  {
    this->dataPtr->ribbonTrail =
        this->dataPtr->scene->OgreSceneManager()->createRibbonTrail(
        this->Name() + "_RibbonTrail");
    this->dataPtr->ribbonTrail->setMaterialName("Gazebo/RibbonTrail");
    // this->dataPtr->ribbonTrail->setTrailLength(100);
    this->dataPtr->ribbonTrail->setMaxChainElements(10000);
    // this->dataPtr->ribbonTrail->setNumberOfChains(1);
    this->dataPtr->ribbonTrail->setVisible(false);
    this->dataPtr->ribbonTrail->setCastShadows(false);
    this->dataPtr->ribbonTrail->setInitialWidth(0, 0.05);
    this->dataPtr->scene->OgreSceneManager()->getRootSceneNode()->attachObject(
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
  if (!this->dataPtr->preRenderConnection)
  {
    this->dataPtr->preRenderConnection = event::Events::ConnectPreRender(
        boost::bind(&Visual::Update, this));
  }

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
  _line->SetPoint(_index, this->WorldPose().Pos());
}

//////////////////////////////////////////////////
std::string Visual::GetMaterialName() const
{
  return this->dataPtr->myMaterialName;
}

//////////////////////////////////////////////////
ignition::math::AxisAlignedBox Visual::BoundingBox() const
{
  ignition::math::AxisAlignedBox box(
      ignition::math::Vector3d::Zero,
      ignition::math::Vector3d::Zero);
  this->BoundsHelper(this->GetSceneNode(), box);
  return box;
}

//////////////////////////////////////////////////
void Visual::BoundsHelper(Ogre::SceneNode *_node,
                          ignition::math::AxisAlignedBox &_box) const
{
  _node->_updateBounds();
  _node->_update(false, true);

  Ogre::Matrix4 invTransform =
      this->dataPtr->sceneNode->_getFullTransform().inverse();

  Ogre::SceneNode::ChildNodeIterator it = _node->getChildIterator();

  for (int i = 0; i < _node->numAttachedObjects(); i++)
  {
    Ogre::MovableObject *obj = _node->getAttachedObject(i);

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

      ignition::math::Vector3d min;
      ignition::math::Vector3d max;

      // Ogre does not return a valid bounding box for lights.
      if (obj->getMovableType() == "Light")
      {
        min = ignition::math::Vector3d(-0.5, -0.5, -0.5);
        max = ignition::math::Vector3d(0.5, 0.5, 0.5);
      }
      else
      {
        // Get transform to be applied to the current node.
        Ogre::Matrix4 transform = invTransform * _node->_getFullTransform();
        // Correct precision error which makes ogre's isAffine check fail.
        transform[3][0] = transform[3][1] = transform[3][2] = 0;
        transform[3][3] = 1;
        // get oriented bounding box in object's local space
#if OGRE_VERSION_MAJOR == 1 && OGRE_VERSION_MINOR >= 11
        bb.transform(transform);
#else
        bb.transformAffine(transform);
#endif

        min = Conversions::ConvertIgn(bb.getMinimum());
        max = Conversions::ConvertIgn(bb.getMaximum());
      }

      _box.Merge(ignition::math::AxisAlignedBox(min, max));
    }
  }

  while (it.hasMoreElements())
  {
    Ogre::SceneNode *next = dynamic_cast<Ogre::SceneNode*>(it.getNext());
    this->BoundsHelper(next, _box);
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

  GZ_ASSERT(_mesh != nullptr, "Unable to insert a null mesh");

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
      Ogre::HardwareVertexBufferSharedPtr texBuf;
      float *vertices;
      float *texMappings = nullptr;
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
      // allocate buffer for texture mapping, when doing animations, OGRE
      // requires the vertex position and normals reside in their own buffer,
      // see `https://ogrecave.github.io/ogre/api/1.11/_animation.html` under,
      // `Vertex buffer arrangements`.
      currOffset = 0;
      if (subMesh.GetTexCoordCount() > 0)
      {
        vertexDecl->addElement(1, currOffset, Ogre::VET_FLOAT2,
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

      if (subMesh.GetTexCoordCount() > 0)
      {
        texBuf = Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(
            vertexDecl->getVertexSize(1),
            vertexData->vertexCount,
            Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY,
            false);
      }

      vertexData->vertexBufferBinding->setBinding(0, vBuf);
      vertices = static_cast<float*>(vBuf->lock(
                      Ogre::HardwareBuffer::HBL_DISCARD));

      if (subMesh.GetTexCoordCount() > 0)
      {
        vertexData->vertexBufferBinding->setBinding(1, texBuf);
        texMappings = static_cast<float*>(texBuf->lock(
                        Ogre::HardwareBuffer::HBL_DISCARD));
      }

      if (_mesh->HasSkeleton())
      {
        if (subMesh.GetNodeAssignmentsCount() > 0)
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
        else
        {
          // When there is a skeleton associated with the mesh,
          // OGRE requires at least 1 bone assignment to compile the blend
          // weights.
          // The submeshs loaded from COLLADA may not have weights so we need
          // to add a dummy bone assignment for OGRE.
          Ogre::VertexBoneAssignment vba;
          vba.vertexIndex = 0;
          vba.boneIndex = 0;
          vba.weight = 0;
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
          *texMappings++ = subMesh.TexCoord(j).X();
          *texMappings++ = subMesh.TexCoord(j).Y();
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
      if (subMesh.GetTexCoordCount() > 0)
      {
        texBuf->unlock();
      }
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
      if (!this->dataPtr->scene->HasLayer(this->dataPtr->layer))
        rendering::Events::newLayer(this->dataPtr->layer);

      // Set invisible if this visual's layer is not active
      this->SetVisible(this->dataPtr->scene->LayerState(this->dataPtr->layer));
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
    {
      std::string filename = _msg->geometry().mesh().filename();
      newGeometryName = common::find_file(filename);
    }

    if (newGeometryType != geometryType ||
        (newGeometryType == "mesh" && !newGeometryName.empty() &&
        newGeometryName != geometryName))
    {
      std::string origMaterial = this->dataPtr->myMaterialName;
      bool origCastShadows = this->dataPtr->castShadows;

      sdf::ElementPtr geomElem = this->dataPtr->sdf->GetElement("geometry");
      geomElem->ClearElements();

      this->DetachObjects();

      Ogre::MovableObject *obj = nullptr;
      if (newGeometryType == "box" || newGeometryType == "cylinder" ||
          newGeometryType == "sphere" || newGeometryType == "plane")
      {
        obj = this->AttachMesh("unit_" + newGeometryType);
        sdf::ElementPtr shapeElem = geomElem->AddElement(newGeometryType);
        if (newGeometryType == "sphere" || newGeometryType == "cylinder")
          shapeElem->GetElement("radius")->Set(0.5);
      }
      else if (newGeometryType == "mesh")
      {
        std::string filename = _msg->geometry().mesh().filename();
        std::string meshName = newGeometryName;
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

        obj = this->AttachMesh(meshName, submeshName, centerSubmesh);

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
      Ogre::Entity *ent = static_cast<Ogre::Entity *>(obj);
      if (ent && ent->hasSkeleton())
        this->dataPtr->skeleton = ent->getSkeleton();
      this->SetMaterial(origMaterial, false);
      this->UpdateTransparency(true);
      this->SetCastShadows(origCastShadows);
    }

    ignition::math::Vector3d geomScale(1, 1, 1);

    if (_msg->geometry().type() == msgs::Geometry::BOX)
    {
      geomScale = msgs::ConvertIgn(_msg->geometry().box().size());
    }
    else if (_msg->geometry().type() == msgs::Geometry::CYLINDER)
    {
      geomScale.X(_msg->geometry().cylinder().radius() * 2.0);
      geomScale.Y(_msg->geometry().cylinder().radius() * 2.0);
      geomScale.Z(_msg->geometry().cylinder().length());
    }
    else if (_msg->geometry().type() == msgs::Geometry::SPHERE)
    {
      geomScale.X() = geomScale.Y() = geomScale.Z()
          = _msg->geometry().sphere().radius() * 2.0;
    }
    else if (_msg->geometry().type() == msgs::Geometry::PLANE)
    {
      if (_msg->geometry().plane().has_size())
      {
        geomScale.X(_msg->geometry().plane().size().x());
        geomScale.Y(_msg->geometry().plane().size().y());
      }
    }
    else if (_msg->geometry().type() == msgs::Geometry::IMAGE)
    {
      geomScale.X() = geomScale.Y() = geomScale.Z()
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
    this->ProcessMaterialMsg(_msg->material());
  }

  if (_msg->has_transparency())
  {
    this->SetTransparency(_msg->transparency());
  }

  // Note: sometimes a visual msg is received on the ~/visual topic
  // before the full scene msg, which results in a visual created without
  // its plugins loaded. So make sure we check the msg here and load the
  // plugins if not done already.
  if (!_msg->plugin().empty() && this->dataPtr->plugins.empty()
      && !this->dataPtr->sdf->HasElement("plugin"))
  {
    for (int i = 0; i < _msg->plugin_size(); ++i)
    {
      sdf::ElementPtr pluginElem;
      pluginElem = msgs::PluginToSDF(_msg->plugin(i), pluginElem);
      this->dataPtr->sdf->InsertElement(pluginElem);
    }
    this->LoadPlugins();
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
      p->GetParent() != this->dataPtr->scene->WorldVisual())
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
    return nullptr;

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

  rendering::VisualPtr world = this->dataPtr->scene->WorldVisual();
  rendering::VisualPtr vis = _visual->GetParent();
  while (vis)
  {
    if (vis->Name() == this->Name())
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

  rendering::VisualPtr world = this->dataPtr->scene->WorldVisual();
  rendering::VisualPtr vis = this->GetParent();
  while (vis)
  {
    if (vis->Name() == _visual->Name())
      return true;
    vis = vis->GetParent();
  }

  return false;
}

//////////////////////////////////////////////////
unsigned int Visual::GetDepth() const
{
  std::shared_ptr<const Visual> p = shared_from_this();
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
      std::string polyLineName = this->Name();
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

      if (meshManager->HasMesh(polyLineName))
        return polyLineName;
      else
        return std::string();
    }
    else if (geomElem->HasElement("mesh"))
    {
      sdf::ElementPtr meshElem = geomElem->GetElement("mesh");

      if (!meshElem->HasElement("uri"))
      {
        gzerr << "<uri> element missing for geometry element\n";
        return std::string();
      }

      auto uri = common::asFullPath(meshElem->Get<std::string>("uri"),
          meshElem->FilePath());

      auto filename = common::find_file(uri);

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
void Visual::MoveToPositions(const std::vector<ignition::math::Pose3d> &_pts,
                             const double _time,
                             std::function<void()> _onComplete)
{
  Ogre::TransformKeyFrame *key;
  auto start = this->WorldPose().Pos();

  this->dataPtr->onAnimationComplete = _onComplete;

  std::string animName = this->Name() + "_animation";

  Ogre::Animation *anim =
    this->dataPtr->sceneNode->getCreator()->createAnimation(animName, _time);
  anim->setInterpolationMode(Ogre::Animation::IM_SPLINE);

  Ogre::NodeAnimationTrack *strack = anim->createNodeTrack(0,
      this->dataPtr->sceneNode);

  key = strack->createNodeKeyFrame(0);
  key->setTranslate(Ogre::Vector3(start.X(), start.Y(), start.Z()));
  key->setRotation(this->dataPtr->sceneNode->getOrientation());

  double dt = _time / (_pts.size()-1);
  double tt = 0;
  for (unsigned int i = 0; i < _pts.size(); i++)
  {
    key = strack->createNodeKeyFrame(tt);
    key->setTranslate(Ogre::Vector3(
          _pts[i].Pos().X(), _pts[i].Pos().Y(), _pts[i].Pos().Z()));
    key->setRotation(Conversions::Convert(_pts[i].Rot()));

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
void Visual::MoveToPosition(const ignition::math::Pose3d &_pose, double _time)
{
  Ogre::TransformKeyFrame *key;
  auto start = this->WorldPose().Pos();
  ignition::math::Vector3d rpy = _pose.Rot().Euler();

  ignition::math::Quaterniond rotFinal(0, rpy.Y(), rpy.Z());

  std::string animName = this->Name() + "_animation";

  Ogre::Animation *anim =
    this->dataPtr->sceneNode->getCreator()->createAnimation(animName, _time);
  anim->setInterpolationMode(Ogre::Animation::IM_SPLINE);

  Ogre::NodeAnimationTrack *strack =
      anim->createNodeTrack(0, this->dataPtr->sceneNode);

  key = strack->createNodeKeyFrame(0);
  key->setTranslate(Ogre::Vector3(start.X(), start.Y(), start.Z()));
  key->setRotation(this->dataPtr->sceneNode->getOrientation());

  key = strack->createNodeKeyFrame(_time);
  key->setTranslate(Ogre::Vector3(_pose.Pos().X(), _pose.Pos().Y(),
        _pose.Pos().Z()));
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
  // If this is a collision visual, set it visible
  if (this->dataPtr->type == VT_COLLISION)
  {
    this->SetVisible(_show);
  }
  // If this is a link, check if there are pending collision visuals
  else if (_show && this->dataPtr->type == VT_LINK &&
      !this->dataPtr->pendingChildren.empty())
  {
    auto it = std::begin(this->dataPtr->pendingChildren);
    while (it != std::end(this->dataPtr->pendingChildren))
    {
      if (it->first != VT_COLLISION)
      {
        ++it;
        continue;
      }

      auto msg = dynamic_cast<msgs::Visual *>(it->second);
      if (!msg)
      {
        gzerr << "Wrong message to generate collision visual." << std::endl;
      }
      else if (!this->dataPtr->scene->GetVisual(msg->name()))
      {
        // Set orange transparent material
        msg->mutable_material()->mutable_script()->add_uri(
            "file://media/materials/scripts/gazebo.material");
        msg->mutable_material()->mutable_script()->set_name(
            "Gazebo/OrangeTransparent");
        msg->set_cast_shadows(false);

        // Create visual
        VisualPtr visual;
        visual.reset(new Visual(msg->name(), shared_from_this()));

        if (msg->has_id())
          visual->SetId(msg->id());

        auto msgPtr = new ConstVisualPtr(msg);
        visual->LoadFromMsg(*msgPtr);

        visual->SetType(it->first);
        visual->SetVisible(_show);
        visual->SetVisibilityFlags(GZ_VISIBILITY_GUI);
        visual->SetWireframe(this->dataPtr->scene->Wireframe());
      }

      delete msg;
      this->dataPtr->pendingChildren.erase(it);
    }
  }


  // Show for children
  for (auto &child : this->dataPtr->children)
  {
    child->ShowCollision(_show);
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

  if (this->Name().find("__SKELETON_VISUAL__") != std::string::npos)
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
  // If this is a joint visual, set it visible
  if (this->dataPtr->type == VT_PHYSICS &&
      this->Name().find("JOINT_VISUAL__") != std::string::npos)
  {
    this->SetVisible(_show);
  }
  // If this is a link, check if there are pending joint visuals
  else if (_show && this->dataPtr->type == VT_LINK &&
      !this->dataPtr->pendingChildren.empty())
  {
    auto it = std::begin(this->dataPtr->pendingChildren);
    while (it != std::end(this->dataPtr->pendingChildren))
    {
      if (it->first != VT_PHYSICS)
      {
        ++it;
        continue;
      }

      auto msg = dynamic_cast<const msgs::Joint *>(it->second);
      if (!msg)
      {
        ++it;
        continue;
      }

      std::string jointVisName = msg->name() + "_JOINT_VISUAL__";
      if (!this->dataPtr->scene->GetVisual(jointVisName))
      {
        JointVisualPtr jointVis(new JointVisual(jointVisName,
            shared_from_this()));

        auto msgPtr = new ConstJointPtr(msg);
        jointVis->Load(*msgPtr);

        jointVis->SetVisible(_show);
        if (msg->has_id())
          jointVis->SetId(msg->id());
      }

      delete msg;
      this->dataPtr->pendingChildren.erase(it);
    }
  }

  for (auto &child : this->dataPtr->children)
  {
    child->ShowJoints(_show);
  }
}

//////////////////////////////////////////////////
void Visual::ShowCOM(bool _show)
{
  // If this is a COM visual, set it visible
  if (this->dataPtr->type == VT_PHYSICS &&
      this->Name().find("COM_VISUAL__") != std::string::npos)
  {
    this->SetVisible(_show);
  }
  // If this is a link without COM visuals, create them
  else if (_show && this->dataPtr->type == VT_LINK &&
      !this->dataPtr->scene->GetVisual(this->Name() + "_COM_VISUAL__"))
  {
    auto msg = dynamic_cast<msgs::Link *>(this->dataPtr->typeMsg);
    if (!msg)
    {
      gzerr << "Couldn't get link message for visual [" << this->Name() <<
          "]" << std::endl;
      return;
    }
    auto msgPtr = new ConstLinkPtr(msg);

    COMVisualPtr vis(new COMVisual(this->Name() + "_COM_VISUAL__",
        shared_from_this()));
    vis->Load(*msgPtr);
    vis->SetVisible(_show);
  }

  // Show for children
  for (auto &child : this->dataPtr->children)
  {
    child->ShowCOM(_show);
  }
}

//////////////////////////////////////////////////
void Visual::ShowInertia(bool _show)
{
  std::string suffix("_INERTIA_VISUAL__");

  // If this is an inertia visual, set it visible
  if (this->dataPtr->type == VT_PHYSICS &&
      this->Name().find(suffix) != std::string::npos)
  {
    this->SetVisible(_show);
  }
  // If this is a link without inertia visuals, create them
  else if (_show && this->dataPtr->type == VT_LINK && this->dataPtr->typeMsg &&
      !this->dataPtr->scene->GetVisual(this->Name() + suffix))
  {
    auto msg = dynamic_cast<msgs::Link *>(this->dataPtr->typeMsg);
    if (!msg)
    {
      gzerr << "Couldn't get link message for visual [" << this->Name() <<
          "]" << std::endl;
      return;
    }
    auto msgPtr = new ConstLinkPtr(msg);

    InertiaVisualPtr vis(new InertiaVisual(this->Name() +
        suffix, shared_from_this()));
    vis->Load(*msgPtr);
    vis->SetVisible(_show);
  }

  // Show for children
  for (auto &child : this->dataPtr->children)
  {
    child->ShowInertia(_show);
  }
}

//////////////////////////////////////////////////
void Visual::ShowLinkFrame(bool _show)
{
  // If this is a link frame visual, set it visible
  if (this->dataPtr->type == VT_PHYSICS &&
      this->Name().find("LINK_FRAME_VISUAL__") != std::string::npos)
  {
    this->SetVisible(_show);
  }
  // If this is a link without link frame visuals, create them
  else if (_show && this->dataPtr->type == VT_LINK && this->dataPtr->typeMsg &&
    !this->dataPtr->scene->GetVisual(this->Name() + "_LINK_FRAME_VISUAL__"))
  {
    LinkFrameVisualPtr vis(new LinkFrameVisual(this->Name() +
        "_LINK_FRAME_VISUAL__", shared_from_this()));
    vis->Load();
    vis->SetVisible(_show);
  }

  // Show for children
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
    gzerr << "Visual " << this->Name() << " has no skeleton.\n";
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
      gzerr << "Visual[" << this->Name() << "] is attempting to load "
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
void Visual::SetLayer(const int32_t _layer)
{
  this->dataPtr->layer = _layer;

  // Set invisible if this visual's layer is not active
  this->SetVisible(this->dataPtr->scene->LayerState(this->dataPtr->layer));
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

//////////////////////////////////////////////////
bool Visual::UseRTShader() const
{
  return this->dataPtr->useRTShader;
}

//////////////////////////////////////////////////
void Visual::SetTypeMsg(const google::protobuf::Message *_msg)
{
  if (!_msg)
  {
    gzerr << "Null type message." << std::endl;
    return;
  }
  this->dataPtr->typeMsg = _msg->New();
  this->dataPtr->typeMsg->CopyFrom(*_msg);
}

//////////////////////////////////////////////////
void Visual::AddPendingChild(std::pair<VisualType,
    const google::protobuf::Message *> _pair)
{
  // Copy msg
  auto msg = _pair.second->New();
  msg->CopyFrom(*_pair.second);

  this->dataPtr->pendingChildren.push_back(std::make_pair(_pair.first, msg));
}

/////////////////////////////////////////////////
void Visual::ProcessMaterialMsg(const msgs::Material &_msg)
{
  if (_msg.has_lighting())
  {
    this->SetLighting(_msg.lighting());
  }

  if (_msg.has_script())
  {
    for (int i = 0; i < _msg.script().uri_size(); ++i)
    {
      RenderEngine::Instance()->AddResourcePath(
          _msg.script().uri(i));
    }
    if (_msg.script().has_name() &&
        !_msg.script().name().empty())
    {
      this->SetMaterial(_msg.script().name());
    }
  }

  if (_msg.has_ambient())
  {
    this->SetAmbient(ignition::math::Color(
          _msg.ambient().r(), _msg.ambient().g(), _msg.ambient().b(),
          _msg.ambient().a()));
  }

  if (_msg.has_diffuse())
  {
    this->SetDiffuse(ignition::math::Color(
          _msg.diffuse().r(), _msg.diffuse().g(), _msg.diffuse().b(),
          _msg.diffuse().a()));
  }

  if (_msg.has_specular())
  {
    this->SetSpecular(ignition::math::Color(
          _msg.specular().r(), _msg.specular().g(), _msg.specular().b(),
          _msg.specular().a()));
  }

  if (_msg.has_emissive())
  {
    this->SetEmissive(ignition::math::Color(
          _msg.emissive().r(), _msg.emissive().g(), _msg.emissive().b(),
          _msg.emissive().a()));
  }

  if (_msg.has_shader_type())
  {
    if (_msg.shader_type() == msgs::Material::VERTEX)
    {
      this->SetShaderType("vertex");
    }
    else if (_msg.shader_type() == msgs::Material::PIXEL)
    {
      this->SetShaderType("pixel");
    }
    else if (_msg.shader_type() ==
        msgs::Material::NORMAL_MAP_OBJECT_SPACE)
    {
      this->SetShaderType("normal_map_object_space");
    }
    else if (_msg.shader_type() ==
        msgs::Material::NORMAL_MAP_TANGENT_SPACE)
    {
      this->SetShaderType("normal_map_tangent_space");
    }
    else
    {
      gzerr << "Unrecognized shader type" << std::endl;
    }

    if (_msg.has_normal_map())
      this->SetNormalMap(_msg.normal_map());
  }
}

/////////////////////////////////////////////////
void Visual::ProcessMaterialMsg(const ignition::msgs::Material &_msg)
{
  this->SetLighting(_msg.lighting());

  if (_msg.has_script())
  {
    for (int i = 0; i < _msg.script().uri_size(); ++i)
    {
      RenderEngine::Instance()->AddResourcePath(
          _msg.script().uri(i));
    }
    if (!_msg.script().name().empty())
    {
      this->SetMaterial(_msg.script().name());
    }
  }

  if (_msg.has_ambient())
  {
    this->SetAmbient(ignition::math::Color(
          _msg.ambient().r(), _msg.ambient().g(), _msg.ambient().b(),
          _msg.ambient().a()));
  }

  if (_msg.has_diffuse())
  {
    this->SetDiffuse(ignition::math::Color(
          _msg.diffuse().r(), _msg.diffuse().g(), _msg.diffuse().b(),
          _msg.diffuse().a()));
  }

  if (_msg.has_specular())
  {
    this->SetSpecular(ignition::math::Color(
          _msg.specular().r(), _msg.specular().g(), _msg.specular().b(),
          _msg.specular().a()));
  }

  if (_msg.has_emissive())
  {
    this->SetEmissive(ignition::math::Color(
          _msg.emissive().r(), _msg.emissive().g(), _msg.emissive().b(),
          _msg.emissive().a()));
  }

  if (_msg.shader_type() == ignition::msgs::Material::VERTEX)
  {
    this->SetShaderType("vertex");
  }
  else if (_msg.shader_type() == ignition::msgs::Material::PIXEL)
  {
    this->SetShaderType("pixel");
  }
  else if (_msg.shader_type() ==
      ignition::msgs::Material::NORMAL_MAP_OBJECT_SPACE)
  {
    this->SetShaderType("normal_map_object_space");
  }
  else if (_msg.shader_type() ==
      ignition::msgs::Material::NORMAL_MAP_TANGENT_SPACE)
  {
    this->SetShaderType("normal_map_tangent_space");
  }
  else
  {
    gzerr << "Unrecognized shader type" << std::endl;
  }

  if (!_msg.normal_map().empty())
    this->SetNormalMap(_msg.normal_map());
}

/////////////////////////////////////////////////
void Visual::FillMaterialMsg(ignition::msgs::Material &_msg) const
{
  _msg.set_lighting(this->GetLighting());

  if (!this->dataPtr->origMaterialName.empty())
  {
    // \todo: Material URI's that are specific to a visual are not
    // recoverable. Refer to the Visual::ProcessMaterialMsg function
    _msg.mutable_script()->set_name(this->dataPtr->origMaterialName);
  }

  _msg.mutable_ambient()->set_r(this->dataPtr->ambient.R());
  _msg.mutable_ambient()->set_g(this->dataPtr->ambient.G());
  _msg.mutable_ambient()->set_b(this->dataPtr->ambient.B());
  _msg.mutable_ambient()->set_a(this->dataPtr->ambient.A());

  _msg.mutable_diffuse()->set_r(this->dataPtr->diffuse.R());
  _msg.mutable_diffuse()->set_g(this->dataPtr->diffuse.G());
  _msg.mutable_diffuse()->set_b(this->dataPtr->diffuse.B());
  _msg.mutable_diffuse()->set_a(this->dataPtr->diffuse.A());

  _msg.mutable_specular()->set_r(this->dataPtr->specular.R());
  _msg.mutable_specular()->set_g(this->dataPtr->specular.G());
  _msg.mutable_specular()->set_b(this->dataPtr->specular.B());
  _msg.mutable_specular()->set_a(this->dataPtr->specular.A());

  _msg.mutable_emissive()->set_r(this->dataPtr->emissive.R());
  _msg.mutable_emissive()->set_g(this->dataPtr->emissive.G());
  _msg.mutable_emissive()->set_b(this->dataPtr->emissive.B());
  _msg.mutable_emissive()->set_a(this->dataPtr->emissive.A());

  if (!this->GetNormalMap().empty())
    _msg.set_normal_map(this->GetNormalMap());

  if (this->GetShaderType().compare("vertex") == 0)
      _msg.set_shader_type(ignition::msgs::Material::VERTEX);
  else if (this->GetShaderType().compare("pixel") == 0)
      _msg.set_shader_type(ignition::msgs::Material::PIXEL);
  else if (this->GetShaderType().compare("normal_map_object_space") == 0)
      _msg.set_shader_type(ignition::msgs::Material::NORMAL_MAP_OBJECT_SPACE);
  else if (this->GetShaderType().compare("normal_map_tangent_space") == 0)
      _msg.set_shader_type(ignition::msgs::Material::NORMAL_MAP_TANGENT_SPACE);
  else if (!this->GetShaderType().empty())
    gzerr << "Unrecognized shader type[" << this->GetShaderType() << "]\n";
}
