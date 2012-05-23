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
#include <boost/lexical_cast.hpp>

#include "rendering/ogre.h"
#include "msgs/msgs.hh"
#include "sdf/sdf.hh"

#include "common/Exception.hh"
#include "common/Console.hh"

#include "rendering/Projector.hh"
#include "rendering/Heightmap.hh"
#include "rendering/RenderEvents.hh"
#include "rendering/LaserVisual.hh"
#include "rendering/CameraVisual.hh"
#include "rendering/JointVisual.hh"
#include "rendering/COMVisual.hh"
#include "rendering/ContactVisual.hh"
#include "rendering/Conversions.hh"
#include "rendering/Light.hh"
#include "rendering/Visual.hh"
#include "rendering/RenderEngine.hh"
#include "rendering/UserCamera.hh"
#include "rendering/Camera.hh"
#include "rendering/DepthCamera.hh"
#include "rendering/GpuLaser.hh"
#include "rendering/Grid.hh"
#include "rendering/SelectionObj.hh"
#include "rendering/DynamicLines.hh"
#include "rendering/RFIDVisual.hh"
#include "rendering/RFIDTagVisual.hh"

#include "rendering/RTShaderSystem.hh"
#include "transport/Transport.hh"
#include "transport/Node.hh"

#include "rendering/Scene.hh"

using namespace gazebo;
using namespace rendering;


uint32_t Scene::idCounter = 0;

struct VisualMessageLess {
    bool operator() (boost::shared_ptr<msgs::Visual const> _i,
                     boost::shared_ptr<msgs::Visual const> _j)
    {
      return _i->name().size() < _j->name().size();
    }
} VisualMessageLessOp;


//////////////////////////////////////////////////
Scene::Scene(const std::string &_name, bool _enableVisualizations)
{
  this->enableVisualizations = _enableVisualizations;
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(_name);
  this->id = idCounter++;
  this->idString = boost::lexical_cast<std::string>(this->id);

  this->name = _name;
  this->manager = NULL;
  this->raySceneQuery = NULL;

  this->receiveMutex = new boost::mutex();

  this->connections.push_back(
      event::Events::ConnectPreRender(boost::bind(&Scene::PreRender, this)));

  this->sceneSub = this->node->Subscribe("~/scene", &Scene::OnSceneMsg, this);
  this->sensorSub = this->node->Subscribe("~/sensor",
                                          &Scene::OnSensorMsg, this);
  this->visSub = this->node->Subscribe("~/visual", &Scene::OnVisualMsg, this);
  this->lightSub = this->node->Subscribe("~/light", &Scene::OnLightMsg, this);
  this->poseSub = this->node->Subscribe("~/pose/info", &Scene::OnPoseMsg, this);
  this->jointSub = this->node->Subscribe("~/joint", &Scene::OnJointMsg, this);
  this->skeletonPoseSub = this->node->Subscribe("~/skeleton_pose/info",
          &Scene::OnSkeletonPoseMsg, this);
  this->selectionSub = this->node->Subscribe("~/selection",
      &Scene::OnSelectionMsg, this);
  this->requestSub = this->node->Subscribe("~/request",
      &Scene::OnRequest, this);

  this->selectionObj = new SelectionObj(this);

  this->sdf.reset(new sdf::Element);
  sdf::initFile("sdf/scene.sdf", this->sdf);

  this->clearAll = false;
  this->heightmap = NULL;
}

//////////////////////////////////////////////////
void Scene::Clear()
{
  this->node->Fini();
  this->visualMsgs.clear();
  this->lightMsgs.clear();
  this->poseMsgs.clear();
  this->sceneMsgs.clear();
  this->jointMsgs.clear();
  this->linkMsgs.clear();
  this->cameras.clear();
  this->userCameras.clear();
  this->lights.clear();


  while (this->visuals.size() > 0)
    this->RemoveVisual(this->visuals.begin()->second);
  this->visuals.clear();

  for (uint32_t i = 0; i < this->grids.size(); i++)
    delete this->grids[i];
  this->grids.clear();

  this->sensorMsgs.clear();
  RTShaderSystem::Instance()->Clear();
}

//////////////////////////////////////////////////
Scene::~Scene()
{
  delete this->selectionObj;
  delete this->requestMsg;
  delete this->receiveMutex;
  delete this->raySceneQuery;

  this->node->Fini();
  this->node.reset();
  this->visSub.reset();
  this->lightSub.reset();
  this->poseSub.reset();
  this->jointSub.reset();
  this->skeletonPoseSub.reset();
  this->selectionSub.reset();
  this->responseSub.reset();
  this->requestSub.reset();
  this->requestPub.reset();


  Visual_M::iterator iter;
  this->visuals.clear();
  this->jointMsgs.clear();
  this->linkMsgs.clear();
  this->sceneMsgs.clear();
  this->poseMsgs.clear();
  this->lightMsgs.clear();
  this->visualMsgs.clear();

  this->worldVisual.reset();
  this->selectionMsg.reset();
  this->lights.clear();

  // Remove a scene
  RTShaderSystem::Instance()->RemoveScene(this);

  for (uint32_t i = 0; i < this->grids.size(); i++)
    delete this->grids[i];
  this->grids.clear();

  this->cameras.clear();
  this->userCameras.clear();

  if (this->manager)
  {
    RenderEngine::Instance()->root->destroySceneManager(this->manager);
    this->manager = NULL;
  }
  this->connections.clear();

  this->sdf->Reset();
  this->sdf.reset();
}

//////////////////////////////////////////////////
void Scene::Load(sdf::ElementPtr _sdf)
{
  this->sdf->Copy(_sdf);
  this->Load();
}

//////////////////////////////////////////////////
void Scene::Load()
{
  Ogre::Root *root = RenderEngine::Instance()->root;

  if (this->manager)
    root->destroySceneManager(this->manager);

  this->manager = root->createSceneManager(Ogre::ST_GENERIC);
}

//////////////////////////////////////////////////
VisualPtr Scene::GetWorldVisual() const
{
  return this->worldVisual;
}

//////////////////////////////////////////////////
void Scene::Init()
{
  this->worldVisual.reset(new Visual("__world_node__", shared_from_this()));

  RTShaderSystem::Instance()->AddScene(this);

  for (uint32_t i = 0; i < this->grids.size(); i++)
    this->grids[i]->Init();

  // Create the sky
  if (this->sdf->HasElement("sky"))
    this->SetSky(this->sdf->GetElement("sky")->GetValueString("material"));

  // Create Fog
  if (this->sdf->HasElement("fog"))
  {
    boost::shared_ptr<sdf::Element> fogElem = this->sdf->GetElement("fog");
    this->SetFog(fogElem->GetValueString("type"),
                  fogElem->GetValueColor("rgba"),
                  fogElem->GetValueDouble("density"),
                  fogElem->GetValueDouble("start"),
                  fogElem->GetValueDouble("end"));
  }

  // Create ray scene query
  this->raySceneQuery = this->manager->createRayQuery(Ogre::Ray());
  this->raySceneQuery->setSortByDistance(true);
  this->raySceneQuery->setQueryMask(Ogre::SceneManager::ENTITY_TYPE_MASK);

  // Force shadows on.
  sdf::ElementPtr shadowElem = this->sdf->GetOrCreateElement("shadows");
  shadowElem->GetAttribute("enabled")->Set(true);
  RTShaderSystem::Instance()->ApplyShadows(this);

  // Send a request to get the current world state
  this->requestPub = this->node->Advertise<msgs::Request>("~/request", 5, true);
  this->responseSub = this->node->Subscribe("~/response",
      &Scene::OnResponse, this, true);

  this->requestMsg = msgs::CreateRequest("scene_info");
  this->requestPub->Publish(*this->requestMsg);

  // Register this scene the the real time shaders system
  this->selectionObj->Init();

  /*this->projector = new Projector(this->worldVisual);
  this->projector->Load(math::Pose(0, 0 , 2, 0, 0, 0),
      "stereo_projection_pattern_high_res.png");
  this->projector->Toggle();
  */

  // TODO: Add GUI option to view all contacts
  /*ContactVisualPtr contactVis(new ContactVisual(
        "_GUIONLY_world_contact_vis",
        this->worldVisual, "~/physics/contacts"));
  this->visuals[contactVis->GetName()] = contactVis;
  */
}

//////////////////////////////////////////////////
Ogre::SceneManager *Scene::GetManager() const
{
  return this->manager;
}

//////////////////////////////////////////////////
std::string Scene::GetName() const
{
  return this->name;
}

//////////////////////////////////////////////////
void Scene::SetAmbientColor(const common::Color &color)
{
  sdf::ElementPtr elem = this->sdf->GetOrCreateElement("ambient");
  elem->GetAttribute("rgba")->Set(color);

  // Ambient lighting
  if (this->manager)
  {
    this->manager->setAmbientLight(
        Conversions::Convert(elem->GetValueColor("rgba")));
  }
}

//////////////////////////////////////////////////
common::Color Scene::GetAmbientColor() const
{
  sdf::ElementPtr elem = this->sdf->GetOrCreateElement("ambient");
  return elem->GetValueColor("rgba");
}

//////////////////////////////////////////////////
void Scene::SetBackgroundColor(const common::Color &color)
{
  sdf::ElementPtr elem = this->sdf->GetOrCreateElement("background");
  elem->GetAttribute("rgba")->Set(color);

  std::vector<CameraPtr>::iterator iter;
  for (iter = this->cameras.begin(); iter != this->cameras.end(); ++iter)
  {
    if ((*iter)->GetViewport())
      (*iter)->GetViewport()->setBackgroundColour(Conversions::Convert(color));
  }

  std::vector<UserCameraPtr>::iterator iter2;
  for (iter2 = this->userCameras.begin();
       iter2 != this->userCameras.end(); ++iter2)
  {
    if ((*iter2)->GetViewport())
      (*iter2)->GetViewport()->setBackgroundColour(Conversions::Convert(color));
  }
}

//////////////////////////////////////////////////
common::Color Scene::GetBackgroundColor() const
{
  sdf::ElementPtr elem = this->sdf->GetOrCreateElement("background");
  return elem->GetValueColor("rgba");
}

//////////////////////////////////////////////////
void Scene::CreateGrid(uint32_t cell_count, float cell_length,
                       float line_width, const common::Color &color)
{
  Grid *grid = new Grid(this, cell_count, cell_length, line_width, color);

  if (this->manager)
    grid->Init();

  this->grids.push_back(grid);
}

//////////////////////////////////////////////////
Grid *Scene::GetGrid(uint32_t index) const
{
  if (index >= this->grids.size())
  {
    gzerr << "Scene::GetGrid() Invalid index\n";
    return NULL;
  }

  return this->grids[index];
}

//////////////////////////////////////////////////
CameraPtr Scene::CreateCamera(const std::string &_name, bool _autoRender)
{
  CameraPtr camera(new Camera(this->name + "::" + _name, this, _autoRender));
  this->cameras.push_back(camera);

  return camera;
}

//////////////////////////////////////////////////
DepthCameraPtr Scene::CreateDepthCamera(const std::string &_name,
                                        bool _autoRender)
{
  DepthCameraPtr camera(new DepthCamera(this->name + "::" + _name,
        this, _autoRender));
  this->cameras.push_back(camera);

  return camera;
}

//////////////////////////////////////////////////
GpuLaserPtr Scene::CreateGpuLaser(const std::string &_name,
                                        bool _autoRender)
{
  GpuLaserPtr camera(new GpuLaser(this->name + "::" + _name,
        this, _autoRender));
  this->cameras.push_back(camera);

  return camera;
}

//////////////////////////////////////////////////
uint32_t Scene::GetCameraCount() const
{
  return this->cameras.size();
}

//////////////////////////////////////////////////
CameraPtr Scene::GetCamera(uint32_t index) const
{
  CameraPtr cam;

  if (index < this->cameras.size())
    cam = this->cameras[index];

  return cam;
}

//////////////////////////////////////////////////
CameraPtr Scene::GetCamera(const std::string &_name) const
{
  CameraPtr result;
  std::vector<CameraPtr>::const_iterator iter;
  for (iter = this->cameras.begin(); iter != this->cameras.end(); ++iter)
  {
    if ((*iter)->GetName() == _name)
      result = *iter;
  }

  return result;
}

//////////////////////////////////////////////////
UserCameraPtr Scene::CreateUserCamera(const std::string &name_)
{
  UserCameraPtr camera(new UserCamera(this->GetName() + "::" + name_, this));
  camera->Load();
  camera->Init();
  this->userCameras.push_back(camera);

  return camera;
}

//////////////////////////////////////////////////
uint32_t Scene::GetUserCameraCount() const
{
  return this->userCameras.size();
}

//////////////////////////////////////////////////
UserCameraPtr Scene::GetUserCamera(uint32_t index) const
{
  UserCameraPtr cam;

  if (index < this->userCameras.size())
    cam = this->userCameras[index];

  return cam;
}

//////////////////////////////////////////////////
VisualPtr Scene::CreateVisual(const std::string &_name)
{
  if (this->GetVisual(_name) != NULL)
  {
    gzerr << "Visual with name[" << _name << "] already exists\n";
    return VisualPtr();
  }

  VisualPtr result(new Visual(_name, this->worldVisual));
  result->Load();
  this->visuals[_name] = result;

  return result;
}

//////////////////////////////////////////////////
LightPtr Scene::GetLight(const std::string &_name) const
{
  LightPtr result;
  Light_M::const_iterator iter = this->lights.find(_name);
  if (iter != this->lights.end())
    result = iter->second;
  return result;
}

//////////////////////////////////////////////////
unsigned int Scene::GetLightCount() const
{
  return this->lights.size();
}

//////////////////////////////////////////////////
LightPtr Scene::GetLight(unsigned int _index) const
{
  LightPtr result;
  if (_index < this->lights.size())
  {
    Light_M::const_iterator iter = this->lights.begin();
    std::advance(iter, _index);
    result = iter->second;
  }
  else
  {
    gzerr << "Error: light index(" << _index << ") larger than light count("
          << this->lights.size() << "\n";
  }

  return result;
}

//////////////////////////////////////////////////
VisualPtr Scene::GetVisual(const std::string &_name) const
{
  VisualPtr result;
  Visual_M::const_iterator iter = this->visuals.find(_name);
  if (iter != this->visuals.end())
    result = iter->second;
  else
  {
    iter = this->visuals.find(this->GetName() + "::" + _name);
    if (iter != this->visuals.end())
      result = iter->second;
  }

  return result;
}

//////////////////////////////////////////////////
void Scene::SelectVisual(const std::string &_name) const
{
  VisualPtr vis = this->GetVisual(_name);
  if (vis)
    this->selectionObj->Attach(vis);
  else
    this->selectionObj->Clear();
}

//////////////////////////////////////////////////
VisualPtr Scene::SelectVisualAt(CameraPtr camera, math::Vector2i mousePos)
{
  std::string mod;
  VisualPtr vis = this->GetVisualAt(camera, mousePos, mod);
  if (vis)
    this->selectionObj->Attach(vis);
  else
    this->selectionObj->Clear();

  return vis;
}


//////////////////////////////////////////////////
VisualPtr Scene::GetVisualAt(CameraPtr camera, math::Vector2i mousePos,
                             std::string &_mod)
{
  VisualPtr visual;
  Ogre::Entity *closestEntity = this->GetOgreEntityAt(camera, mousePos, false);

  _mod = "";
  if (closestEntity)
  {
    // Make sure we set the _mod only if we have found a selection object
    if (closestEntity->getName().substr(0, 15) == "__SELECTION_OBJ" &&
        closestEntity->getUserAny().getType() == typeid(std::string))
      _mod = Ogre::any_cast<std::string>(closestEntity->getUserAny());

    visual = this->GetVisual(Ogre::any_cast<std::string>(
          closestEntity->getUserAny()));
  }

  return visual;
}

//////////////////////////////////////////////////
VisualPtr Scene::GetModelVisualAt(CameraPtr _camera, math::Vector2i _mousePos)
{
  VisualPtr vis = this->GetVisualAt(_camera, _mousePos);
  if (vis)
    vis = this->GetVisual(vis->GetName().substr(0, vis->GetName().find("::")));

  return vis;
}

//////////////////////////////////////////////////
void Scene::SnapVisualToNearestBelow(const std::string &_visualName)
{
  VisualPtr visBelow = this->GetVisualBelow(_visualName);
  VisualPtr vis = this->GetVisual(_visualName);

  if (vis && visBelow)
  {
    math::Vector3 pos = vis->GetWorldPose().pos;
    double dz = vis->GetBoundingBox().min.z - visBelow->GetBoundingBox().max.z;
    pos.z -= dz;
    vis->SetWorldPosition(pos);
  }
}

//////////////////////////////////////////////////
VisualPtr Scene::GetVisualBelow(const std::string &_visualName)
{
  VisualPtr result;
  VisualPtr vis = this->GetVisual(_visualName);

  if (vis)
  {
    std::vector<VisualPtr> below;
    this->GetVisualsBelowPoint(vis->GetWorldPose().pos, below);

    double maxZ = -10000;

    for (unsigned int i = 0; i < below.size(); ++i)
    {
      if (below[i]->GetName().find(vis->GetName()) != 0
          && below[i]->GetBoundingBox().max.z > maxZ)
      {
        maxZ = below[i]->GetBoundingBox().max.z;
        result = below[i];
      }
    }
  }

  return result;
}

//////////////////////////////////////////////////
void Scene::GetVisualsBelowPoint(const math::Vector3 &_pt,
                                 std::vector<VisualPtr> &_visuals)
{
  Ogre::Ray ray(Conversions::Convert(_pt), Ogre::Vector3(0, 0, -1));

  this->raySceneQuery->setRay(ray);
  this->raySceneQuery->setSortByDistance(true, 0);

  // Perform the scene query
  Ogre::RaySceneQueryResult &result = this->raySceneQuery->execute();
  Ogre::RaySceneQueryResult::iterator iter = result.begin();

  _visuals.clear();

  for (iter = result.begin(); iter != result.end(); ++iter)
  {
    // is the result a MovableObject
    if (iter->movable && iter->movable->getMovableType().compare("Entity") == 0)
    {
      if (!iter->movable->isVisible() ||
          iter->movable->getName().find("__COLLISION_VISUAL__") !=
          std::string::npos)
        continue;
      if (iter->movable->getName().substr(0, 15) == "__SELECTION_OBJ")
        continue;

      Ogre::Entity *pentity = static_cast<Ogre::Entity*>(iter->movable);
      if (pentity)
      {
        VisualPtr v = this->GetVisual(Ogre::any_cast<std::string>(
                                      pentity->getUserAny()));
        if (v)
          _visuals.push_back(v);
      }
    }
  }
}

//////////////////////////////////////////////////
VisualPtr Scene::GetVisualAt(CameraPtr camera, math::Vector2i mousePos)
{
  VisualPtr visual;

  Ogre::Entity *closestEntity = this->GetOgreEntityAt(camera, mousePos, true);
  if (closestEntity)
  {
    visual = this->GetVisual(Ogre::any_cast<std::string>(
          closestEntity->getUserAny()));
  }

  return visual;
}

/////////////////////////////////////////////////
Ogre::Entity *Scene::GetOgreEntityAt(CameraPtr _camera,
                                     math::Vector2i _mousePos,
                                     bool _ignoreSelectionObj)
{
  Ogre::Camera *ogreCam = _camera->GetOgreCamera();

  Ogre::Real closest_distance = -1.0f;
  Ogre::Ray mouseRay = ogreCam->getCameraToViewportRay(
      static_cast<float>(_mousePos.x) /
      ogreCam->getViewport()->getActualWidth(),
      static_cast<float>(_mousePos.y) /
      ogreCam->getViewport()->getActualHeight());

  this->raySceneQuery->setRay(mouseRay);

  // Perform the scene query
  Ogre::RaySceneQueryResult &result = this->raySceneQuery->execute();
  Ogre::RaySceneQueryResult::iterator iter = result.begin();
  Ogre::Entity *closestEntity = NULL;

  for (iter = result.begin(); iter != result.end(); ++iter)
  {
    // is the result a MovableObject
    if (iter->movable && iter->movable->getMovableType().compare("Entity") == 0)
    {
      if (!iter->movable->isVisible() ||
          iter->movable->getName().find("__COLLISION_VISUAL__") !=
          std::string::npos)
        continue;
      if (_ignoreSelectionObj &&
          iter->movable->getName().substr(0, 15) == "__SELECTION_OBJ")
        continue;

      Ogre::Entity *pentity = static_cast<Ogre::Entity*>(iter->movable);

      // mesh data to retrieve
      size_t vertex_count;
      size_t index_count;
      Ogre::Vector3 *vertices;
      uint64_t *indices;

      // Get the mesh information
      this->GetMeshInformation(pentity->getMesh().get(), vertex_count,
          vertices, index_count, indices,
          pentity->getParentNode()->_getDerivedPosition(),
          pentity->getParentNode()->_getDerivedOrientation(),
          pentity->getParentNode()->_getDerivedScale());

      bool new_closest_found = false;
      for (int i = 0; i < static_cast<int>(index_count); i += 3)
      {
        // when indices size is not divisible by 3
        if (i+2 >= static_cast<int>(index_count))
          break;

        // check for a hit against this triangle
        std::pair<bool, Ogre::Real> hit = Ogre::Math::intersects(mouseRay,
            vertices[indices[i]],
            vertices[indices[i+1]],
            vertices[indices[i+2]],
            true, false);

        // if it was a hit check if its the closest
        if (hit.first)
        {
          if ((closest_distance < 0.0f) || (hit.second < closest_distance))
          {
            // this is the closest so far, save it off
            closest_distance = hit.second;
            new_closest_found = true;
          }
        }
      }

      delete [] vertices;
      delete [] indices;

      if (new_closest_found)
      {
        closestEntity = pentity;
        // break;
      }
    }
  }

  return closestEntity;
}

//////////////////////////////////////////////////
math::Vector3 Scene::GetFirstContact(CameraPtr camera, math::Vector2i mousePos)
{
  Ogre::Camera *ogreCam = camera->GetOgreCamera();
  // Ogre::Real closest_distance = -1.0f;
  Ogre::Ray mouseRay = ogreCam->getCameraToViewportRay(
      static_cast<float>(mousePos.x) /
      ogreCam->getViewport()->getActualWidth(),
      static_cast<float>(mousePos.y) /
      ogreCam->getViewport()->getActualHeight());

  this->raySceneQuery->setRay(mouseRay);

  // Perform the scene query
  Ogre::RaySceneQueryResult &result = this->raySceneQuery->execute();
  Ogre::RaySceneQueryResult::iterator iter = result.begin();

  Ogre::Vector3 pt = mouseRay.getPoint(iter->distance);

  return math::Vector3(pt.x, pt.y, pt.z);
}

//////////////////////////////////////////////////
void Scene::PrintSceneGraph()
{
  this->PrintSceneGraphHelper("", this->manager->getRootSceneNode());
}

//////////////////////////////////////////////////
void Scene::PrintSceneGraphHelper(const std::string &prefix_, Ogre::Node *node_)
{
  Ogre::SceneNode *snode = dynamic_cast<Ogre::SceneNode*>(node_);

  std::string nodeName = node_->getName();
  int numAttachedObjs = 0;
  bool isInSceneGraph = false;
  if (snode)
  {
    numAttachedObjs = snode->numAttachedObjects();
    isInSceneGraph = snode->isInSceneGraph();
  }
  else
  {
    gzerr << "Invalid SceneNode\n";
    return;
  }

  int numChildren = node_->numChildren();
  Ogre::Vector3 pos = node_->getPosition();
  Ogre::Vector3 scale = node_->getScale();

  std::cout << prefix_ << nodeName << "\n";
  std::cout << prefix_ << "  Num Objs[" << numAttachedObjs << "]\n";
  for (int i = 0; i < numAttachedObjs; i++)
  {
    std::cout << prefix_
      << "    Obj[" << snode->getAttachedObject(i)->getName() << "]\n";
  }
  std::cout << prefix_ << "  Num Children[" << numChildren << "]\n";
  std::cout << prefix_ << "  IsInGraph[" << isInSceneGraph << "]\n";
  std::cout << prefix_
    << "  Pos[" << pos.x << " " << pos.y << " " << pos.z << "]\n";
  std::cout << prefix_
    << "  Scale[" << scale.x << " " << scale.y << " " << scale.z << "]\n";

  for (uint32_t i = 0; i < node_->numChildren(); i++)
  {
    this->PrintSceneGraphHelper(prefix_ + "  ", node_->getChild(i));
  }
}

//////////////////////////////////////////////////
void Scene::DrawLine(const math::Vector3 &start_,
                     const math::Vector3 &end_,
                     const std::string &name_)
{
  Ogre::SceneNode *sceneNode = NULL;
  Ogre::ManualObject *obj = NULL;
  bool attached = false;

  if (this->manager->hasManualObject(name_))
  {
    sceneNode = this->manager->getSceneNode(name_);
    obj = this->manager->getManualObject(name_);
    attached = true;
  }
  else
  {
    sceneNode = this->manager->getRootSceneNode()->createChildSceneNode(name_);
    obj = this->manager->createManualObject(name_);
  }

  sceneNode->setVisible(true);
  obj->setVisible(true);

  obj->clear();
  obj->begin("Gazebo/Red", Ogre::RenderOperation::OT_LINE_LIST);
  obj->position(start_.x, start_.y, start_.z);
  obj->position(end_.x, end_.y, end_.z);
  obj->end();

  if (!attached)
    sceneNode->attachObject(obj);
}

//////////////////////////////////////////////////
void Scene::SetFog(const std::string &_type, const common::Color &_color,
                    double _density, double _start, double _end)
{
  Ogre::FogMode fogType = Ogre::FOG_NONE;

  if (_type == "linear")
    fogType = Ogre::FOG_LINEAR;
  else if (_type == "exp")
    fogType = Ogre::FOG_EXP;
  else if (_type == "exp2")
    fogType = Ogre::FOG_EXP2;

  sdf::ElementPtr elem = this->sdf->GetOrCreateElement("fog");

  elem->GetAttribute("type")->Set(_type);
  elem->GetAttribute("rgba")->Set(_color);
  elem->GetAttribute("density")->Set(_density);
  elem->GetAttribute("start")->Set(_start);
  elem->GetAttribute("end")->Set(_end);

  if (this->manager)
    this->manager->setFog(fogType, Conversions::Convert(_color),
                           _density, _start, _end);
}

//////////////////////////////////////////////////
void Scene::SetVisible(const std::string &name_, bool visible_)
{
  if (this->manager->hasSceneNode(name_))
    this->manager->getSceneNode(name_)->setVisible(visible_);

  if (this->manager->hasManualObject(name_))
    this->manager->getManualObject(name_)->setVisible(visible_);
}

//////////////////////////////////////////////////
uint32_t Scene::GetId() const
{
  return this->id;
}

//////////////////////////////////////////////////
std::string Scene::GetIdString() const
{
  return this->idString;
}


//////////////////////////////////////////////////
void Scene::GetMeshInformation(const Ogre::Mesh *mesh,
                               size_t &vertex_count,
                               Ogre::Vector3* &vertices,
                               size_t &index_count,
                               uint64_t* &indices,
                               const Ogre::Vector3 &position,
                               const Ogre::Quaternion &orient,
                               const Ogre::Vector3 &scale)
{
  bool added_shared = false;
  size_t current_offset = 0;
  size_t next_offset = 0;
  size_t index_offset = 0;

  vertex_count = index_count = 0;

  // Calculate how many vertices and indices we're going to need
  for (uint16_t i = 0; i < mesh->getNumSubMeshes(); ++i)
  {
    Ogre::SubMesh* submesh = mesh->getSubMesh(i);

    // We only need to add the shared vertices once
    if (submesh->useSharedVertices)
    {
      if (!added_shared)
      {
        vertex_count += mesh->sharedVertexData->vertexCount;
        added_shared = true;
      }
    }
    else
    {
      vertex_count += submesh->vertexData->vertexCount;
    }

    // Add the indices
    index_count += submesh->indexData->indexCount;
  }


  // Allocate space for the vertices and indices
  vertices = new Ogre::Vector3[vertex_count];
  indices = new uint64_t[index_count];

  added_shared = false;

  // Run through the submeshes again, adding the data into the arrays
  for (uint16_t i = 0; i < mesh->getNumSubMeshes(); ++i)
  {
    Ogre::SubMesh* submesh = mesh->getSubMesh(i);

    Ogre::VertexData* vertex_data =
      submesh->useSharedVertices ? mesh->sharedVertexData : submesh->vertexData;

    if ((!submesh->useSharedVertices) ||
        (submesh->useSharedVertices && !added_shared))
    {
      if (submesh->useSharedVertices)
      {
        added_shared = true;
      }

      const Ogre::VertexElement* posElem =
        vertex_data->vertexDeclaration->findElementBySemantic(
            Ogre::VES_POSITION);

      Ogre::HardwareVertexBufferSharedPtr vbuf =
        vertex_data->vertexBufferBinding->getBuffer(posElem->getSource());

      unsigned char *vertex =
        static_cast<unsigned char*>(
            vbuf->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));

      // There is _no_ baseVertexPointerToElement() which takes an
      // Ogre::Real or a double as second argument. So make it float,
      // to avoid trouble when Ogre::Real will be comiled/typedefed as double:
      //      Ogre::Real* pReal;
      float *pReal;

      for (size_t j = 0; j < vertex_data->vertexCount;
           ++j, vertex += vbuf->getVertexSize())
      {
        posElem->baseVertexPointerToElement(vertex, &pReal);
        Ogre::Vector3 pt(pReal[0], pReal[1], pReal[2]);
        vertices[current_offset + j] = (orient * (pt * scale)) + position;
      }

      vbuf->unlock();
      next_offset += vertex_data->vertexCount;
    }

    Ogre::IndexData* index_data = submesh->indexData;
    Ogre::HardwareIndexBufferSharedPtr ibuf = index_data->indexBuffer;

    if ((ibuf->getType() == Ogre::HardwareIndexBuffer::IT_32BIT))
    {
      uint32_t*  pLong = static_cast<uint32_t*>(
          ibuf->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));

      for (size_t k = 0; k < index_data->indexCount; k++)
      {
        indices[index_offset++] = pLong[k];
      }
    }
    else
    {
      uint64_t*  pLong = static_cast<uint64_t*>(
          ibuf->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));

      uint16_t* pShort = reinterpret_cast<uint16_t*>(pLong);
      for (size_t k = 0; k < index_data->indexCount; k++)
      {
        indices[index_offset++] = static_cast<uint64_t>(pShort[k]);
      }
    }

    ibuf->unlock();
    current_offset = next_offset;
  }
}

/////////////////////////////////////////////////
void Scene::OnResponse(ConstResponsePtr &_msg)
{
  if (!this->requestMsg || _msg->id() != this->requestMsg->id())
    return;

  boost::shared_ptr<msgs::Scene> sm(new msgs::Scene());
  boost::mutex::scoped_lock lock(*this->receiveMutex);
  if (_msg->has_type() && _msg->type() == sm->GetTypeName())
  {
    sm->ParseFromString(_msg->serialized_data());
    this->sceneMsgs.push_back(sm);
  }
}

/////////////////////////////////////////////////
void Scene::ProcessSceneMsg(ConstScenePtr &_msg)
{
  std::string modelName, linkName;
  for (int i = 0; i < _msg->model_size(); i++)
  {
    modelName = _msg->model(i).name() + "::";
    boost::shared_ptr<msgs::Pose> pm(new msgs::Pose(_msg->model(i).pose()));
    pm->set_name(_msg->model(i).name());
    this->poseMsgs.push_front(pm);

    for (int j = 0; j < _msg->model(i).visual_size(); j++)
    {
      boost::shared_ptr<msgs::Visual> vm(new msgs::Visual(
            _msg->model(i).visual(j)));
      this->visualMsgs.push_back(vm);
    }

    for (int j = 0; j < _msg->model(i).joint_size(); j++)
    {
      boost::shared_ptr<msgs::Joint> jm(new msgs::Joint(
            _msg->model(i).joint(j)));
      this->jointMsgs.push_back(jm);
    }

    for (int j = 0; j < _msg->model(i).link_size(); j++)
    {
      linkName = modelName +_msg->model(i).link(j).name();
      boost::shared_ptr<msgs::Pose> pm2(
          new msgs::Pose(_msg->model(i).link(j).pose()));
      pm2->set_name(linkName);
      this->poseMsgs.push_front(pm2);

      if (_msg->model(i).link(j).has_inertial())
      {
        boost::shared_ptr<msgs::Link> lm(
            new msgs::Link(_msg->model(i).link(j)));
        this->linkMsgs.push_back(lm);
      }

      for (int k = 0; k < _msg->model(i).link(j).visual_size(); k++)
      {
        boost::shared_ptr<msgs::Visual> vm(new msgs::Visual(
              _msg->model(i).link(j).visual(k)));
        this->visualMsgs.push_back(vm);
      }

      for (int k = 0; k < _msg->model(i).link(j).collision_size(); k++)
      {
        for (int l = 0;
             l < _msg->model(i).link(j).collision(k).visual_size(); l++)
        {
          boost::shared_ptr<msgs::Visual> vm(new msgs::Visual(
                _msg->model(i).link(j).collision(k).visual(l)));
          this->visualMsgs.push_back(vm);
        }
      }

      for (int k = 0; k < _msg->model(i).link(j).sensor_size(); k++)
      {
        boost::shared_ptr<msgs::Sensor> sm(new msgs::Sensor(
              _msg->model(i).link(j).sensor(k)));
        this->sensorMsgs.push_back(sm);
      }
    }
  }

  for (int i = 0; i < _msg->light_size(); i++)
  {
    boost::shared_ptr<msgs::Light> lm(new msgs::Light(_msg->light(i)));
    this->lightMsgs.push_back(lm);
  }

  for (int i = 0; i < _msg->joint_size(); i++)
  {
    boost::shared_ptr<msgs::Joint> jm(new msgs::Joint(_msg->joint(i)));
    this->jointMsgs.push_back(jm);
  }

  if (_msg->has_ambient())
    this->SetAmbientColor(msgs::Convert(_msg->ambient()));

  if (_msg->has_background())
    this->SetBackgroundColor(msgs::Convert(_msg->background()));

  if (_msg->has_sky_material())
    this->SetSky(_msg->sky_material());

  if (_msg->has_shadows())
    this->SetShadowsEnabled(_msg->shadows());

  if (_msg->has_grid())
    this->SetGrid(_msg->grid());

  if (_msg->has_fog())
  {
    sdf::ElementPtr elem = this->sdf->GetOrCreateElement("fog");

    if (_msg->fog().has_color())
      elem->GetAttribute("rgba")->Set(msgs::Convert(_msg->fog().color()));

    if (_msg->fog().has_density())
      elem->GetAttribute("density")->Set(_msg->fog().density());

    if (_msg->fog().has_start())
      elem->GetAttribute("start")->Set(_msg->fog().start());

    if (_msg->fog().has_end())
      elem->GetAttribute("end")->Set(_msg->fog().end());

    if (_msg->fog().has_type())
    {
      std::string type;
      if (_msg->fog().type() == msgs::Fog::LINEAR)
        type = "linear";
      else if (_msg->fog().type() == msgs::Fog::EXPONENTIAL)
        type = "exp";
      else if (_msg->fog().type() == msgs::Fog::EXPONENTIAL2)
        type = "exp2";
      else
        type = "none";

      elem->GetAttribute("type")->Set(type);
    }

    this->SetFog(elem->GetValueString("type"),
                  elem->GetValueColor("rgba"),
                  elem->GetValueDouble("density"),
                  elem->GetValueDouble("start"),
                  elem->GetValueDouble("end"));
  }
}

//////////////////////////////////////////////////
void Scene::OnSensorMsg(ConstSensorPtr &_msg)
{
  boost::mutex::scoped_lock lock(*this->receiveMutex);
  this->sensorMsgs.push_back(_msg);
}

//////////////////////////////////////////////////
void Scene::OnSceneMsg(ConstScenePtr &_msg)
{
  boost::mutex::scoped_lock lock(*this->receiveMutex);
  this->sceneMsgs.push_back(_msg);
}

//////////////////////////////////////////////////
void Scene::OnVisualMsg(ConstVisualPtr &_msg)
{
  boost::mutex::scoped_lock lock(*this->receiveMutex);
  this->visualMsgs.push_back(_msg);
}

//////////////////////////////////////////////////
void Scene::PreRender()
{
  boost::mutex::scoped_lock lock(*this->receiveMutex);

  static RequestMsgs_L::iterator rIter;
  static SceneMsgs_L::iterator sIter;
  static VisualMsgs_L::iterator vIter;
  static LightMsgs_L::iterator lIter;
  static PoseMsgs_L::iterator pIter;
  static SkeletonPoseMsgs_L::iterator spIter;
  static JointMsgs_L::iterator jIter;
  static SensorMsgs_L::iterator sensorIter;
  static LinkMsgs_L::iterator linkIter;

  // Process the scene messages. DO THIS FIRST
  for (sIter = this->sceneMsgs.begin();
       sIter != this->sceneMsgs.end(); ++sIter)
  {
    this->ProcessSceneMsg(*sIter);
  }
  this->sceneMsgs.clear();

  sensorIter = this->sensorMsgs.begin();
  while (sensorIter != this->sensorMsgs.end())
  {
    if (this->ProcessSensorMsg(*sensorIter))
      this->sensorMsgs.erase(sensorIter++);
    else
      ++sensorIter;
  }

  // Process the visual messages
  this->visualMsgs.sort(VisualMessageLessOp);
  vIter = this->visualMsgs.begin();
  while (vIter != this->visualMsgs.end())
  {
    if (this->ProcessVisualMsg(*vIter))
      this->visualMsgs.erase(vIter++);
    else
      ++vIter;
  }

  // Process the light messages
  for (lIter =  this->lightMsgs.begin();
       lIter != this->lightMsgs.end(); ++lIter)
  {
    this->ProcessLightMsg(*lIter);
  }
  this->lightMsgs.clear();

  // Process all the model messages last. Remove pose message from the list
  // only when a corresponding visual exits. We may receive pose updates
  // over the wire before  we recieve the visual
  pIter = this->poseMsgs.begin();
  while (pIter != this->poseMsgs.end())
  {
    Visual_M::iterator iter = this->visuals.find((*pIter)->name());
    if (iter != this->visuals.end())
    {
      // If an object is selected, don't let the physics engine move it.
      if (this->selectionObj->GetVisualName().empty() ||
          !this->selectionObj->IsActive() ||
          iter->first.find(this->selectionObj->GetVisualName()) ==
          std::string::npos)
      {
        math::Pose pose = msgs::Convert(*(*pIter));
        iter->second->SetPose(pose);
      }
      PoseMsgs_L::iterator prev = pIter++;
      this->poseMsgs.erase(prev);
    }
    else
      ++pIter;
  }

  // process skeleton pose msgs
  spIter = this->skeletonPoseMsgs.begin();
  while (spIter != this->skeletonPoseMsgs.end())
  {
    Visual_M::iterator iter = this->visuals.find((*spIter)->model_name());
    for (int i = 0; i < (*spIter)->pose_size(); i++)
    {
      const msgs::Pose& pose_msg = (*spIter)->pose(i);
      Visual_M::iterator iter2 = this->visuals.find(pose_msg.name());
      if (iter2 != this->visuals.end())
      {
        // If an object is selected, don't let the physics engine move it.
        if (this->selectionObj->GetVisualName().empty() ||
           !this->selectionObj->IsActive() ||
           iter->first.find(this->selectionObj->GetVisualName()) ==
            std::string::npos)
        {
          math::Pose pose = msgs::Convert(pose_msg);
          iter2->second->SetPose(pose);
        }
      }
    }

    if (iter != this->visuals.end())
    {
      iter->second->SetSkeletonPose(*(*spIter).get());
      SkeletonPoseMsgs_L::iterator prev = spIter++;
      this->skeletonPoseMsgs.erase(prev);
    }
    else
      ++spIter;
  }

  // Process the request messages
  for (rIter =  this->requestMsgs.begin();
       rIter != this->requestMsgs.end(); ++rIter)
  {
    this->ProcessRequestMsg(*rIter);
  }
  this->requestMsgs.clear();

  // Process the joint messages
  jIter = this->jointMsgs.begin();
  while (jIter != this->jointMsgs.end())
  {
    if (this->ProcessJointMsg(*jIter))
      this->jointMsgs.erase(jIter++);
    else
      ++jIter;
  }

  // Process the link messages
  linkIter = this->linkMsgs.begin();
  while (linkIter != this->linkMsgs.end())
  {
    if (this->ProcessLinkMsg(*linkIter))
      this->linkMsgs.erase(linkIter++);
    else
      ++linkIter;
  }

  if (this->selectionMsg)
  {
    this->SelectVisual(this->selectionMsg->name());
    this->selectionMsg.reset();
  }
}

/////////////////////////////////////////////////
void Scene::OnJointMsg(ConstJointPtr &_msg)
{
  boost::mutex::scoped_lock lock(*this->receiveMutex);
  this->jointMsgs.push_back(_msg);
}

/////////////////////////////////////////////////
bool Scene::ProcessSensorMsg(ConstSensorPtr &_msg)
{
  if (!this->enableVisualizations)
    return true;

  if (_msg->type() == "ray" && _msg->visualize() && !_msg->topic().empty())
  {
    if (!this->visuals[_msg->name()+"_laser_vis"])
    {
      VisualPtr parentVis = this->GetVisual(_msg->parent());
      if (!parentVis)
        return false;

      LaserVisualPtr laserVis(new LaserVisual(
            _msg->name()+"_GUIONLY_laser_vis", parentVis, _msg->topic()));
      laserVis->Load();
      this->visuals[_msg->name()+"_laser_vis"] = laserVis;
    }
  }
  else if (_msg->type() == "camera" && _msg->visualize())
  {
    VisualPtr parentVis = this->GetVisual(_msg->parent());
    if (!parentVis)
      return false;

    CameraVisualPtr cameraVis(new CameraVisual(
          _msg->name()+"_GUIONLY_camera_vis", parentVis));

    cameraVis->SetPose(msgs::Convert(_msg->pose()));

    cameraVis->Load(_msg->camera().image_size().x(),
                    _msg->camera().image_size().y());

    this->visuals[cameraVis->GetName()] = cameraVis;
  }
  else if (_msg->type() == "contact" && _msg->visualize() &&
           !_msg->topic().empty())
  {
    ContactVisualPtr contactVis(new ContactVisual(
          _msg->name()+"_GUIONLY_contact_vis",
          this->worldVisual, _msg->topic()));

    this->visuals[contactVis->GetName()] = contactVis;
  }
  else if (_msg->type() == "rfidtag" && _msg->visualize() &&
           !_msg->topic().empty())
  {
    VisualPtr parentVis = this->GetVisual(_msg->parent());
    if (!parentVis)
      return false;

    RFIDTagVisualPtr rfidVis(new RFIDTagVisual(
          _msg->name() + "_GUIONLY_rfidtag_vis", parentVis, _msg->topic()));

    this->visuals[rfidVis->GetName()] = rfidVis;
  }
  else if (_msg->type() == "rfid" && _msg->visualize() &&
           !_msg->topic().empty())
  {
    VisualPtr parentVis = this->GetVisual(_msg->parent());
    if (!parentVis)
      return false;

    RFIDVisualPtr rfidVis(new RFIDVisual(
          _msg->name() + "_GUIONLY_rfid_vis", parentVis, _msg->topic()));
    this->visuals[rfidVis->GetName()] = rfidVis;
  }

  return true;
}

/////////////////////////////////////////////////
bool Scene::ProcessLinkMsg(ConstLinkPtr &_msg)
{
  VisualPtr linkVis = this->GetVisual(_msg->name());

  if (!linkVis)
    return false;

  COMVisualPtr comVis(new COMVisual(_msg->name() + "_COM_VISUAL__", linkVis));
  comVis->Load(_msg);
  comVis->SetVisible(false);
  this->visuals[comVis->GetName()] = comVis;

  for (int i = 0; i < _msg->projector_size(); ++i)
  {
    Projector *projector = new Projector(linkVis);
    projector->Load(_msg->projector(i));
    projector->Toggle();
    this->projectors.push_back(projector);
  }

  return true;
}

/////////////////////////////////////////////////
bool Scene::ProcessJointMsg(ConstJointPtr &_msg)
{
  VisualPtr childVis;

  if (_msg->child() == "world")
    childVis = this->worldVisual;
  else
    childVis = this->GetVisual(_msg->child());

  if (!childVis)
    return false;

  JointVisualPtr jointVis(new JointVisual(
          _msg->name() + "_JOINT_VISUAL__", childVis));
  jointVis->Load(_msg);
  jointVis->SetVisible(false);

  this->visuals[jointVis->GetName()] = jointVis;
  return true;
}

/////////////////////////////////////////////////
void Scene::OnRequest(ConstRequestPtr &_msg)
{
  boost::mutex::scoped_lock lock(*this->receiveMutex);
  this->requestMsgs.push_back(_msg);
}

/////////////////////////////////////////////////
void Scene::ProcessRequestMsg(ConstRequestPtr &_msg)
{
  if (_msg->request() == "entity_delete")
  {
    Visual_M::iterator iter;
    iter = this->visuals.find(_msg->data());
    if (iter != this->visuals.end())
    {
      this->RemoveVisual(iter->second);
    }
  }
  else if (_msg->request() == "show_collision")
  {
    VisualPtr vis = this->GetVisual(_msg->data());
    if (vis)
      vis->ShowCollision(true);
    else
      gzerr << "Unable to find visual[" << _msg->data() << "]\n";
  }
  else if (_msg->request() == "hide_collision")
  {
    VisualPtr vis = this->GetVisual(_msg->data());
    if (vis)
      vis->ShowCollision(false);
  }
  else if (_msg->request() == "show_joints")
  {
    VisualPtr vis = this->GetVisual(_msg->data());
    if (vis)
      vis->ShowJoints(true);
    else
      gzerr << "Unable to find joint visual[" << _msg->data() << "]\n";
  }
  else if (_msg->request() == "hide_joints")
  {
    VisualPtr vis = this->GetVisual(_msg->data());
    if (vis)
      vis->ShowJoints(false);
  }
  else if (_msg->request() == "show_com")
  {
    VisualPtr vis = this->GetVisual(_msg->data());
    if (vis)
      vis->ShowCOM(true);
    else
      gzerr << "Unable to find joint visual[" << _msg->data() << "]\n";
  }
  else if (_msg->request() == "hide_com")
  {
    VisualPtr vis = this->GetVisual(_msg->data());
    if (vis)
      vis->ShowCOM(false);
  }
  else if (_msg->request() == "set_transparency")
  {
    VisualPtr vis = this->GetVisual(_msg->data());
    if (vis)
      vis->SetTransparency(_msg->dbl_data());
  }
  else if (_msg->request() == "show_skeleton")
  {
    VisualPtr vis = this->GetVisual(_msg->data());
    bool show = (math::equal(_msg->dbl_data(), 1.0)) ? true : false;
      if (vis)
        vis->ShowSkeleton(show);
  }
}

/////////////////////////////////////////////////
bool Scene::ProcessVisualMsg(ConstVisualPtr &_msg)
{
  bool result = false;
  Visual_M::iterator iter;
  iter = this->visuals.find(_msg->name());

  if (_msg->has_delete_me() && _msg->delete_me())
  {
    if (iter != this->visuals.end())
    {
      this->visuals.erase(iter);
      result = true;
    }
  }
  else if (iter != this->visuals.end())
  {
    iter->second->UpdateFromMsg(_msg);
    result = true;
  }
  else
  {
    VisualPtr visual;

    // TODO: A bit of a hack.
    if (_msg->has_geometry() &&
        _msg->geometry().type() == msgs::Geometry::HEIGHTMAP)
    {
      // Ignore collision visuals for the heightmap
      if (_msg->name().find("__COLLISION_VISUAL__") == std::string::npos &&
          this->heightmap == NULL)
      {
        try
        {
          this->heightmap = new Heightmap(shared_from_this());
          this->heightmap->LoadFromMsg(_msg);
        } catch(...)
        {
          return false;
        }
      }
      return true;
    }

    // If the visual has a parent which is not the name of the scene...
    if (_msg->has_parent_name() && _msg->parent_name() != this->GetName())
    {
      iter = this->visuals.find(_msg->name());
      if (iter != this->visuals.end())
        gzerr << "Visual already exists. This shouldn't happen.\n";

      // Make sure the parent visual exists before trying to add a child
      // visual
      iter = this->visuals.find(_msg->parent_name());
      if (iter != this->visuals.end())
        visual.reset(new Visual(_msg->name(), iter->second));
    }
    else
    {
      // Add a visual that is attached to the scene root
      visual.reset(new Visual(_msg->name(), this->worldVisual));
    }

    if (visual)
    {
      result = true;
      visual->LoadFromMsg(_msg);
      this->visuals[_msg->name()] = visual;
      if (visual->GetName().find("__COLLISION_VISUAL__") != std::string::npos ||
          visual->GetName().find("__SKELETON_VISUAL__") != std::string::npos)
      {
        visual->SetVisible(false);
      }
    }
  }

  return result;
}

/////////////////////////////////////////////////
void Scene::OnPoseMsg(ConstPosePtr &_msg)
{
  boost::mutex::scoped_lock lock(*this->receiveMutex);
  PoseMsgs_L::iterator iter;

  // Find an old model message, and remove them
  for (iter = this->poseMsgs.begin(); iter != this->poseMsgs.end(); ++iter)
  {
    if ((*iter)->name() == _msg->name())
    {
      this->poseMsgs.erase(iter);
      break;
    }
  }

  this->poseMsgs.push_back(_msg);
}

/////////////////////////////////////////////////
void Scene::OnSkeletonPoseMsg(ConstPoseAnimationPtr &_msg)
{
  boost::mutex::scoped_lock lock(*this->receiveMutex);
  SkeletonPoseMsgs_L::iterator iter;

  // Find an old model message, and remove them
  for (iter = this->skeletonPoseMsgs.begin();
        iter != this->skeletonPoseMsgs.end(); ++iter)
  {
    if ((*iter)->model_name() == _msg->model_name())
    {
      this->skeletonPoseMsgs.erase(iter);
      break;
    }
  }

  this->skeletonPoseMsgs.push_back(_msg);
}


/////////////////////////////////////////////////
void Scene::OnLightMsg(ConstLightPtr &_msg)
{
  boost::mutex::scoped_lock lock(*this->receiveMutex);
  this->lightMsgs.push_back(_msg);
}

/////////////////////////////////////////////////
void Scene::ProcessLightMsg(ConstLightPtr &_msg)
{
  Light_M::iterator iter;
  iter = this->lights.find(_msg->name());

  if (iter == this->lights.end())
  {
    LightPtr light(new Light(this));
    light->LoadFromMsg(_msg);
    this->lights[_msg->name()] = light;
    RTShaderSystem::Instance()->UpdateShaders();
  }
}

/////////////////////////////////////////////////
void Scene::OnSelectionMsg(ConstSelectionPtr &_msg)
{
  this->selectionMsg = _msg;
}

/////////////////////////////////////////////////
void Scene::SetSky(const std::string &_material)
{
  this->sdf->GetOrCreateElement("background")->GetOrCreateElement(
      "sky")->GetAttribute("material")->Set(_material);

  try
  {
    Ogre::Quaternion orientation;
    orientation.FromAngleAxis(Ogre::Degree(90), Ogre::Vector3(1, 0, 0));
    double curvature = 10;  // ogre recommended default
    double tiling = 8;  // ogre recommended default
    double distance = 100;  // ogre recommended default
    this->manager->setSkyDome(true, _material, curvature,
        tiling, distance, true, orientation);
  }
  catch(int)
  {
    gzwarn << "Unable to set sky dome to material[" << _material << "]\n";
  }
}

/////////////////////////////////////////////////
void Scene::SetShadowsEnabled(bool _value)
{
  sdf::ElementPtr shadowElem = this->sdf->GetOrCreateElement("shadows");
  if (_value != shadowElem->GetValueBool("enabled"))
  {
    shadowElem->GetAttribute("enabled")->Set(_value);
    if (_value)
      RTShaderSystem::Instance()->ApplyShadows(this);
    else
      RTShaderSystem::Instance()->RemoveShadows(this);
  }
}

/////////////////////////////////////////////////
bool Scene::GetShadowsEnabled() const
{
  sdf::ElementPtr shadowElem = this->sdf->GetOrCreateElement("shadows");
  return shadowElem->GetValueBool("enabled");
}

/////////////////////////////////////////////////
void Scene::AddVisual(VisualPtr &_vis)
{
  this->visuals[_vis->GetName()] = _vis;
}

/////////////////////////////////////////////////
void Scene::RemoveVisual(VisualPtr _vis)
{
  if (_vis)
  {
    Visual_M::iterator iter = this->visuals.find(_vis->GetName());
    if (iter != this->visuals.end())
    {
      iter->second->Fini();
      this->visuals.erase(iter);
    }

    if (this->selectionObj->GetVisualName() == _vis->GetName())
      this->selectionObj->Clear();
  }
}

/////////////////////////////////////////////////
std::string Scene::GetUniqueName(const std::string &_prefix)
{
  std::ostringstream test;
  bool exists = true;
  int index = 0;
  while (exists)
  {
    test.str("");
    test << _prefix << "_" << index++;
    exists = this->manager->hasSceneNode(test.str());
  }

  return test.str();
}

/////////////////////////////////////////////////
SelectionObj *Scene::GetSelectionObj() const
{
  return this->selectionObj;
}

/////////////////////////////////////////////////
void Scene::SetGrid(bool _enabled)
{
  if (_enabled)
  {
    Grid *grid = new Grid(this, 1, 1, 10, common::Color(1, 1, 0, 1));
    grid->Init();
    this->grids.push_back(grid);

    grid = new Grid(this, 20, 1, 10, common::Color(1, 1, 1, 1));
    grid->Init();
    this->grids.push_back(grid);
  }
}

/////////////////////////////////////////////////
VisualPtr Scene::CloneVisual(const std::string &_visualName,
                             const std::string &_newName)
{
  VisualPtr result;
  VisualPtr vis = this->GetVisual(_visualName);
  if (vis)
  {
    result = vis->Clone(_newName, this->worldVisual);
    this->visuals[_newName] = result;
  }
  return result;
}

//////////////////////////////////////////////////
std::string Scene::StripSceneName(const std::string &_name) const
{
  if (_name.find(this->GetName() + "::") == 0)
    return _name.substr(this->GetName().size() + 2);
  else
    return _name;
}

//////////////////////////////////////////////////
Heightmap *Scene::GetHeightmap() const
{
  return this->heightmap;
}
