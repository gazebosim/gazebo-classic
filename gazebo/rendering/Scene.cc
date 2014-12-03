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
#include <boost/lexical_cast.hpp>

#include "gazebo/rendering/skyx/include/SkyX.h"
#include "gazebo/rendering/ogre_gazebo.h"

#include "gazebo/msgs/msgs.hh"

#include "gazebo/common/Exception.hh"
#include "gazebo/common/Assert.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/gazebo_config.h"
#include "gazebo/rendering/Road2d.hh"
#include "gazebo/rendering/Projector.hh"
#include "gazebo/rendering/Heightmap.hh"
#include "gazebo/rendering/RenderEvents.hh"
#include "gazebo/rendering/LaserVisual.hh"
#include "gazebo/rendering/SonarVisual.hh"
#include "gazebo/rendering/WrenchVisual.hh"
#include "gazebo/rendering/CameraVisual.hh"
#include "gazebo/rendering/JointVisual.hh"
#include "gazebo/rendering/COMVisual.hh"
#include "gazebo/rendering/ContactVisual.hh"
#include "gazebo/rendering/Conversions.hh"
#include "gazebo/rendering/Light.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/rendering/RenderEngine.hh"
#include "gazebo/rendering/UserCamera.hh"
#include "gazebo/rendering/Camera.hh"
#include "gazebo/rendering/DepthCamera.hh"
#include "gazebo/rendering/GpuLaser.hh"
#include "gazebo/rendering/Grid.hh"
#include "gazebo/rendering/DynamicLines.hh"
#include "gazebo/rendering/RFIDVisual.hh"
#include "gazebo/rendering/RFIDTagVisual.hh"
#include "gazebo/rendering/VideoVisual.hh"
#include "gazebo/rendering/TransmitterVisual.hh"

#if OGRE_VERSION_MAJOR >= 1 && OGRE_VERSION_MINOR >= 8
#include "gazebo/rendering/deferred_shading/SSAOLogic.hh"
#include "gazebo/rendering/deferred_shading/GBufferSchemeHandler.hh"
#include "gazebo/rendering/deferred_shading/NullSchemeHandler.hh"
#include "gazebo/rendering/deferred_shading/MergeSchemeHandler.hh"
#include "gazebo/rendering/deferred_shading/DeferredLightCP.hh"
#endif

#include "gazebo/rendering/RTShaderSystem.hh"
#include "gazebo/transport/TransportIface.hh"
#include "gazebo/transport/Node.hh"

#include "gazebo/rendering/Scene.hh"

#ifdef HAVE_OCULUS
#include "gazebo/rendering/OculusCamera.hh"
#endif

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
Scene::Scene(const std::string &_name, bool _enableVisualizations,
    bool _isServer)
{
  // \todo: This is a hack. There is no guarantee (other than the
  // improbability of creating an extreme number of visuals), that
  // this contactVisId is unique.
  this->contactVisId = GZ_UINT32_MAX;

  this->initialized = false;
  this->showCOMs = false;
  this->showCollisions = false;
  this->showJoints = false;
  this->transparent = false;
  this->wireframe = false;

  this->requestMsg = NULL;
  this->enableVisualizations = _enableVisualizations;
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(_name);
  this->id = idCounter++;
  this->idString = boost::lexical_cast<std::string>(this->id);

  this->name = _name;
  this->manager = NULL;
  this->raySceneQuery = NULL;
  this->skyx = NULL;

  this->receiveMutex = new boost::mutex();

  this->connections.push_back(
      event::Events::ConnectPreRender(boost::bind(&Scene::PreRender, this)));

  this->sensorSub = this->node->Subscribe("~/sensor",
                                          &Scene::OnSensorMsg, this, true);
  this->visSub = this->node->Subscribe("~/visual", &Scene::OnVisualMsg, this);

  this->lightPub = this->node->Advertise<msgs::Light>("~/light");

  this->lightSub = this->node->Subscribe("~/light", &Scene::OnLightMsg, this);

  if (_isServer)
  {
    this->poseSub = this->node->Subscribe("~/pose/local/info",
        &Scene::OnPoseMsg, this);
  }
  else
  {
    this->poseSub = this->node->Subscribe("~/pose/info",
        &Scene::OnPoseMsg, this);
  }

  this->jointSub = this->node->Subscribe("~/joint", &Scene::OnJointMsg, this);
  this->skeletonPoseSub = this->node->Subscribe("~/skeleton_pose/info",
          &Scene::OnSkeletonPoseMsg, this);
  this->selectionSub = this->node->Subscribe("~/selection",
      &Scene::OnSelectionMsg, this);
  this->skySub = this->node->Subscribe("~/sky", &Scene::OnSkyMsg, this);
  this->modelInfoSub = this->node->Subscribe("~/model/info",
                                             &Scene::OnModelMsg, this);

  this->requestPub = this->node->Advertise<msgs::Request>("~/request");

  this->requestSub = this->node->Subscribe("~/request",
      &Scene::OnRequest, this);

  // \TODO: This causes the Scene to occasionally miss the response to
  // scene_info
  // this->responsePub = this->node->Advertise<msgs::Response>("~/response");
  this->responseSub = this->node->Subscribe("~/response",
      &Scene::OnResponse, this, true);
  this->sceneSub = this->node->Subscribe("~/scene", &Scene::OnScene, this);

  this->sdf.reset(new sdf::Element);
  sdf::initFile("scene.sdf", this->sdf);

  this->terrain = NULL;
  this->selectedVis.reset();

  this->sceneSimTimePosesApplied = common::Time();
  this->sceneSimTimePosesReceived = common::Time();
}

//////////////////////////////////////////////////
void Scene::Clear()
{
  this->node->Fini();
  this->modelMsgs.clear();
  this->visualMsgs.clear();
  this->lightMsgs.clear();
  this->poseMsgs.clear();
  this->sceneMsgs.clear();
  this->jointMsgs.clear();
  this->joints.clear();
  this->linkMsgs.clear();
  this->cameras.clear();
  this->userCameras.clear();
  this->lights.clear();

  delete this->terrain;
  this->terrain = NULL;

  // Erase the world visual
  this->visuals.erase(this->visuals.begin());

  while (!this->visuals.empty())
    if (this->visuals.begin()->second)
      this->RemoveVisual(this->visuals.begin()->second);
  this->visuals.clear();

  for (uint32_t i = 0; i < this->grids.size(); i++)
    delete this->grids[i];
  this->grids.clear();

  this->sensorMsgs.clear();
  RTShaderSystem::Instance()->Clear();

  this->initialized = false;
}

//////////////////////////////////////////////////
Scene::~Scene()
{
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

  Visual_M::iterator iter;
  this->visuals.clear();
  this->jointMsgs.clear();
  this->joints.clear();
  this->linkMsgs.clear();
  this->sceneMsgs.clear();
  this->poseMsgs.clear();
  this->lightMsgs.clear();
  this->visualMsgs.clear();

  this->worldVisual.reset();
  this->selectionMsg.reset();
  this->lights.clear();

  // Remove a scene
  RTShaderSystem::Instance()->RemoveScene(shared_from_this());

  for (uint32_t i = 0; i < this->grids.size(); i++)
    delete this->grids[i];
  this->grids.clear();

  for (unsigned int i = 0; i < this->cameras.size(); ++i)
    this->cameras[i].reset();
  this->cameras.clear();

  for (unsigned int i = 0; i < this->userCameras.size(); ++i)
    this->userCameras[i].reset();
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
  this->initialized = false;
  Ogre::Root *root = RenderEngine::Instance()->root;

  if (this->manager)
    root->destroySceneManager(this->manager);

  this->manager = root->createSceneManager(Ogre::ST_GENERIC);
  this->manager->setAmbientLight(Ogre::ColourValue(0.1, 0.1, 0.1, 0.1));

#if OGRE_VERSION_MAJOR > 1 || OGRE_VERSION_MINOR >= 9
  this->manager->addRenderQueueListener(
      RenderEngine::Instance()->GetOverlaySystem());
#endif
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
  this->worldVisual->SetId(0);
  this->visuals[0] = this->worldVisual;

  // RTShader system self-enables if the render path type is FORWARD,
  RTShaderSystem::Instance()->AddScene(shared_from_this());
  RTShaderSystem::Instance()->ApplyShadows(shared_from_this());

  if (RenderEngine::Instance()->GetRenderPathType() == RenderEngine::DEFERRED)
    this->InitDeferredShading();

  for (uint32_t i = 0; i < this->grids.size(); i++)
    this->grids[i]->Init();

  // Create Sky. This initializes SkyX, and makes it invisible. A Sky
  // message must be received (via a scene message or on the ~/sky topic).
  try
  {
    this->SetSky();
  }
  catch(...)
  {
    gzerr << "Failed to create the sky\n";
  }

  // Create Fog
  if (this->sdf->HasElement("fog"))
  {
    boost::shared_ptr<sdf::Element> fogElem = this->sdf->GetElement("fog");
    this->SetFog(fogElem->Get<std::string>("type"),
                 fogElem->Get<common::Color>("color"),
                 fogElem->Get<double>("density"),
                 fogElem->Get<double>("start"),
                 fogElem->Get<double>("end"));
  }

  // Create ray scene query
  this->raySceneQuery = this->manager->createRayQuery(Ogre::Ray());
  this->raySceneQuery->setSortByDistance(true);
  this->raySceneQuery->setQueryMask(Ogre::SceneManager::ENTITY_TYPE_MASK);

  // Force shadows on.
  this->SetShadowsEnabled(true);

  this->requestPub->WaitForConnection();
  this->requestMsg = msgs::CreateRequest("scene_info");
  this->requestPub->Publish(*this->requestMsg);

  Road2d *road = new Road2d();
  road->Load(this->worldVisual);
}

//////////////////////////////////////////////////
bool Scene::GetInitialized() const
{
  return this->initialized;
}

//////////////////////////////////////////////////
void Scene::InitDeferredShading()
{
#if OGRE_VERSION_MAJOR > 1 || OGRE_VERSION_MINOR >= 8
  Ogre::CompositorManager &compMgr = Ogre::CompositorManager::getSingleton();

  // Deferred Shading scheme handler
  Ogre::MaterialManager::getSingleton().addListener(
      new GBufferSchemeHandler(GBufferMaterialGenerator::GBT_FAT),
      "DSGBuffer");

  // Deferred Lighting scheme handlers
  Ogre::MaterialManager::getSingleton().addListener(
      new GBufferSchemeHandler(GBufferMaterialGenerator::GBT_NORMAL_AND_DEPTH),
      "DLGBuffer");
  Ogre::MaterialManager::getSingleton().addListener(
      new MergeSchemeHandler(false), "DLMerge");

  Ogre::MaterialManager::getSingleton().addListener(
      new NullSchemeHandler, "NoGBuffer");

  compMgr.registerCustomCompositionPass("DeferredShadingLight",
      new DeferredLightCompositionPass<DeferredShading>);
  compMgr.registerCustomCompositionPass("DeferredLightingLight",
      new DeferredLightCompositionPass<DeferredLighting>);

  compMgr.registerCompositorLogic("SSAOLogic", new SSAOLogic);

  // Create and instance geometry for VPL
  Ogre::MeshPtr VPLMesh =
    Ogre::MeshManager::getSingleton().createManual("VPLMesh",
        Ogre::ResourceGroupManager::AUTODETECT_RESOURCE_GROUP_NAME);

  Ogre::SubMesh *submeshMesh = VPLMesh->createSubMesh();
  submeshMesh->operationType = Ogre::RenderOperation::OT_TRIANGLE_LIST;
  submeshMesh->indexData = new Ogre::IndexData();
  submeshMesh->vertexData = new Ogre::VertexData();
  submeshMesh->useSharedVertices = false;
  VPLMesh->_setBoundingSphereRadius(10.8f);
  VPLMesh->_setBounds(Ogre::AxisAlignedBox(
        Ogre::Vector3(-10.8, -10.8, -10.8), Ogre::Vector3(10.8, 10.8, 10.8)));

  GeomUtils::CreateSphere(submeshMesh->vertexData, submeshMesh->indexData,
      1.0, 6, 6, false, false);

  int numVPLs = 400;
  Ogre::InstanceManager *im =
    this->manager->createInstanceManager("VPL_InstanceMgr",
      "VPLMesh", Ogre::ResourceGroupManager::AUTODETECT_RESOURCE_GROUP_NAME,
          Ogre::InstanceManager::HWInstancingBasic, numVPLs, Ogre::IM_USEALL);

  for (int i = 0; i < numVPLs; ++i)
  {
    // Ogre::InstancedEntity *new_entity =
    im->createInstancedEntity("DeferredLighting/VPL");
  }

  im->setBatchesAsStaticAndUpdate(true);
#endif
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
void Scene::SetAmbientColor(const common::Color &_color)
{
  this->sdf->GetElement("ambient")->Set(_color);

  // Ambient lighting
  if (this->manager &&
      Conversions::Convert(this->manager->getAmbientLight()) != _color)
  {
    this->manager->setAmbientLight(Conversions::Convert(_color));
  }
}

//////////////////////////////////////////////////
common::Color Scene::GetAmbientColor() const
{
  return this->sdf->Get<common::Color>("ambient");
}

//////////////////////////////////////////////////
void Scene::SetBackgroundColor(const common::Color &_color)
{
  this->sdf->GetElement("background")->Set(_color);
  Ogre::ColourValue clr = Conversions::Convert(_color);

  std::vector<CameraPtr>::iterator iter;
  for (iter = this->cameras.begin(); iter != this->cameras.end(); ++iter)
  {
    if ((*iter)->GetViewport() &&
        (*iter)->GetViewport()->getBackgroundColour() != clr)
      (*iter)->GetViewport()->setBackgroundColour(clr);
  }

  std::vector<UserCameraPtr>::iterator iter2;
  for (iter2 = this->userCameras.begin();
       iter2 != this->userCameras.end(); ++iter2)
  {
    if ((*iter2)->GetViewport() &&
        (*iter2)->GetViewport()->getBackgroundColour() != clr)
    {
      (*iter2)->GetViewport()->setBackgroundColour(clr);
    }
  }
}

//////////////////////////////////////////////////
common::Color Scene::GetBackgroundColor() const
{
  return this->sdf->Get<common::Color>("background");
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
uint32_t Scene::GetGridCount() const
{
  return this->grids.size();
}

//////////////////////////////////////////////////
CameraPtr Scene::CreateCamera(const std::string &_name, bool _autoRender)
{
  CameraPtr camera(new Camera(_name, shared_from_this(), _autoRender));
  this->cameras.push_back(camera);

  return camera;
}

//////////////////////////////////////////////////
DepthCameraPtr Scene::CreateDepthCamera(const std::string &_name,
                                        bool _autoRender)
{
  DepthCameraPtr camera(new DepthCamera(this->name + "::" + _name,
        shared_from_this(), _autoRender));
  this->cameras.push_back(camera);

  return camera;
}

//////////////////////////////////////////////////
GpuLaserPtr Scene::CreateGpuLaser(const std::string &_name,
                                        bool _autoRender)
{
  GpuLaserPtr camera(new GpuLaser(this->name + "::" + _name,
        shared_from_this(), _autoRender));
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
#ifdef HAVE_OCULUS
OculusCameraPtr Scene::CreateOculusCamera(const std::string &_name)
{
  OculusCameraPtr camera(new OculusCamera(_name, shared_from_this()));

  if (camera->Ready())
  {
    camera->Load();
    camera->Init();
    this->oculusCameras.push_back(camera);
  }

  return camera;
}

//////////////////////////////////////////////////
uint32_t Scene::GetOculusCameraCount() const
{
  return this->oculusCameras.size();
}
#endif

//////////////////////////////////////////////////
UserCameraPtr Scene::CreateUserCamera(const std::string &_name)
{
  UserCameraPtr camera(new UserCamera(_name, shared_from_this()));
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
void Scene::RemoveCamera(const std::string &_name)
{
  std::vector<CameraPtr>::iterator iter;
  for (iter = this->cameras.begin(); iter != this->cameras.end(); ++iter)
  {
    if ((*iter)->GetName() == _name)
    {
      (*iter)->Fini();
      (*iter).reset();
      this->cameras.erase(iter);
      break;
    }
  }
}

//////////////////////////////////////////////////
LightPtr Scene::GetLight(const std::string &_name) const
{
  LightPtr result;
  std::string n = this->StripSceneName(_name);
  Light_M::const_iterator iter = this->lights.find(n);
  if (iter != this->lights.end())
    result = iter->second;
  return result;
}

//////////////////////////////////////////////////
uint32_t Scene::GetLightCount() const
{
  return this->lights.size();
}

//////////////////////////////////////////////////
LightPtr Scene::GetLight(uint32_t _index) const
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
VisualPtr Scene::GetVisual(uint32_t _id) const
{
  Visual_M::const_iterator iter = this->visuals.find(_id);
  if (iter != this->visuals.end())
    return iter->second;
  return VisualPtr();
}

//////////////////////////////////////////////////
VisualPtr Scene::GetVisual(const std::string &_name) const
{
  VisualPtr result;

  Visual_M::const_iterator iter;
  for (iter = this->visuals.begin(); iter != this->visuals.end(); ++iter)
    if (iter->second->GetName() == _name)
      break;

  if (iter != this->visuals.end())
    result = iter->second;
  else
  {
    std::string otherName = this->GetName() + "::" + _name;
    for (iter = this->visuals.begin(); iter != this->visuals.end(); ++iter)
      if (iter->second->GetName() == otherName)
        break;

    if (iter != this->visuals.end())
      result = iter->second;
  }

  return result;
}

//////////////////////////////////////////////////
uint32_t Scene::GetVisualCount() const
{
  return this->visuals.size();
}

//////////////////////////////////////////////////
void Scene::SelectVisual(const std::string &_name, const std::string &_mode)
{
  this->selectedVis = this->GetVisual(_name);
  this->selectionMode = _mode;
}

//////////////////////////////////////////////////
VisualPtr Scene::GetSelectedVisual() const
{
  return this->selectedVis;
}

//////////////////////////////////////////////////
VisualPtr Scene::GetVisualAt(CameraPtr _camera,
                             const math::Vector2i &_mousePos,
                             std::string &_mod)
{
  VisualPtr visual;
  Ogre::Entity *closestEntity = this->GetOgreEntityAt(_camera, _mousePos,
                                                       false);

  _mod = "";
  if (closestEntity)
  {
    // Make sure we set the _mod only if we have found a selection object
    if (closestEntity->getName().substr(0, 15) == "__SELECTION_OBJ" &&
        closestEntity->getUserAny().getType() == typeid(std::string))
    {
      try
      {
        _mod = Ogre::any_cast<std::string>(closestEntity->getUserAny());
      }
      catch(boost::bad_any_cast &e)
      {
        gzerr << "boost any_cast error:" << e.what() << "\n";
      }
    }

    try
    {
      visual = this->GetVisual(Ogre::any_cast<std::string>(
            closestEntity->getUserAny()));
    }
    catch(boost::bad_any_cast &e)
    {
      gzerr << "boost any_cast error:" << e.what() << "\n";
    }
  }

  return visual;
}

//////////////////////////////////////////////////
VisualPtr Scene::GetModelVisualAt(CameraPtr _camera,
                                  const math::Vector2i &_mousePos)
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

    for (uint32_t i = 0; i < below.size(); ++i)
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
double Scene::GetHeightBelowPoint(const math::Vector3 &_pt)
{
  double height = 0;
  Ogre::Ray ray(Conversions::Convert(_pt), Ogre::Vector3(0, 0, -1));

  if (!this->raySceneQuery)
    this->raySceneQuery = this->manager->createRayQuery(Ogre::Ray());
  this->raySceneQuery->setRay(ray);
  this->raySceneQuery->setSortByDistance(true, 0);

  // Perform the scene query
  Ogre::RaySceneQueryResult &result = this->raySceneQuery->execute();
  Ogre::RaySceneQueryResult::iterator iter;

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

      height = _pt.z - iter->distance;
      break;
    }
  }

  // The default ray scene query does not work with terrain, so we have to
  // check ourselves.
  if (this->terrain)
  {
    double terrainHeight = this->terrain->GetHeight(_pt.x, _pt.y, _pt.z);
    if (terrainHeight <= _pt.z)
      height = std::max(height, terrainHeight);
  }

  return height;
}

//////////////////////////////////////////////////
void Scene::GetVisualsBelowPoint(const math::Vector3 &_pt,
                                 std::vector<VisualPtr> &_visuals)
{
  Ogre::Ray ray(Conversions::Convert(_pt), Ogre::Vector3(0, 0, -1));

  if (!this->raySceneQuery)
    this->raySceneQuery = this->manager->createRayQuery(Ogre::Ray());

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

      Ogre::Entity *ogreEntity = static_cast<Ogre::Entity*>(iter->movable);
      if (ogreEntity)
      {
        try
        {
          VisualPtr v = this->GetVisual(Ogre::any_cast<std::string>(
                                        ogreEntity->getUserAny()));
          if (v)
            _visuals.push_back(v);
        }
        catch(boost::bad_any_cast &e)
        {
          gzerr << "boost any_cast error:" << e.what() << "\n";
        }
      }
    }
  }
}

//////////////////////////////////////////////////
VisualPtr Scene::GetVisualAt(CameraPtr _camera,
                             const math::Vector2i &_mousePos)
{
  VisualPtr visual;

  Ogre::Entity *closestEntity = this->GetOgreEntityAt(_camera,
                                                      _mousePos, true);
  if (closestEntity)
  {
    try
    {
      visual = this->GetVisual(Ogre::any_cast<std::string>(
            closestEntity->getUserAny()));
    }
    catch(boost::bad_any_cast &e)
    {
      gzerr << "boost any_cast error:" << e.what() << "\n";
    }
  }

  return visual;
}

/////////////////////////////////////////////////
Ogre::Entity *Scene::GetOgreEntityAt(CameraPtr _camera,
                                     const math::Vector2i &_mousePos,
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

      Ogre::Entity *ogreEntity = static_cast<Ogre::Entity*>(iter->movable);

      // mesh data to retrieve
      size_t vertex_count;
      size_t index_count;
      Ogre::Vector3 *vertices;
      uint64_t *indices;

      // Get the mesh information
      this->GetMeshInformation(ogreEntity->getMesh().get(), vertex_count,
          vertices, index_count, indices,
          ogreEntity->getParentNode()->_getDerivedPosition(),
          ogreEntity->getParentNode()->_getDerivedOrientation(),
          ogreEntity->getParentNode()->_getDerivedScale());

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
        closestEntity = ogreEntity;
        // break;
      }
    }
  }

  return closestEntity;
}

//////////////////////////////////////////////////
bool Scene::GetFirstContact(CameraPtr _camera,
                            const math::Vector2i &_mousePos,
                            math::Vector3 &_position)
{
  bool valid = false;
  Ogre::Camera *ogreCam = _camera->GetOgreCamera();

  _position = math::Vector3::Zero;

  // Ogre::Real closest_distance = -1.0f;
  Ogre::Ray mouseRay = ogreCam->getCameraToViewportRay(
      static_cast<float>(_mousePos.x) /
      ogreCam->getViewport()->getActualWidth(),
      static_cast<float>(_mousePos.y) /
      ogreCam->getViewport()->getActualHeight());

  this->raySceneQuery->setSortByDistance(true);
  this->raySceneQuery->setRay(mouseRay);

  // Perform the scene query
  Ogre::RaySceneQueryResult &result = this->raySceneQuery->execute();
  Ogre::RaySceneQueryResult::iterator iter = result.begin();

  double distance = -1.0;

  // Iterate over all the results.
  for (; iter != result.end() && distance <= 0.0; ++iter)
  {
    // Skip results where the distance is zero or less
    if (iter->distance <= 0.0)
      continue;

    unsigned int flags = iter->movable->getVisibilityFlags();

    // Only accept a hit if there is an entity and not a gui visual
    if (iter->movable &&
        iter->movable->getMovableType().compare("Entity") == 0 &&
        !(flags != GZ_VISIBILITY_ALL && flags & GZ_VISIBILITY_GUI))
    {
      Ogre::Entity *ogreEntity = static_cast<Ogre::Entity*>(iter->movable);

      // mesh data to retrieve
      size_t vertexCount;
      size_t indexCount;
      Ogre::Vector3 *vertices;
      uint64_t *indices;

      // Get the mesh information
      this->GetMeshInformation(ogreEntity->getMesh().get(), vertexCount,
          vertices, indexCount, indices,
          ogreEntity->getParentNode()->_getDerivedPosition(),
          ogreEntity->getParentNode()->_getDerivedOrientation(),
          ogreEntity->getParentNode()->_getDerivedScale());

      for (int i = 0; i < static_cast<int>(indexCount); i += 3)
      {
        // when indices size is not divisible by 3
        if (i+2 >= static_cast<int>(indexCount))
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
          if ((distance < 0.0f) || (hit.second < distance))
          {
            // this is the closest so far, save it off
            distance = hit.second;
          }
        }
      }
    }
  }

  // If nothing was hit, then check the terrain.
  if (distance <= 0.0 && this->terrain)
  {
    // The terrain uses a special ray intersection test.
    Ogre::TerrainGroup::RayResult terrainResult =
      this->terrain->GetOgreTerrain()->rayIntersects(mouseRay);

    if (terrainResult.hit)
    {
      _position = Conversions::Convert(terrainResult.position);
      valid = true;
    }
  }

  // Compute the interesection point using the mouse ray and a distance
  // value.
  if (_position == math::Vector3::Zero && distance > 0.0)
  {
    _position = Conversions::Convert(mouseRay.getPoint(distance));
    valid = true;
  }

  return valid;
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

  sdf::ElementPtr elem = this->sdf->GetElement("fog");

  elem->GetElement("type")->Set(_type);
  elem->GetElement("color")->Set(_color);
  elem->GetElement("density")->Set(_density);
  elem->GetElement("start")->Set(_start);
  elem->GetElement("end")->Set(_end);

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
bool Scene::ProcessSceneMsg(ConstScenePtr &_msg)
{
  {
    boost::recursive_mutex::scoped_lock lock(this->poseMsgMutex);
    for (int i = 0; i < _msg->model_size(); i++)
    {
      PoseMsgs_M::iterator iter = this->poseMsgs.find(_msg->model(i).id());
      if (iter != this->poseMsgs.end())
        iter->second.CopyFrom(_msg->model(i).pose());
      else
        this->poseMsgs.insert(
            std::make_pair(_msg->model(i).id(), _msg->model(i).pose()));

      this->poseMsgs[_msg->model(i).id()].set_name(_msg->model(i).name());
      this->poseMsgs[_msg->model(i).id()].set_id(_msg->model(i).id());

      this->ProcessModelMsg(_msg->model(i));
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

  if (_msg->has_shadows())
    this->SetShadowsEnabled(_msg->shadows());

  if (_msg->has_grid())
    this->SetGrid(_msg->grid());

  // Process the sky message.
  if (_msg->has_sky())
  {
    boost::shared_ptr<msgs::Sky> sm(new msgs::Sky(_msg->sky()));
    this->OnSkyMsg(sm);
  }

  if (_msg->has_fog())
  {
    sdf::ElementPtr elem = this->sdf->GetElement("fog");

    if (_msg->fog().has_color())
      elem->GetElement("color")->Set(
          msgs::Convert(_msg->fog().color()));

    if (_msg->fog().has_density())
      elem->GetElement("density")->Set(_msg->fog().density());

    if (_msg->fog().has_start())
      elem->GetElement("start")->Set(_msg->fog().start());

    if (_msg->fog().has_end())
      elem->GetElement("end")->Set(_msg->fog().end());

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

      elem->GetElement("type")->Set(type);
    }

    this->SetFog(elem->Get<std::string>("type"),
                 elem->Get<common::Color>("color"),
                 elem->Get<double>("density"),
                 elem->Get<double>("start"),
                 elem->Get<double>("end"));
  }

  return true;
}

//////////////////////////////////////////////////
bool Scene::ProcessModelMsg(const msgs::Model &_msg)
{
  std::string modelName, linkName;

  modelName = _msg.name() + "::";
  for (int j = 0; j < _msg.visual_size(); j++)
  {
    boost::shared_ptr<msgs::Visual> vm(new msgs::Visual(
          _msg.visual(j)));
    this->visualMsgs.push_back(vm);
  }

  // Set the scale of the model visual
  if (_msg.has_scale())
  {
    // update scale using a visual msg
    boost::shared_ptr<msgs::Visual> vm(new msgs::Visual);
    if (_msg.has_id())
      vm->set_id(_msg.id());
    if (_msg.has_name())
      vm->set_name(_msg.name());
    vm->mutable_scale()->set_x(_msg.scale().x());
    vm->mutable_scale()->set_y(_msg.scale().y());
    vm->mutable_scale()->set_z(_msg.scale().z());
    this->visualMsgs.push_back(vm);
  }

  for (int j = 0; j < _msg.joint_size(); j++)
  {
    boost::shared_ptr<msgs::Joint> jm(new msgs::Joint(
          _msg.joint(j)));
    this->jointMsgs.push_back(jm);

    for (int k = 0; k < _msg.joint(j).sensor_size(); k++)
    {
      boost::shared_ptr<msgs::Sensor> sm(new msgs::Sensor(
            _msg.joint(j).sensor(k)));
      this->sensorMsgs.push_back(sm);
    }
  }

  for (int j = 0; j < _msg.link_size(); j++)
  {
    linkName = modelName + _msg.link(j).name();

    {
      boost::recursive_mutex::scoped_lock lock(this->poseMsgMutex);
      if (_msg.link(j).has_pose())
      {
        PoseMsgs_M::iterator iter = this->poseMsgs.find(_msg.link(j).id());
        if (iter != this->poseMsgs.end())
          iter->second.CopyFrom(_msg.link(j).pose());
        else
          this->poseMsgs.insert(
              std::make_pair(_msg.link(j).id(), _msg.link(j).pose()));

        this->poseMsgs[_msg.link(j).id()].set_name(linkName);
        this->poseMsgs[_msg.link(j).id()].set_id(_msg.link(j).id());
      }
    }

    if (_msg.link(j).has_inertial())
    {
      boost::shared_ptr<msgs::Link> lm(new msgs::Link(_msg.link(j)));
      this->linkMsgs.push_back(lm);
    }

    for (int k = 0; k < _msg.link(j).visual_size(); k++)
    {
      boost::shared_ptr<msgs::Visual> vm(new msgs::Visual(
            _msg.link(j).visual(k)));
      this->visualMsgs.push_back(vm);
    }

    for (int k = 0; k < _msg.link(j).collision_size(); k++)
    {
      for (int l = 0;
          l < _msg.link(j).collision(k).visual_size(); l++)
      {
        boost::shared_ptr<msgs::Visual> vm(new msgs::Visual(
              _msg.link(j).collision(k).visual(l)));
        this->visualMsgs.push_back(vm);
      }
    }

    for (int k = 0; k < _msg.link(j).sensor_size(); k++)
    {
      boost::shared_ptr<msgs::Sensor> sm(new msgs::Sensor(
            _msg.link(j).sensor(k)));
      this->sensorMsgs.push_back(sm);
    }
  }

  return true;
}

//////////////////////////////////////////////////
void Scene::OnSensorMsg(ConstSensorPtr &_msg)
{
  boost::mutex::scoped_lock lock(*this->receiveMutex);
  this->sensorMsgs.push_back(_msg);
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
  /* Deferred shading debug code. Delete me soon (July 17, 2012)
  static bool first = true;

  if (!first)
  {
    Ogre::RenderSystem *renderSys = this->manager->getDestinationRenderSystem();
    Ogre::RenderSystem::RenderTargetIterator renderIter =
      renderSys->getRenderTargetIterator();

    int i = 0;
    for (; renderIter.current() != renderIter.end(); renderIter.moveNext())
    {
      if (renderIter.current()->second->getNumViewports() > 0)
      {
        std::ostringstream filename, filename2;
        filename << "/tmp/render_targets/iter_" << this->iterations
                 << "_" << i << ".png";
        filename2 << "/tmp/render_targets/iter_"
                  << this->iterations << "_" << i << "_b.png";

        Ogre::MultiRenderTarget *mtarget = dynamic_cast<Ogre::MultiRenderTarget*>(renderIter.current()->second);
        if (mtarget)
        {
          // std::cout << renderIter.current()->first << "\n";
          mtarget->getBoundSurface(0)->writeContentsToFile(filename.str());

          mtarget->getBoundSurface(1)->writeContentsToFile(filename2.str());
          i++;
        }
        else
        {
          renderIter.current()->second->writeContentsToFile(filename.str());
          i++;
        }
      }
    }
    this->iterations++;
  }
  else
    first = false;
  */

  static RequestMsgs_L::iterator rIter;
  static SceneMsgs_L::iterator sIter;
  static ModelMsgs_L::iterator modelIter;
  static VisualMsgs_L::iterator visualIter;
  static LightMsgs_L::iterator lightIter;
  static PoseMsgs_M::iterator pIter;
  static SkeletonPoseMsgs_L::iterator spIter;
  static JointMsgs_L::iterator jointIter;
  static SensorMsgs_L::iterator sensorIter;
  static LinkMsgs_L::iterator linkIter;

  SceneMsgs_L sceneMsgsCopy;
  ModelMsgs_L modelMsgsCopy;
  SensorMsgs_L sensorMsgsCopy;
  LightMsgs_L lightMsgsCopy;
  VisualMsgs_L visualMsgsCopy;
  JointMsgs_L jointMsgsCopy;
  LinkMsgs_L linkMsgsCopy;

  {
    boost::mutex::scoped_lock lock(*this->receiveMutex);

    std::copy(this->sceneMsgs.begin(), this->sceneMsgs.end(),
              std::back_inserter(sceneMsgsCopy));
    this->sceneMsgs.clear();

    std::copy(this->modelMsgs.begin(), this->modelMsgs.end(),
              std::back_inserter(modelMsgsCopy));
    this->modelMsgs.clear();

    std::copy(this->sensorMsgs.begin(), this->sensorMsgs.end(),
              std::back_inserter(sensorMsgsCopy));
    this->sensorMsgs.clear();

    std::copy(this->lightMsgs.begin(), this->lightMsgs.end(),
              std::back_inserter(lightMsgsCopy));
    this->lightMsgs.clear();

    this->visualMsgs.sort(VisualMessageLessOp);
    std::copy(this->visualMsgs.begin(), this->visualMsgs.end(),
              std::back_inserter(visualMsgsCopy));
    this->visualMsgs.clear();

    std::copy(this->jointMsgs.begin(), this->jointMsgs.end(),
              std::back_inserter(jointMsgsCopy));
    this->jointMsgs.clear();

    std::copy(this->linkMsgs.begin(), this->linkMsgs.end(),
              std::back_inserter(linkMsgsCopy));
    this->linkMsgs.clear();
  }

  // Process the scene messages. DO THIS FIRST
  for (sIter = sceneMsgsCopy.begin(); sIter != sceneMsgsCopy.end();)
  {
    if (this->ProcessSceneMsg(*sIter))
    {
      if (!this->initialized)
        RTShaderSystem::Instance()->UpdateShaders();
      this->initialized = true;
      sceneMsgsCopy.erase(sIter++);
    }
    else
      ++sIter;
  }

  // Process the model messages.
  for (modelIter = modelMsgsCopy.begin(); modelIter != modelMsgsCopy.end();)
  {
    if (this->ProcessModelMsg(**modelIter))
      modelMsgsCopy.erase(modelIter++);
    else
      ++modelIter;
  }

  // Process the sensor messages.
  for (sensorIter = sensorMsgsCopy.begin(); sensorIter != sensorMsgsCopy.end();)
  {
    if (this->ProcessSensorMsg(*sensorIter))
      sensorMsgsCopy.erase(sensorIter++);
    else
      ++sensorIter;
  }

  // Process the light messages.
  for (lightIter = lightMsgsCopy.begin(); lightIter != lightMsgsCopy.end();)
  {
    if (this->ProcessLightMsg(*lightIter))
      lightMsgsCopy.erase(lightIter++);
    else
      ++lightIter;
  }

  // Process the visual messages.
  for (visualIter = visualMsgsCopy.begin(); visualIter != visualMsgsCopy.end();)
  {
    if (this->ProcessVisualMsg(*visualIter))
      visualMsgsCopy.erase(visualIter++);
    else
      ++visualIter;
  }

  // Process the joint messages.
  for (jointIter = jointMsgsCopy.begin(); jointIter != jointMsgsCopy.end();)
  {
    if (this->ProcessJointMsg(*jointIter))
      jointMsgsCopy.erase(jointIter++);
    else
      ++jointIter;
  }

  // Process the link messages.
  for (linkIter = linkMsgsCopy.begin(); linkIter != linkMsgsCopy.end();)
  {
    if (this->ProcessLinkMsg(*linkIter))
      linkMsgsCopy.erase(linkIter++);
    else
      ++linkIter;
  }

  // Process the request messages
  for (rIter =  this->requestMsgs.begin();
      rIter != this->requestMsgs.end(); ++rIter)
  {
    this->ProcessRequestMsg(*rIter);
  }
  this->requestMsgs.clear();


  {
    boost::mutex::scoped_lock lock(*this->receiveMutex);

    std::copy(sceneMsgsCopy.begin(), sceneMsgsCopy.end(),
        std::front_inserter(this->sceneMsgs));

    std::copy(modelMsgsCopy.begin(), modelMsgsCopy.end(),
        std::front_inserter(this->modelMsgs));

    std::copy(sensorMsgsCopy.begin(), sensorMsgsCopy.end(),
        std::front_inserter(this->sensorMsgs));

    std::copy(lightMsgsCopy.begin(), lightMsgsCopy.end(),
        std::front_inserter(this->lightMsgs));

    std::copy(visualMsgsCopy.begin(), visualMsgsCopy.end(),
        std::front_inserter(this->visualMsgs));

    std::copy(jointMsgsCopy.begin(), jointMsgsCopy.end(),
        std::front_inserter(this->jointMsgs));

    std::copy(linkMsgsCopy.begin(), linkMsgsCopy.end(),
        std::front_inserter(this->linkMsgs));
  }

  {
    boost::recursive_mutex::scoped_lock lock(this->poseMsgMutex);

    // Process all the model messages last. Remove pose message from the list
    // only when a corresponding visual exits. We may receive pose updates
    // over the wire before  we recieve the visual
    pIter = this->poseMsgs.begin();
    while (pIter != this->poseMsgs.end())
    {
      Visual_M::iterator iter = this->visuals.find(pIter->first);
      if (iter != this->visuals.end() && iter->second)
      {
        // If an object is selected, don't let the physics engine move it.
        if (!this->selectedVis || this->selectionMode != "move" ||
            iter->first != this->selectedVis->GetId())
        {
          math::Pose pose = msgs::Convert(pIter->second);
          GZ_ASSERT(iter->second, "Visual pointer is NULL");
          iter->second->SetPose(pose);
          PoseMsgs_M::iterator prev = pIter++;
          this->poseMsgs.erase(prev);
        }
        else
          ++pIter;
      }
      else
        ++pIter;
    }

    // process skeleton pose msgs
    spIter = this->skeletonPoseMsgs.begin();
    while (spIter != this->skeletonPoseMsgs.end())
    {
      Visual_M::iterator iter = this->visuals.find((*spIter)->model_id());
      for (int i = 0; i < (*spIter)->pose_size(); i++)
      {
        const msgs::Pose& pose_msg = (*spIter)->pose(i);
        if (pose_msg.has_id())
        {
          Visual_M::iterator iter2 = this->visuals.find(pose_msg.id());
          if (iter2 != this->visuals.end())
          {
            // If an object is selected, don't let the physics engine move it.
            if (!this->selectedVis || this->selectionMode != "move" ||
              iter->first != this->selectedVis->GetId())
            {
              math::Pose pose = msgs::Convert(pose_msg);
              iter2->second->SetPose(pose);
            }
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

    // official time stamp of approval
    this->sceneSimTimePosesApplied = this->sceneSimTimePosesReceived;

    if (this->selectionMsg)
    {
      this->SelectVisual(this->selectionMsg->name(), "normal");
      this->selectionMsg.reset();
    }
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

  if ((_msg->type() == "ray" || _msg->type() == "gpu_ray") && _msg->visualize()
      && !_msg->topic().empty())
  {
    std::string rayVisualName = _msg->parent() + "::" + _msg->name();
    if (this->visuals.find(_msg->id()) == this->visuals.end())
    {
      VisualPtr parentVis = this->GetVisual(_msg->parent_id());
      if (!parentVis)
        return false;

      LaserVisualPtr laserVis(new LaserVisual(
            rayVisualName+"_GUIONLY_laser_vis", parentVis, _msg->topic()));
      laserVis->Load();
      laserVis->SetId(_msg->id());
      this->visuals[_msg->id()] = laserVis;
    }
  }
  else if ((_msg->type() == "sonar") && _msg->visualize()
      && !_msg->topic().empty())
  {
    std::string sonarVisualName = _msg->parent() + "::" + _msg->name();
    if (this->visuals.find(_msg->id()) == this->visuals.end())
    {
      VisualPtr parentVis = this->GetVisual(_msg->parent());
      if (!parentVis)
        return false;

      SonarVisualPtr sonarVis(new SonarVisual(
            sonarVisualName+"_GUIONLY_sonar_vis", parentVis, _msg->topic()));
      sonarVis->Load();
      sonarVis->SetId(_msg->id());
      this->visuals[_msg->id()] = sonarVis;
    }
  }
  else if ((_msg->type() == "force_torque") && _msg->visualize()
      && !_msg->topic().empty())
  {
    std::string wrenchVisualName = _msg->parent() + "::" + _msg->name();
    if (this->visuals.find(_msg->id()) == this->visuals.end())
    {
      ConstJointPtr jointMsg = this->joints[_msg->parent()];

      if (!jointMsg)
        return false;

      VisualPtr parentVis = this->GetVisual(jointMsg->child());

      if (!parentVis)
        return false;

      WrenchVisualPtr wrenchVis(new WrenchVisual(
            wrenchVisualName+"_GUIONLY_wrench_vis", parentVis,
            _msg->topic()));
      wrenchVis->Load(jointMsg);
      wrenchVis->SetId(_msg->id());
      this->visuals[_msg->id()] = wrenchVis;
    }
  }
  else if (_msg->type() == "camera" && _msg->visualize())
  {
    VisualPtr parentVis = this->GetVisual(_msg->parent_id());
    if (!parentVis)
      return false;

    // image size is 0 if renering is unavailable
    if (_msg->camera().image_size().x() > 0 &&
        _msg->camera().image_size().y() > 0)
    {
      Visual_M::iterator iter = this->visuals.find(_msg->id());
      if (iter == this->visuals.end())
      {
        CameraVisualPtr cameraVis(new CameraVisual(
              _msg->name()+"_GUIONLY_camera_vis", parentVis));

        // need to call AttachVisual in order for cameraVis to be added to
        // parentVis' children list so that it can be properly deleted.
        parentVis->AttachVisual(cameraVis);

        cameraVis->SetPose(msgs::Convert(_msg->pose()));
        cameraVis->SetId(_msg->id());
        cameraVis->Load(_msg->camera());
        this->visuals[cameraVis->GetId()] = cameraVis;
      }
    }
  }
  else if (_msg->type() == "contact" && _msg->visualize() &&
           !_msg->topic().empty())
  {
    ContactVisualPtr contactVis(new ContactVisual(
          _msg->name()+"__GUIONLY_CONTACT_VISUAL__",
          this->worldVisual, _msg->topic()));
    contactVis->SetId(_msg->id());

    this->contactVisId = _msg->id();
    this->visuals[contactVis->GetId()] = contactVis;
  }
  else if (_msg->type() == "rfidtag" && _msg->visualize() &&
           !_msg->topic().empty())
  {
    VisualPtr parentVis = this->GetVisual(_msg->parent());
    if (!parentVis)
      return false;

    RFIDTagVisualPtr rfidVis(new RFIDTagVisual(
          _msg->name() + "_GUIONLY_rfidtag_vis", parentVis, _msg->topic()));
    rfidVis->SetId(_msg->id());

    this->visuals[rfidVis->GetId()] = rfidVis;
  }
  else if (_msg->type() == "rfid" && _msg->visualize() &&
           !_msg->topic().empty())
  {
    VisualPtr parentVis = this->GetVisual(_msg->parent());
    if (!parentVis)
      return false;

    RFIDVisualPtr rfidVis(new RFIDVisual(
          _msg->name() + "_GUIONLY_rfid_vis", parentVis, _msg->topic()));
    rfidVis->SetId(_msg->id());
    this->visuals[rfidVis->GetId()] = rfidVis;
  }
  else if (_msg->type() == "wireless_transmitter" && _msg->visualize() &&
           !_msg->topic().empty())
  {
    VisualPtr parentVis = this->GetVisual(_msg->parent());
    if (!parentVis)
      return false;

    VisualPtr transmitterVis(new TransmitterVisual(
          _msg->name() + "_GUIONLY_transmitter_vis", parentVis, _msg->topic()));
    this->visuals[transmitterVis->GetId()] = transmitterVis;
    transmitterVis->Load();
  }

  return true;
}

/////////////////////////////////////////////////
bool Scene::ProcessLinkMsg(ConstLinkPtr &_msg)
{
  VisualPtr linkVis;

  if (_msg->has_id())
    linkVis = this->GetVisual(_msg->id());
  else
    linkVis = this->GetVisual(_msg->name());

  if (!linkVis)
  {
    gzerr << "No link visual with id[" << _msg->id() << "] and name["
      << _msg->name() << "]\n";
    return false;
  }

  if (!this->GetVisual(_msg->name() + "_COM_VISUAL__"))
  {
    this->CreateCOMVisual(_msg, linkVis);
  }

  for (int i = 0; i < _msg->projector_size(); ++i)
  {
    std::string pname = _msg->name() + "::" + _msg->projector(i).name();

    if (this->projectors.find(pname) == this->projectors.end())
    {
      Projector *projector = new Projector(linkVis);
      projector->Load(_msg->projector(i));
      projector->Toggle();
      this->projectors[pname] = projector;
    }
  }

  return true;
}

/////////////////////////////////////////////////
bool Scene::ProcessJointMsg(ConstJointPtr &_msg)
{
  VisualPtr childVis;

  if (_msg->has_child() && _msg->child() == "world")
    childVis = this->worldVisual;
  else if (_msg->has_child_id())
    childVis = this->GetVisual(_msg->child_id());

  if (!childVis)
    return false;

  JointVisualPtr jointVis(new JointVisual(
      _msg->name() + "_JOINT_VISUAL__", childVis));
  jointVis->Load(_msg);
  jointVis->SetVisible(this->showJoints);
  if (_msg->has_id())
    jointVis->SetId(_msg->id());

  this->visuals[jointVis->GetId()] = jointVis;

  return true;
}

/////////////////////////////////////////////////
void Scene::OnScene(ConstScenePtr &_msg)
{
  boost::mutex::scoped_lock lock(*this->receiveMutex);
  this->sceneMsgs.push_back(_msg);
}

/////////////////////////////////////////////////
void Scene::OnResponse(ConstResponsePtr &_msg)
{
  if (!this->requestMsg || _msg->id() != this->requestMsg->id())
    return;

  msgs::Scene sceneMsg;
  sceneMsg.ParseFromString(_msg->serialized_data());
  boost::shared_ptr<msgs::Scene> sm(new msgs::Scene(sceneMsg));
  this->sceneMsgs.push_back(sm);
  this->requestMsg = NULL;
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
  if (_msg->request() == "entity_info")
  {
    msgs::Response response;
    response.set_id(_msg->id());
    response.set_request(_msg->request());

    Light_M::iterator iter;
    iter = this->lights.find(_msg->data());
    if (iter != this->lights.end())
    {
      msgs::Light lightMsg;
      iter->second->FillMsg(lightMsg);

      std::string *serializedData = response.mutable_serialized_data();
      lightMsg.SerializeToString(serializedData);
      response.set_type(lightMsg.GetTypeName());

      response.set_response("success");
    }
    else
      response.set_response("failure");

    // this->responsePub->Publish(response);
  }
  else if (_msg->request() == "entity_delete")
  {
    Light_M::iterator lightIter = this->lights.find(_msg->data());

    // Check to see if the deleted entity is a light.
    if (lightIter != this->lights.end())
    {
      this->lights.erase(lightIter);
    }
    // Otherwise delete a visual
    else
    {
      VisualPtr visPtr;
      try
      {
        Visual_M::iterator iter;
        iter = this->visuals.find(boost::lexical_cast<uint32_t>(_msg->data()));
        visPtr = iter->second;
      } catch(...)
      {
        visPtr = this->GetVisual(_msg->data());
      }

      if (visPtr)
        this->RemoveVisual(visPtr);
    }
  }
  else if (_msg->request() == "show_contact")
  {
    this->ShowContacts(true);
  }
  else if (_msg->request() == "hide_contact")
  {
    this->ShowContacts(false);
  }
  else if (_msg->request() == "show_collision")
  {
    if (_msg->data() == "all")
      this->ShowCollisions(true);
    else
    {
      VisualPtr vis = this->GetVisual(_msg->data());
      if (vis)
        vis->ShowCollision(true);
      else
        gzerr << "Unable to find visual[" << _msg->data() << "]\n";
    }
  }
  else if (_msg->request() == "hide_collision")
  {
    if (_msg->data() == "all")
      this->ShowCollisions(false);
    else
    {
      VisualPtr vis = this->GetVisual(_msg->data());
      if (vis)
        vis->ShowCollision(false);
    }
  }
  else if (_msg->request() == "show_joints")
  {
    if (_msg->data() == "all")
      this->ShowJoints(true);
    else
    {
      VisualPtr vis = this->GetVisual(_msg->data());
      if (vis)
        vis->ShowJoints(true);
      else
        gzerr << "Unable to find joint visual[" << _msg->data() << "]\n";
    }
  }
  else if (_msg->request() == "hide_joints")
  {
    if (_msg->data() == "all")
      this->ShowJoints(false);
    else
    {
      VisualPtr vis = this->GetVisual(_msg->data());
      if (vis)
        vis->ShowJoints(false);
    }
  }
  else if (_msg->request() == "show_com")
  {
    if (_msg->data() == "all")
      this->ShowCOMs(true);
    else
    {
      VisualPtr vis = this->GetVisual(_msg->data());
      if (vis)
        vis->ShowCOM(true);
      else
        gzerr << "Unable to find joint visual[" << _msg->data() << "]\n";
    }
  }
  else if (_msg->request() == "hide_com")
  {
    if (_msg->data() == "all")
      this->ShowCOMs(false);
    else
    {
      VisualPtr vis = this->GetVisual(_msg->data());
      if (vis)
        vis->ShowCOM(false);
    }
  }
  else if (_msg->request() == "set_transparent")
  {
    if (_msg->data() == "all")
      this->SetTransparent(true);
    else
    {
      VisualPtr vis = this->GetVisual(_msg->data());
      if (vis)
        vis->SetTransparency(0.5);
    }
  }
  else if (_msg->request() == "set_wireframe")
  {
    if (_msg->data() == "all")
      this->SetWireframe(true);
    else
    {
      VisualPtr vis = this->GetVisual(_msg->data());
      if (vis)
        vis->SetWireframe(true);
    }
  }
  else if (_msg->request() == "set_solid")
  {
    if (_msg->data() == "all")
      this->SetWireframe(false);
    else
    {
      VisualPtr vis = this->GetVisual(_msg->data());
      if (vis)
        vis->SetWireframe(false);
    }
  }
  else if (_msg->request() == "set_opaque")
  {
    if (_msg->data() == "all")
      this->SetTransparent(false);
    else
    {
      VisualPtr vis = this->GetVisual(_msg->data());
      if (vis)
        vis->SetTransparency(0.0);
    }
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
  Visual_M::iterator iter = this->visuals.end();

  if (_msg->has_id())
    iter = this->visuals.find(_msg->id());
  else
  {
    VisualPtr vis = this->GetVisual(_msg->name());
    iter = vis ? this->visuals.find(vis->GetId()) : this->visuals.end();
  }

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
          this->terrain == NULL)
      {
        try
        {
          if (!this->terrain)
          {
            this->terrain = new Heightmap(shared_from_this());
            this->terrain->LoadFromMsg(_msg);
          }
          else
            gzerr << "Only one Heightmap can be created per Scene\n";
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
      if (_msg->has_id())
        iter = this->visuals.find(_msg->id());
      else
      {
        VisualPtr vis = this->GetVisual(_msg->name());
        iter = vis ? this->visuals.find(vis->GetId()) : this->visuals.end();
      }

      if (iter != this->visuals.end())
        gzerr << "Visual already exists. This shouldn't happen.\n";

      // Make sure the parent visual exists before trying to add a child
      // visual
      iter = this->visuals.find(_msg->parent_id());
      if (iter != this->visuals.end())
      {
        visual.reset(new Visual(_msg->name(), iter->second));
        if (_msg->has_id())
          visual->SetId(_msg->id());
      }
    }
    else
    {
      // Add a visual that is attached to the scene root
      visual.reset(new Visual(_msg->name(), this->worldVisual));
      if (_msg->has_id())
        visual->SetId(_msg->id());
    }

    if (visual)
    {
      result = true;
      visual->LoadFromMsg(_msg);
      this->visuals[visual->GetId()] = visual;
      if (visual->GetName().find("__COLLISION_VISUAL__") != std::string::npos ||
          visual->GetName().find("__SKELETON_VISUAL__") != std::string::npos)
      {
        visual->SetVisible(false);
        visual->SetVisibilityFlags(GZ_VISIBILITY_GUI);
      }

      visual->ShowCOM(this->showCOMs);
      visual->ShowCollision(this->showCollisions);
      visual->ShowJoints(this->showJoints);
      visual->SetTransparency(this->transparent ? 0.5 : 0.0);
      visual->SetWireframe(this->wireframe);

      visual->UpdateFromMsg(_msg);
    }
  }

  return result;
}

/////////////////////////////////////////////////
common::Time Scene::GetSimTime() const
{
  boost::mutex::scoped_lock lock(*this->receiveMutex);
  return this->sceneSimTimePosesApplied;
}

/////////////////////////////////////////////////
void Scene::OnPoseMsg(ConstPosesStampedPtr &_msg)
{
  boost::recursive_mutex::scoped_lock lock(this->poseMsgMutex);
  this->sceneSimTimePosesReceived =
    common::Time(_msg->time().sec(), _msg->time().nsec());

  for (int i = 0; i < _msg->pose_size(); ++i)
  {
    PoseMsgs_M::iterator iter = this->poseMsgs.find(_msg->pose(i).id());
    if (iter != this->poseMsgs.end())
      iter->second.CopyFrom(_msg->pose(i));
    else
      this->poseMsgs.insert(
          std::make_pair(_msg->pose(i).id(), _msg->pose(i)));
  }
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
bool Scene::ProcessLightMsg(ConstLightPtr &_msg)
{
  Light_M::iterator iter;
  iter = this->lights.find(_msg->name());

  if (iter == this->lights.end())
  {
    LightPtr light(new Light(shared_from_this()));
    light->LoadFromMsg(_msg);
    this->lights[_msg->name()] = light;
    RTShaderSystem::Instance()->UpdateShaders();
  }
  else
  {
    iter->second->UpdateFromMsg(_msg);
    RTShaderSystem::Instance()->UpdateShaders();
  }

  return true;
}

/////////////////////////////////////////////////
void Scene::OnSelectionMsg(ConstSelectionPtr &_msg)
{
  this->selectionMsg = _msg;
}

/////////////////////////////////////////////////
void Scene::OnModelMsg(ConstModelPtr &_msg)
{
  boost::mutex::scoped_lock lock(*this->receiveMutex);
  this->modelMsgs.push_back(_msg);
}

/////////////////////////////////////////////////
void Scene::OnSkyMsg(ConstSkyPtr &_msg)
{
  if (!this->skyx)
    return;

  this->skyx->setVisible(true);

  SkyX::VClouds::VClouds *vclouds =
    this->skyx->getVCloudsManager()->getVClouds();

  if (_msg->has_time())
  {
    Ogre::Vector3 t = this->skyxController->getTime();
    t.x = math::clamp(_msg->time(), 0.0, 24.0);
    this->skyxController->setTime(t);
  }

  if (_msg->has_sunrise())
  {
    Ogre::Vector3 t = this->skyxController->getTime();
    t.y = math::clamp(_msg->sunrise(), 0.0, 24.0);
    this->skyxController->setTime(t);
  }

  if (_msg->has_sunset())
  {
    Ogre::Vector3 t = this->skyxController->getTime();
    t.z = math::clamp(_msg->sunset(), 0.0, 24.0);
    this->skyxController->setTime(t);
  }

  if (_msg->has_wind_speed())
    vclouds->setWindSpeed(_msg->wind_speed());

  if (_msg->has_wind_direction())
    vclouds->setWindDirection(Ogre::Radian(_msg->wind_direction()));

  if (_msg->has_cloud_ambient())
  {
    vclouds->setAmbientFactors(Ogre::Vector4(
          _msg->cloud_ambient().r(),
          _msg->cloud_ambient().g(),
          _msg->cloud_ambient().b(),
          _msg->cloud_ambient().a()));
  }

  if (_msg->has_humidity())
  {
    Ogre::Vector2 wheater = vclouds->getWheater();
    vclouds->setWheater(math::clamp(_msg->humidity(), 0.0, 1.0),
                        wheater.y, true);
  }

  if (_msg->has_mean_cloud_size())
  {
    Ogre::Vector2 wheater = vclouds->getWheater();
    vclouds->setWheater(wheater.x,
                        math::clamp(_msg->mean_cloud_size(), 0.0, 1.0), true);
  }

  this->skyx->update(0);
}

/////////////////////////////////////////////////
void Scene::SetSky()
{
  // Create SkyX
  this->skyxController = new SkyX::BasicController();
  this->skyx = new SkyX::SkyX(this->manager, this->skyxController);
  this->skyx->create();

  this->skyx->setTimeMultiplier(0);

  // Set the time: x = current time[0-24], y = sunrise time[0-24],
  // z = sunset time[0-24]
  this->skyxController->setTime(Ogre::Vector3(10.0, 6.0, 20.0f));

  // Moon phase in [-1,1] range, where -1 means fully covered Moon,
  // 0 clear Moon and 1 fully covered Moon
  this->skyxController->setMoonPhase(0);

  this->skyx->getAtmosphereManager()->setOptions(
      SkyX::AtmosphereManager::Options(
        9.77501f,   // Inner radius
        10.2963f,   // Outer radius
        0.01f,      // Height position
        0.0017f,    // RayleighMultiplier
        0.000675f,  // MieMultiplier
        30,         // Sun Intensity
        Ogre::Vector3(0.57f, 0.54f, 0.44f),  // Wavelength
        -0.991f, 2.5f, 4));

  this->skyx->getVCloudsManager()->setWindSpeed(0.6);

  // Use true to update volumetric clouds based on the time multiplier
  this->skyx->getVCloudsManager()->setAutoupdate(false);

  SkyX::VClouds::VClouds *vclouds =
    this->skyx->getVCloudsManager()->getVClouds();

  // Set wind direction in radians
  vclouds->setWindDirection(Ogre::Radian(0.0));
  vclouds->setAmbientColor(Ogre::Vector3(0.9, 0.9, 1.0));

  // x = sun light power
  // y = sun beta multiplier
  // z = ambient color multiplier
  // w = distance attenuation
  vclouds->setLightResponse(Ogre::Vector4(0.9, 0.6, 0.5, 0.3));
  vclouds->setAmbientFactors(Ogre::Vector4(0.45, 0.3, 0.6, 0.1));
  vclouds->setWheater(.6, .6, false);

  if (true)
  {
    // Create VClouds
    if (!this->skyx->getVCloudsManager()->isCreated())
    {
      // SkyX::MeshManager::getSkydomeRadius(...) works for both finite and
      // infinite(=0) camera far clip distances
      this->skyx->getVCloudsManager()->create(2000.0);
      // this->skyx->getMeshManager()->getSkydomeRadius(mRenderingCamera));
    }
  }
  else
  {
    // Remove VClouds
    if (this->skyx->getVCloudsManager()->isCreated())
    {
      this->skyx->getVCloudsManager()->remove();
    }
  }

  // vclouds->getLightningManager()->setEnabled(preset.vcLightnings);
  // vclouds->getLightningManager()->setAverageLightningApparitionTime(
  //     preset.vcLightningsAT);
  // vclouds->getLightningManager()->setLightningColor(
  //     preset.vcLightningsColor);
  // vclouds->getLightningManager()->setLightningTimeMultiplier(
  //    preset.vcLightningsTM);

  Ogre::Root::getSingletonPtr()->addFrameListener(this->skyx);

  this->skyx->update(0);
  this->skyx->setVisible(false);
}

/////////////////////////////////////////////////
void Scene::SetShadowsEnabled(bool _value)
{
  this->sdf->GetElement("shadows")->Set(_value);

  if (RenderEngine::Instance()->GetRenderPathType() == RenderEngine::DEFERRED)
  {
#if OGRE_VERSION_MAJOR >= 1 && OGRE_VERSION_MINOR >= 8
    this->manager->setShadowTechnique(Ogre::SHADOWTYPE_TEXTURE_ADDITIVE);
    this->manager->setShadowTextureCasterMaterial(
        "DeferredRendering/Shadows/RSMCaster_Spot");
    this->manager->setShadowTextureCount(1);
    this->manager->setShadowFarDistance(150);
    // Use a value of "2" to use a different depth buffer pool and
    // avoid sharing this with the Backbuffer's
    this->manager->setShadowTextureConfig(0, 1024, 1024,
        Ogre::PF_FLOAT32_RGBA, 0, 2);
    this->manager->setShadowDirectionalLightExtrusionDistance(75);
    this->manager->setShadowCasterRenderBackFaces(false);
    this->manager->setShadowTextureSelfShadow(true);
    this->manager->setShadowDirLightTextureOffset(1.75);
#endif
  }
  else if (RenderEngine::Instance()->GetRenderPathType() ==
           RenderEngine::FORWARD)
  {
    // RT Shader shadows
    if (_value)
      RTShaderSystem::Instance()->ApplyShadows(shared_from_this());
    else
      RTShaderSystem::Instance()->RemoveShadows(shared_from_this());
  }
  else
  {
    this->manager->setShadowCasterRenderBackFaces(false);
    this->manager->setShadowTextureSize(512);

    // The default shadows.
    if (_value && this->manager->getShadowTechnique()
        != Ogre::SHADOWTYPE_TEXTURE_ADDITIVE)
      this->manager->setShadowTechnique(Ogre::SHADOWTYPE_TEXTURE_ADDITIVE);
    else
      this->manager->setShadowTechnique(Ogre::SHADOWTYPE_NONE);
  }
}

/////////////////////////////////////////////////
bool Scene::GetShadowsEnabled() const
{
  return this->sdf->Get<bool>("shadows");
}

/////////////////////////////////////////////////
void Scene::AddVisual(VisualPtr _vis)
{
  if (this->visuals.find(_vis->GetId()) != this->visuals.end())
    gzerr << "Duplicate visuals detected[" << _vis->GetName() << "]\n";

  this->visuals[_vis->GetId()] = _vis;
}

/////////////////////////////////////////////////
void Scene::RemoveVisual(VisualPtr _vis)
{
  if (_vis)
  {
    // Remove all projectors attached to the visual
    std::map<std::string, Projector *>::iterator piter =
      this->projectors.begin();
    while (piter != this->projectors.end())
    {
      // Check to see if the projector is a child of the visual that is
      // being removed.
      if (piter->second->GetParent()->GetRootVisual()->GetName() ==
          _vis->GetRootVisual()->GetName())
      {
        delete piter->second;
        this->projectors.erase(piter++);
      }
      else
        ++piter;
    }

    // Delete the visual
    Visual_M::iterator iter = this->visuals.find(_vis->GetId());
    if (iter != this->visuals.end())
    {
      iter->second->Fini();
      this->visuals.erase(iter);
    }

    if (this->selectedVis && this->selectedVis->GetId() == _vis->GetId())
      this->selectedVis.reset();
  }
}

/////////////////////////////////////////////////
void Scene::AddLight(LightPtr _light)
{
  std::string n = this->StripSceneName(_light->GetName());
  Light_M::iterator iter = this->lights.find(n);
  if (iter != this->lights.end())
    gzerr << "Duplicate lights detected[" << _light->GetName() << "]\n";

  this->lights[n] = _light;
}

/////////////////////////////////////////////////
void Scene::RemoveLight(LightPtr _light)
{
  if (_light)
  {
    // Delete the light
    std::string n = this->StripSceneName(_light->GetName());
    this->lights.erase(n);
  }
}

/////////////////////////////////////////////////
void Scene::SetGrid(bool _enabled)
{
  if (_enabled && this->grids.empty())
  {
    Grid *grid = new Grid(this, 20, 1, 10, common::Color(0.3, 0.3, 0.3, 0.5));
    grid->Init();
    this->grids.push_back(grid);

    grid = new Grid(this, 4, 5, 20, common::Color(0.8, 0.8, 0.8, 0.5));
    grid->Init();
    this->grids.push_back(grid);
  }
  else
  {
    for (uint32_t i = 0; i < this->grids.size(); ++i)
    {
      this->grids[i]->Enable(_enabled);
    }
  }
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
  boost::mutex::scoped_lock lock(*this->receiveMutex);
  return this->terrain;
}

/////////////////////////////////////////////////
void Scene::CreateCOMVisual(ConstLinkPtr &_msg, VisualPtr _linkVisual)
{
  COMVisualPtr comVis(new COMVisual(_msg->name() + "_COM_VISUAL__",
                                    _linkVisual));
  comVis->Load(_msg);
  comVis->SetVisible(this->showCOMs);
  this->visuals[comVis->GetId()] = comVis;
}

/////////////////////////////////////////////////
void Scene::CreateCOMVisual(sdf::ElementPtr _elem, VisualPtr _linkVisual)
{
  COMVisualPtr comVis(new COMVisual(_linkVisual->GetName() + "_COM_VISUAL__",
                                    _linkVisual));
  comVis->Load(_elem);
  comVis->SetVisible(false);
  this->visuals[comVis->GetId()] = comVis;
}

/////////////////////////////////////////////////
void Scene::SetWireframe(bool _show)
{
  this->wireframe = _show;
  for (Visual_M::iterator iter = this->visuals.begin();
       iter != this->visuals.end(); ++iter)
  {
    iter->second->SetWireframe(_show);
  }

  if (this->terrain)
    this->terrain->SetWireframe(_show);
}

/////////////////////////////////////////////////
void Scene::SetTransparent(bool _show)
{
  this->transparent = _show;
  for (Visual_M::iterator iter = this->visuals.begin();
       iter != this->visuals.end(); ++iter)
  {
    iter->second->SetTransparency(_show ? 0.5 : 0.0);
  }
}

/////////////////////////////////////////////////
void Scene::ShowCOMs(bool _show)
{
  this->showCOMs = _show;
  for (Visual_M::iterator iter = this->visuals.begin();
       iter != this->visuals.end(); ++iter)
  {
    iter->second->ShowCOM(_show);
  }
}

/////////////////////////////////////////////////
void Scene::ShowCollisions(bool _show)
{
  this->showCollisions = _show;
  for (Visual_M::iterator iter = this->visuals.begin();
       iter != this->visuals.end(); ++iter)
  {
    iter->second->ShowCollision(_show);
  }
}

/////////////////////////////////////////////////
void Scene::ShowJoints(bool _show)
{
  this->showJoints = _show;
  for (Visual_M::iterator iter = this->visuals.begin();
       iter != this->visuals.end(); ++iter)
  {
    iter->second->ShowJoints(_show);
  }
}

/////////////////////////////////////////////////
void Scene::ShowContacts(bool _show)
{
  ContactVisualPtr vis;

  if (this->contactVisId == GZ_UINT32_MAX && _show)
  {
    vis.reset(new ContactVisual("__GUIONLY_CONTACT_VISUAL__",
              this->worldVisual, "~/physics/contacts"));
    vis->SetEnabled(_show);
    this->contactVisId = vis->GetId();
    this->visuals[this->contactVisId] = vis;
  }
  else
    vis = boost::dynamic_pointer_cast<ContactVisual>(
        this->visuals[this->contactVisId]);

  if (vis)
    vis->SetEnabled(_show);
  else
    gzerr << "Unable to get contact visualization. This should never happen.\n";
}

/////////////////////////////////////////////////
void Scene::ShowClouds(bool _show)
{
  if (!this->skyx)
    return;

  SkyX::VCloudsManager *mgr = this->skyx->getVCloudsManager();
  if (mgr)
  {
    SkyX::VClouds::VClouds *vclouds =
        this->skyx->getVCloudsManager()->getVClouds();
    if (vclouds)
      vclouds->setVisible(_show);
  }
}

/////////////////////////////////////////////////
bool Scene::GetShowClouds() const
{
  if (!this->skyx)
    return false;

  SkyX::VCloudsManager *mgr = this->skyx->getVCloudsManager();
  if (mgr)
  {
    SkyX::VClouds::VClouds *vclouds =
        this->skyx->getVCloudsManager()->getVClouds();
    if (vclouds)
      return vclouds->isVisible();
  }

  return false;
}

/////////////////////////////////////////////////
void Scene::SetSkyXMode(unsigned int _mode)
{
  /// \todo This function is currently called on initialization of rendering
  /// based sensors to disable clouds and moon. More testing is required to
  /// make sure it functions correctly when called during a render update,
  /// issue #693.

  if (!this->skyx)
    return;

  bool enabled = _mode != GZ_SKYX_NONE;
  this->skyx->setEnabled(enabled);

  if (!enabled)
    return;

  this->skyx->setCloudsEnabled(_mode & GZ_SKYX_CLOUDS);
  this->skyx->setMoonEnabled(_mode & GZ_SKYX_MOON);
}

/////////////////////////////////////////////////
void Scene::RemoveProjectors()
{
  for (std::map<std::string, Projector *>::iterator iter =
      this->projectors.begin(); iter != this->projectors.end(); ++iter)
  {
    delete iter->second;
  }
  this->projectors.clear();
}
