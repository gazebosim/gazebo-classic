/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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
#include <boost/bind.hpp>
#include <boost/lexical_cast.hpp>

#include "gazebo/rendering/skyx/include/SkyX.h"
#include "gazebo/rendering/ogre_gazebo.h"

#include "gazebo/msgs/msgs.hh"

#include "gazebo/common/Exception.hh"
#include "gazebo/common/Assert.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/rendering/Road2d.hh"
#include "gazebo/rendering/Projector.hh"
#include "gazebo/rendering/Heightmap.hh"
#include "gazebo/rendering/RenderEvents.hh"
#include "gazebo/rendering/LaserVisual.hh"
#include "gazebo/rendering/SonarVisual.hh"
#include "gazebo/rendering/WrenchVisual.hh"
#include "gazebo/rendering/CameraVisual.hh"
#include "gazebo/rendering/LogicalCameraVisual.hh"
#include "gazebo/rendering/JointVisual.hh"
#include "gazebo/rendering/COMVisual.hh"
#include "gazebo/rendering/InertiaVisual.hh"
#include "gazebo/rendering/LinkFrameVisual.hh"
#include "gazebo/rendering/ContactVisual.hh"
#include "gazebo/rendering/Conversions.hh"
#include "gazebo/rendering/Light.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/rendering/RenderEngine.hh"
#include "gazebo/rendering/UserCamera.hh"
#include "gazebo/rendering/Camera.hh"
#include "gazebo/rendering/WideAngleCamera.hh"
#include "gazebo/rendering/DepthCamera.hh"
#include "gazebo/rendering/GpuLaser.hh"
#include "gazebo/rendering/Grid.hh"
#include "gazebo/rendering/OriginVisual.hh"
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

#include "gazebo/rendering/ScenePrivate.hh"
#include "gazebo/rendering/Scene.hh"

#ifdef HAVE_OCULUS
#include "gazebo/rendering/OculusCamera.hh"
#endif

using namespace gazebo;
using namespace rendering;

uint32_t ScenePrivate::idCounter = 0;

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
  : dataPtr(new ScenePrivate)
{
  // \todo: This is a hack. There is no guarantee (other than the
  // improbability of creating an extreme number of visuals), that
  // this contactVisId is unique.
  this->dataPtr->contactVisId = GZ_UINT32_MAX;

  this->dataPtr->initialized = false;
  this->dataPtr->showCOMs = false;
  this->dataPtr->showInertias = false;
  this->dataPtr->showLinkFrames = false;
  this->dataPtr->showCollisions = false;
  this->dataPtr->showJoints = false;
  this->dataPtr->transparent = false;
  this->dataPtr->wireframe = false;

  this->dataPtr->requestMsg = NULL;
  this->dataPtr->enableVisualizations = _enableVisualizations;
  this->dataPtr->node = transport::NodePtr(new transport::Node());
  this->dataPtr->node->Init(_name);
  this->dataPtr->id = ScenePrivate::idCounter++;
  this->dataPtr->idString = boost::lexical_cast<std::string>(this->dataPtr->id);

  this->dataPtr->name = _name;
  this->dataPtr->manager = NULL;
  this->dataPtr->raySceneQuery = NULL;
  this->dataPtr->skyx = NULL;

  this->dataPtr->receiveMutex = new boost::mutex();

  this->dataPtr->connections.push_back(
      event::Events::ConnectPreRender(boost::bind(&Scene::PreRender, this)));

  this->dataPtr->connections.push_back(
      rendering::Events::ConnectToggleLayer(
        boost::bind(&Scene::ToggleLayer, this, _1)));

  this->dataPtr->sensorSub = this->dataPtr->node->Subscribe("~/sensor",
                                          &Scene::OnSensorMsg, this, true);
  this->dataPtr->visSub =
      this->dataPtr->node->Subscribe("~/visual", &Scene::OnVisualMsg, this);

  this->dataPtr->lightPub =
      this->dataPtr->node->Advertise<msgs::Light>("~/light");

  this->dataPtr->lightSub =
      this->dataPtr->node->Subscribe("~/light", &Scene::OnLightMsg, this);

  if (_isServer)
  {
    this->dataPtr->poseSub = this->dataPtr->node->Subscribe("~/pose/local/info",
        &Scene::OnPoseMsg, this);
  }
  else
  {
    this->dataPtr->poseSub = this->dataPtr->node->Subscribe("~/pose/info",
        &Scene::OnPoseMsg, this);
  }

  this->dataPtr->jointSub =
      this->dataPtr->node->Subscribe("~/joint", &Scene::OnJointMsg, this);
  this->dataPtr->skeletonPoseSub =
      this->dataPtr->node->Subscribe("~/skeleton_pose/info",
      &Scene::OnSkeletonPoseMsg, this);
  this->dataPtr->skySub =
      this->dataPtr->node->Subscribe("~/sky", &Scene::OnSkyMsg, this);
  this->dataPtr->modelInfoSub = this->dataPtr->node->Subscribe("~/model/info",
                                             &Scene::OnModelMsg, this);

  this->dataPtr->requestPub =
      this->dataPtr->node->Advertise<msgs::Request>("~/request");

  this->dataPtr->requestSub = this->dataPtr->node->Subscribe("~/request",
      &Scene::OnRequest, this);

  this->dataPtr->responseSub = this->dataPtr->node->Subscribe("~/response",
      &Scene::OnResponse, this, true);
  this->dataPtr->sceneSub =
      this->dataPtr->node->Subscribe("~/scene", &Scene::OnScene, this);

  this->dataPtr->sdf.reset(new sdf::Element);
  sdf::initFile("scene.sdf", this->dataPtr->sdf);

  this->dataPtr->terrain = NULL;
  this->dataPtr->selectedVis.reset();

  this->dataPtr->sceneSimTimePosesApplied = common::Time();
  this->dataPtr->sceneSimTimePosesReceived = common::Time();
}

//////////////////////////////////////////////////
void Scene::Clear()
{
  this->dataPtr->node->Fini();
  this->dataPtr->modelMsgs.clear();
  this->dataPtr->visualMsgs.clear();
  this->dataPtr->lightMsgs.clear();
  this->dataPtr->poseMsgs.clear();
  this->dataPtr->sceneMsgs.clear();
  this->dataPtr->jointMsgs.clear();
  this->dataPtr->linkMsgs.clear();
  this->dataPtr->sensorMsgs.clear();

  this->dataPtr->poseSub.reset();
  this->dataPtr->jointSub.reset();
  this->dataPtr->sensorSub.reset();
  this->dataPtr->sceneSub.reset();
  this->dataPtr->skeletonPoseSub.reset();
  this->dataPtr->visSub.reset();
  this->dataPtr->skySub.reset();
  this->dataPtr->lightSub.reset();
  this->dataPtr->requestSub.reset();
  this->dataPtr->responseSub.reset();
  this->dataPtr->modelInfoSub.reset();
  this->dataPtr->lightPub.reset();
  this->dataPtr->responsePub.reset();
  this->dataPtr->requestPub.reset();

  this->dataPtr->joints.clear();

  delete this->dataPtr->terrain;
  this->dataPtr->terrain = NULL;

  while (!this->dataPtr->visuals.empty())
    this->RemoveVisual(this->dataPtr->visuals.begin()->first);

  this->dataPtr->visuals.clear();

  if (this->dataPtr->originVisual)
  {
    this->dataPtr->originVisual->Fini();
    this->dataPtr->originVisual.reset();
  }

  if (this->dataPtr->worldVisual)
  {
    this->dataPtr->worldVisual->Fini();
    this->dataPtr->worldVisual.reset();
  }

  while (!this->dataPtr->lights.empty())
    if (this->dataPtr->lights.begin()->second)
      this->RemoveLight(this->dataPtr->lights.begin()->second);
  this->dataPtr->lights.clear();

  for (uint32_t i = 0; i < this->dataPtr->grids.size(); ++i)
    delete this->dataPtr->grids[i];
  this->dataPtr->grids.clear();

  for (unsigned int i = 0; i < this->dataPtr->cameras.size(); ++i)
    this->dataPtr->cameras[i]->Fini();
  this->dataPtr->cameras.clear();

  for (unsigned int i = 0; i < this->dataPtr->userCameras.size(); ++i)
    this->dataPtr->userCameras[i]->Fini();
  this->dataPtr->userCameras.clear();

  delete this->dataPtr->skyx;
  this->dataPtr->skyx = NULL;

  RTShaderSystem::Instance()->RemoveScene(this->GetName());

  this->dataPtr->connections.clear();

  this->dataPtr->initialized = false;
}

//////////////////////////////////////////////////
Scene::~Scene()
{
  delete this->dataPtr->requestMsg;
  this->dataPtr->requestMsg = NULL;
  delete this->dataPtr->receiveMutex;
  this->dataPtr->receiveMutex = NULL;

  // raySceneQuery deletion handled by ogre
  this->dataPtr->raySceneQuery= NULL;

  this->Clear();

  this->dataPtr->sdf->Reset();
  this->dataPtr->sdf.reset();

  delete this->dataPtr;
  this->dataPtr = NULL;
}

//////////////////////////////////////////////////
void Scene::Load(sdf::ElementPtr _sdf)
{
  this->dataPtr->sdf->Copy(_sdf);
  this->Load();
}

//////////////////////////////////////////////////
void Scene::Load()
{
  this->dataPtr->initialized = false;
  Ogre::Root *root = RenderEngine::Instance()->root;

  if (this->dataPtr->manager)
    root->destroySceneManager(this->dataPtr->manager);

  this->dataPtr->manager = root->createSceneManager(Ogre::ST_GENERIC);
  this->dataPtr->manager->setAmbientLight(
      Ogre::ColourValue(0.1, 0.1, 0.1, 0.1));

#if OGRE_VERSION_MAJOR > 1 || OGRE_VERSION_MINOR >= 9
  this->dataPtr->manager->addRenderQueueListener(
      RenderEngine::Instance()->GetOverlaySystem());
#endif
}

//////////////////////////////////////////////////
VisualPtr Scene::GetWorldVisual() const
{
  return this->dataPtr->worldVisual;
}

//////////////////////////////////////////////////
void Scene::Init()
{
  this->dataPtr->worldVisual.reset(new Visual("__world_node__",
      shared_from_this()));
  this->dataPtr->worldVisual->SetId(0);
  this->dataPtr->visuals[0] = this->dataPtr->worldVisual;

  // RTShader system self-enables if the render path type is FORWARD,
  RTShaderSystem::Instance()->AddScene(shared_from_this());
  RTShaderSystem::Instance()->ApplyShadows(shared_from_this());

  if (RenderEngine::Instance()->GetRenderPathType() == RenderEngine::DEFERRED)
    this->InitDeferredShading();

  for (uint32_t i = 0; i < this->dataPtr->grids.size(); ++i)
    this->dataPtr->grids[i]->Init();

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
  if (this->dataPtr->sdf->HasElement("fog"))
  {
    sdf::ElementPtr fogElem = this->dataPtr->sdf->GetElement("fog");
    this->SetFog(fogElem->Get<std::string>("type"),
                 fogElem->Get<common::Color>("color"),
                 fogElem->Get<double>("density"),
                 fogElem->Get<double>("start"),
                 fogElem->Get<double>("end"));
  }

  // Create ray scene query
  this->dataPtr->raySceneQuery =
      this->dataPtr->manager->createRayQuery(Ogre::Ray());
  this->dataPtr->raySceneQuery->setSortByDistance(true);
  this->dataPtr->raySceneQuery->setQueryMask(
      Ogre::SceneManager::ENTITY_TYPE_MASK);

  // Force shadows on.
  this->SetShadowsEnabled(true);

  // Create origin visual
  this->dataPtr->originVisual.reset(new OriginVisual("__WORLD_ORIGIN__",
      this->dataPtr->worldVisual));
  this->dataPtr->originVisual->Load();

  this->dataPtr->requestPub->WaitForConnection();
  this->dataPtr->requestMsg = msgs::CreateRequest("scene_info");
  this->dataPtr->requestPub->Publish(*this->dataPtr->requestMsg);

  Road2d *road = new Road2d();
  road->Load(this->dataPtr->worldVisual);
}

//////////////////////////////////////////////////
bool Scene::GetInitialized() const
{
  return this->dataPtr->initialized;
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
    this->dataPtr->manager->createInstanceManager("VPL_InstanceMgr",
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
  return this->dataPtr->manager;
}

//////////////////////////////////////////////////
std::string Scene::GetName() const
{
  return this->dataPtr->name;
}

//////////////////////////////////////////////////
void Scene::SetAmbientColor(const common::Color &_color)
{
  this->dataPtr->sdf->GetElement("ambient")->Set(_color);

  // Ambient lighting
  if (this->dataPtr->manager &&
      Conversions::Convert(this->dataPtr->manager->getAmbientLight()) != _color)
  {
    this->dataPtr->manager->setAmbientLight(Conversions::Convert(_color));
  }
}

//////////////////////////////////////////////////
common::Color Scene::GetAmbientColor() const
{
  return this->dataPtr->sdf->Get<common::Color>("ambient");
}

//////////////////////////////////////////////////
void Scene::SetBackgroundColor(const common::Color &_color)
{
  this->dataPtr->sdf->GetElement("background")->Set(_color);
  Ogre::ColourValue clr = Conversions::Convert(_color);

  std::vector<CameraPtr>::iterator iter;
  for (iter = this->dataPtr->cameras.begin();
      iter != this->dataPtr->cameras.end(); ++iter)
  {
    if ((*iter)->GetViewport() &&
        (*iter)->GetViewport()->getBackgroundColour() != clr)
      (*iter)->GetViewport()->setBackgroundColour(clr);
  }

  std::vector<UserCameraPtr>::iterator iter2;
  for (iter2 = this->dataPtr->userCameras.begin();
       iter2 != this->dataPtr->userCameras.end(); ++iter2)
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
  return this->dataPtr->sdf->Get<common::Color>("background");
}

//////////////////////////////////////////////////
void Scene::CreateGrid(uint32_t cell_count, float cell_length,
                       float line_width, const common::Color &color)
{
  Grid *grid = new Grid(this, cell_count, cell_length, line_width, color);

  if (this->dataPtr->manager)
    grid->Init();

  this->dataPtr->grids.push_back(grid);
}

//////////////////////////////////////////////////
Grid *Scene::GetGrid(uint32_t index) const
{
  if (index >= this->dataPtr->grids.size())
  {
    gzerr << "Scene::GetGrid() Invalid index\n";
    return NULL;
  }

  return this->dataPtr->grids[index];
}

//////////////////////////////////////////////////
uint32_t Scene::GetGridCount() const
{
  return this->dataPtr->grids.size();
}

//////////////////////////////////////////////////
CameraPtr Scene::CreateCamera(const std::string &_name, bool _autoRender)
{
  CameraPtr camera(new Camera(_name, shared_from_this(), _autoRender));
  this->dataPtr->cameras.push_back(camera);

  return camera;
}

//////////////////////////////////////////////////
DepthCameraPtr Scene::CreateDepthCamera(const std::string &_name,
                                        bool _autoRender)
{
  DepthCameraPtr camera(new DepthCamera(this->dataPtr->name + "::" + _name,
        shared_from_this(), _autoRender));
  this->dataPtr->cameras.push_back(camera);

  return camera;
}

//////////////////////////////////////////////////
WideAngleCameraPtr Scene::CreateWideAngleCamera(const std::string &_name,
                                                bool _autoRender)
{
  WideAngleCameraPtr camera(new WideAngleCamera(_name,
        shared_from_this(), _autoRender));
  this->dataPtr->cameras.push_back(camera);

  return camera;
}

//////////////////////////////////////////////////
GpuLaserPtr Scene::CreateGpuLaser(const std::string &_name,
                                        bool _autoRender)
{
  GpuLaserPtr camera(new GpuLaser(this->dataPtr->name + "::" + _name,
        shared_from_this(), _autoRender));
  this->dataPtr->cameras.push_back(camera);

  return camera;
}

//////////////////////////////////////////////////
uint32_t Scene::GetCameraCount() const
{
  return this->dataPtr->cameras.size();
}

//////////////////////////////////////////////////
CameraPtr Scene::GetCamera(uint32_t index) const
{
  CameraPtr cam;

  if (index < this->dataPtr->cameras.size())
    cam = this->dataPtr->cameras[index];

  return cam;
}

//////////////////////////////////////////////////
CameraPtr Scene::GetCamera(const std::string &_name) const
{
  CameraPtr result;
  std::vector<CameraPtr>::const_iterator iter;
  for (iter = this->dataPtr->cameras.begin();
      iter != this->dataPtr->cameras.end(); ++iter)
  {
    if ((*iter)->GetName() == _name)
      result = *iter;
  }

  return result;
}

#ifdef HAVE_OCULUS
//////////////////////////////////////////////////
OculusCameraPtr Scene::CreateOculusCamera(const std::string &_name)
{
  OculusCameraPtr camera(new OculusCamera(_name, shared_from_this()));

  if (camera->Ready())
  {
    camera->Load();
    camera->Init();
    this->dataPtr->oculusCameras.push_back(camera);
  }

  return camera;
}

//////////////////////////////////////////////////
uint32_t Scene::GetOculusCameraCount() const
{
  return this->dataPtr->oculusCameras.size();
}
#endif

//////////////////////////////////////////////////
UserCameraPtr Scene::CreateUserCamera(const std::string &_name,
                                      bool _stereoEnabled)
{
  UserCameraPtr camera(new UserCamera(_name, shared_from_this(),
        _stereoEnabled));
  camera->Load();
  camera->Init();
  this->dataPtr->userCameras.push_back(camera);

  return camera;
}

//////////////////////////////////////////////////
uint32_t Scene::GetUserCameraCount() const
{
  return this->dataPtr->userCameras.size();
}

//////////////////////////////////////////////////
UserCameraPtr Scene::GetUserCamera(uint32_t index) const
{
  UserCameraPtr cam;

  if (index < this->dataPtr->userCameras.size())
    cam = this->dataPtr->userCameras[index];

  return cam;
}

//////////////////////////////////////////////////
void Scene::RemoveCamera(const std::string &_name)
{
  std::vector<CameraPtr>::iterator iter;
  for (iter = this->dataPtr->cameras.begin();
      iter != this->dataPtr->cameras.end(); ++iter)
  {
    if ((*iter)->GetName() == _name)
    {
      (*iter)->Fini();
      (*iter).reset();
      this->dataPtr->cameras.erase(iter);
      break;
    }
  }
}

//////////////////////////////////////////////////
LightPtr Scene::GetLight(const std::string &_name) const
{
  LightPtr result;
  std::string n = this->StripSceneName(_name);
  Light_M::const_iterator iter = this->dataPtr->lights.find(n);
  if (iter != this->dataPtr->lights.end())
    result = iter->second;
  return result;
}

//////////////////////////////////////////////////
uint32_t Scene::GetLightCount() const
{
  return this->dataPtr->lights.size();
}

//////////////////////////////////////////////////
LightPtr Scene::GetLight(uint32_t _index) const
{
  LightPtr result;
  if (_index < this->dataPtr->lights.size())
  {
    Light_M::const_iterator iter = this->dataPtr->lights.begin();
    std::advance(iter, _index);
    result = iter->second;
  }
  else
  {
    gzerr << "Error: light index(" << _index << ") larger than light count("
          << this->dataPtr->lights.size() << "\n";
  }

  return result;
}

//////////////////////////////////////////////////
VisualPtr Scene::GetVisual(uint32_t _id) const
{
  Visual_M::const_iterator iter = this->dataPtr->visuals.find(_id);
  if (iter != this->dataPtr->visuals.end())
    return iter->second;
  return VisualPtr();
}

//////////////////////////////////////////////////
VisualPtr Scene::GetVisual(const std::string &_name) const
{
  VisualPtr result;

  Visual_M::const_iterator iter;
  for (iter = this->dataPtr->visuals.begin();
      iter != this->dataPtr->visuals.end(); ++iter)
  {
    if (iter->second->GetName() == _name)
      break;
  }

  if (iter != this->dataPtr->visuals.end())
    result = iter->second;
  else
  {
    std::string otherName = this->GetName() + "::" + _name;
    for (iter = this->dataPtr->visuals.begin();
        iter != this->dataPtr->visuals.end(); ++iter)
    {
      if (iter->second->GetName() == otherName)
        break;
    }

    if (iter != this->dataPtr->visuals.end())
      result = iter->second;
  }

  return result;
}

//////////////////////////////////////////////////
uint32_t Scene::GetVisualCount() const
{
  return this->dataPtr->visuals.size();
}

//////////////////////////////////////////////////
void Scene::SelectVisual(const std::string &_name, const std::string &_mode)
{
  this->dataPtr->selectedVis = this->GetVisual(_name);
  this->dataPtr->selectionMode = _mode;
}

//////////////////////////////////////////////////
VisualPtr Scene::GetSelectedVisual() const
{
  return this->dataPtr->selectedVis;
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
        closestEntity->getUserObjectBindings().getUserAny().getType()
        == typeid(std::string))
    {
      try
      {
        _mod = Ogre::any_cast<std::string>(
            closestEntity->getUserObjectBindings().getUserAny());
      }
      catch(boost::bad_any_cast &e)
      {
        gzerr << "boost any_cast error:" << e.what() << "\n";
      }
    }

    try
    {
      visual = this->GetVisual(Ogre::any_cast<std::string>(
            closestEntity->getUserObjectBindings().getUserAny()));
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

  if (!this->dataPtr->raySceneQuery)
  {
    this->dataPtr->raySceneQuery =
        this->dataPtr->manager->createRayQuery(Ogre::Ray());
  }
  this->dataPtr->raySceneQuery->setRay(ray);
  this->dataPtr->raySceneQuery->setSortByDistance(true, 0);

  // Perform the scene query
  Ogre::RaySceneQueryResult &result = this->dataPtr->raySceneQuery->execute();
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
  if (this->dataPtr->terrain)
  {
    double terrainHeight =
        this->dataPtr->terrain->GetHeight(_pt.x, _pt.y, _pt.z);
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

  if (!this->dataPtr->raySceneQuery)
  {
    this->dataPtr->raySceneQuery =
        this->dataPtr->manager->createRayQuery(Ogre::Ray());
  }

  this->dataPtr->raySceneQuery->setRay(ray);
  this->dataPtr->raySceneQuery->setSortByDistance(true, 0);

  // Perform the scene query
  Ogre::RaySceneQueryResult &result = this->dataPtr->raySceneQuery->execute();
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
                ogreEntity->getUserObjectBindings().getUserAny()));
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
            closestEntity->getUserObjectBindings().getUserAny()));
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

  this->dataPtr->raySceneQuery->setRay(mouseRay);

  // Perform the scene query
  Ogre::RaySceneQueryResult &result = this->dataPtr->raySceneQuery->execute();
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

  this->dataPtr->raySceneQuery->setSortByDistance(true);
  this->dataPtr->raySceneQuery->setRay(mouseRay);

  // Perform the scene query
  Ogre::RaySceneQueryResult &result = this->dataPtr->raySceneQuery->execute();
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
  if (distance <= 0.0 && this->dataPtr->terrain)
  {
    // The terrain uses a special ray intersection test.
    Ogre::TerrainGroup::RayResult terrainResult =
      this->dataPtr->terrain->GetOgreTerrain()->rayIntersects(mouseRay);

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
  this->PrintSceneGraphHelper("", this->dataPtr->manager->getRootSceneNode());
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
  for (int i = 0; i < numAttachedObjs; ++i)
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

  for (uint32_t i = 0; i < node_->numChildren(); ++i)
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

  if (this->dataPtr->manager->hasManualObject(name_))
  {
    sceneNode = this->dataPtr->manager->getSceneNode(name_);
    obj = this->dataPtr->manager->getManualObject(name_);
    attached = true;
  }
  else
  {
    sceneNode =
        this->dataPtr->manager->getRootSceneNode()->createChildSceneNode(name_);
    obj = this->dataPtr->manager->createManualObject(name_);
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

  sdf::ElementPtr elem = this->dataPtr->sdf->GetElement("fog");

  elem->GetElement("type")->Set(_type);
  elem->GetElement("color")->Set(_color);
  elem->GetElement("density")->Set(_density);
  elem->GetElement("start")->Set(_start);
  elem->GetElement("end")->Set(_end);

  if (this->dataPtr->manager)
    this->dataPtr->manager->setFog(fogType, Conversions::Convert(_color),
                           _density, _start, _end);
}

//////////////////////////////////////////////////
void Scene::SetVisible(const std::string &name_, bool visible_)
{
  if (this->dataPtr->manager->hasSceneNode(name_))
    this->dataPtr->manager->getSceneNode(name_)->setVisible(visible_);

  if (this->dataPtr->manager->hasManualObject(name_))
    this->dataPtr->manager->getManualObject(name_)->setVisible(visible_);
}

//////////////////////////////////////////////////
uint32_t Scene::GetId() const
{
  return this->dataPtr->id;
}

//////////////////////////////////////////////////
std::string Scene::GetIdString() const
{
  return this->dataPtr->idString;
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
    boost::recursive_mutex::scoped_lock lock(this->dataPtr->poseMsgMutex);
    for (int i = 0; i < _msg->model_size(); ++i)
    {
      PoseMsgs_M::iterator iter =
          this->dataPtr->poseMsgs.find(_msg->model(i).id());
      if (iter != this->dataPtr->poseMsgs.end())
        iter->second.CopyFrom(_msg->model(i).pose());
      else
        this->dataPtr->poseMsgs.insert(
            std::make_pair(_msg->model(i).id(), _msg->model(i).pose()));

      this->dataPtr->poseMsgs[_msg->model(i).id()].set_name(
          _msg->model(i).name());
      this->dataPtr->poseMsgs[_msg->model(i).id()].set_id(_msg->model(i).id());

      this->ProcessModelMsg(_msg->model(i));
    }
  }

  for (int i = 0; i < _msg->light_size(); ++i)
  {
    boost::shared_ptr<msgs::Light> lm(new msgs::Light(_msg->light(i)));
    this->dataPtr->lightMsgs.push_back(lm);
  }

  for (int i = 0; i < _msg->joint_size(); ++i)
  {
    boost::shared_ptr<msgs::Joint> jm(new msgs::Joint(_msg->joint(i)));
    this->dataPtr->jointMsgs.push_back(jm);
  }

  if (_msg->has_ambient())
    this->SetAmbientColor(msgs::Convert(_msg->ambient()));

  if (_msg->has_background())
    this->SetBackgroundColor(msgs::Convert(_msg->background()));

  if (_msg->has_shadows())
    this->SetShadowsEnabled(_msg->shadows());

  if (_msg->has_grid())
    this->SetGrid(_msg->grid());

  if (_msg->has_origin_visual())
    this->ShowOrigin(_msg->origin_visual());

  // Process the sky message.
  if (_msg->has_sky())
  {
    boost::shared_ptr<msgs::Sky> sm(new msgs::Sky(_msg->sky()));
    this->OnSkyMsg(sm);
  }

  if (_msg->has_fog())
  {
    sdf::ElementPtr elem = this->dataPtr->sdf->GetElement("fog");

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
  for (int j = 0; j < _msg.visual_size(); ++j)
  {
    boost::shared_ptr<msgs::Visual> vm(new msgs::Visual(
          _msg.visual(j)));
    this->dataPtr->modelVisualMsgs.push_back(vm);
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
    this->dataPtr->modelVisualMsgs.push_back(vm);
  }

  for (int j = 0; j < _msg.joint_size(); ++j)
  {
    boost::shared_ptr<msgs::Joint> jm(new msgs::Joint(
          _msg.joint(j)));
    this->dataPtr->jointMsgs.push_back(jm);

    for (int k = 0; k < _msg.joint(j).sensor_size(); ++k)
    {
      boost::shared_ptr<msgs::Sensor> sm(new msgs::Sensor(
            _msg.joint(j).sensor(k)));
      this->dataPtr->sensorMsgs.push_back(sm);
    }
  }

  for (int j = 0; j < _msg.link_size(); ++j)
  {
    linkName = modelName + _msg.link(j).name();

    {
      boost::recursive_mutex::scoped_lock lock(this->dataPtr->poseMsgMutex);
      if (_msg.link(j).has_pose())
      {
        PoseMsgs_M::iterator iter =
            this->dataPtr->poseMsgs.find(_msg.link(j).id());
        if (iter != this->dataPtr->poseMsgs.end())
          iter->second.CopyFrom(_msg.link(j).pose());
        else
          this->dataPtr->poseMsgs.insert(
              std::make_pair(_msg.link(j).id(), _msg.link(j).pose()));

        this->dataPtr->poseMsgs[_msg.link(j).id()].set_name(linkName);
        this->dataPtr->poseMsgs[_msg.link(j).id()].set_id(_msg.link(j).id());
      }
    }

    if (_msg.link(j).has_inertial())
    {
      boost::shared_ptr<msgs::Link> lm(new msgs::Link(_msg.link(j)));
      this->dataPtr->linkMsgs.push_back(lm);
    }

    if (_msg.link(j).visual_size() > 0)
    {
      // note: the first visual in the link is the link visual
      msgs::VisualPtr vm(new msgs::Visual(
            _msg.link(j).visual(0)));
      this->dataPtr->linkVisualMsgs.push_back(vm);
    }

    for (int k = 1; k < _msg.link(j).visual_size(); ++k)
    {
      boost::shared_ptr<msgs::Visual> vm(new msgs::Visual(
            _msg.link(j).visual(k)));
      this->dataPtr->visualMsgs.push_back(vm);
    }

    for (int k = 0; k < _msg.link(j).collision_size(); ++k)
    {
      for (int l = 0;
          l < _msg.link(j).collision(k).visual_size(); l++)
      {
        boost::shared_ptr<msgs::Visual> vm(new msgs::Visual(
              _msg.link(j).collision(k).visual(l)));
        this->dataPtr->collisionVisualMsgs.push_back(vm);
      }
    }

    for (int k = 0; k < _msg.link(j).sensor_size(); ++k)
    {
      boost::shared_ptr<msgs::Sensor> sm(new msgs::Sensor(
            _msg.link(j).sensor(k)));
      this->dataPtr->sensorMsgs.push_back(sm);
    }
  }

  for (int i = 0; i < _msg.model_size(); ++i)
  {
    boost::shared_ptr<msgs::Model> mm(new msgs::Model(_msg.model(i)));
    this->dataPtr->modelMsgs.push_back(mm);
  }

  return true;
}

//////////////////////////////////////////////////
void Scene::OnSensorMsg(ConstSensorPtr &_msg)
{
  boost::mutex::scoped_lock lock(*this->dataPtr->receiveMutex);
  this->dataPtr->sensorMsgs.push_back(_msg);
}

//////////////////////////////////////////////////
void Scene::OnVisualMsg(ConstVisualPtr &_msg)
{
  boost::mutex::scoped_lock lock(*this->dataPtr->receiveMutex);
  this->dataPtr->visualMsgs.push_back(_msg);
}

//////////////////////////////////////////////////
void Scene::PreRender()
{
  /* Deferred shading debug code. Delete me soon (July 17, 2012)
  static bool first = true;

  if (!first)
  {
    Ogre::RenderSystem *renderSys =
        this->dataPtr->manager->getDestinationRenderSystem();
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

        Ogre::MultiRenderTarget *mtarget =
            dynamic_cast<Ogre::MultiRenderTarget *>(
            renderIter.current()->second);
        if (mtarget)
        {
          // std::cout << renderIter.current()->first << "\n";
          mtarget->getBoundSurface(0)->writeContentsToFile(filename.str());

          mtarget->getBoundSurface(1)->writeContentsToFile(filename2.str());
          ++i;
        }
        else
        {
          renderIter.current()->second->writeContentsToFile(filename.str());
          ++i;
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
  VisualMsgs_L modelVisualMsgsCopy;
  VisualMsgs_L linkVisualMsgsCopy;
  VisualMsgs_L visualMsgsCopy;
  VisualMsgs_L collisionVisualMsgsCopy;
  JointMsgs_L jointMsgsCopy;
  LinkMsgs_L linkMsgsCopy;

  {
    boost::mutex::scoped_lock lock(*this->dataPtr->receiveMutex);

    std::copy(this->dataPtr->sceneMsgs.begin(), this->dataPtr->sceneMsgs.end(),
              std::back_inserter(sceneMsgsCopy));
    this->dataPtr->sceneMsgs.clear();

    std::copy(this->dataPtr->modelMsgs.begin(), this->dataPtr->modelMsgs.end(),
              std::back_inserter(modelMsgsCopy));
    this->dataPtr->modelMsgs.clear();

    std::copy(this->dataPtr->sensorMsgs.begin(),
              this->dataPtr->sensorMsgs.end(),
              std::back_inserter(sensorMsgsCopy));
    this->dataPtr->sensorMsgs.clear();

    std::copy(this->dataPtr->lightMsgs.begin(), this->dataPtr->lightMsgs.end(),
              std::back_inserter(lightMsgsCopy));
    this->dataPtr->lightMsgs.clear();

    std::copy(this->dataPtr->modelVisualMsgs.begin(),
              this->dataPtr->modelVisualMsgs.end(),
              std::back_inserter(modelVisualMsgsCopy));
    this->dataPtr->modelVisualMsgs.clear();

    std::copy(this->dataPtr->linkVisualMsgs.begin(),
              this->dataPtr->linkVisualMsgs.end(),
              std::back_inserter(linkVisualMsgsCopy));
    this->dataPtr->linkVisualMsgs.clear();

    this->dataPtr->visualMsgs.sort(VisualMessageLessOp);
    std::copy(this->dataPtr->visualMsgs.begin(),
              this->dataPtr->visualMsgs.end(),
              std::back_inserter(visualMsgsCopy));
    this->dataPtr->visualMsgs.clear();

    std::copy(this->dataPtr->collisionVisualMsgs.begin(),
              this->dataPtr->collisionVisualMsgs.end(),
              std::back_inserter(collisionVisualMsgsCopy));
    this->dataPtr->collisionVisualMsgs.clear();

    std::copy(this->dataPtr->jointMsgs.begin(), this->dataPtr->jointMsgs.end(),
              std::back_inserter(jointMsgsCopy));
    this->dataPtr->jointMsgs.clear();

    std::copy(this->dataPtr->linkMsgs.begin(), this->dataPtr->linkMsgs.end(),
              std::back_inserter(linkMsgsCopy));
    this->dataPtr->linkMsgs.clear();
  }

  // Process the scene messages. DO THIS FIRST
  for (sIter = sceneMsgsCopy.begin(); sIter != sceneMsgsCopy.end();)
  {
    if (this->ProcessSceneMsg(*sIter))
    {
      if (!this->dataPtr->initialized)
        RTShaderSystem::Instance()->UpdateShaders();
      this->dataPtr->initialized = true;
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

  // Process the model visual messages.
  for (visualIter = modelVisualMsgsCopy.begin();
      visualIter != modelVisualMsgsCopy.end();)
  {
    if (this->ProcessVisualMsg(*visualIter, Visual::VT_MODEL))
      modelVisualMsgsCopy.erase(visualIter++);
    else
      ++visualIter;
  }

  // Process the link visual messages.
  for (visualIter = linkVisualMsgsCopy.begin();
      visualIter != linkVisualMsgsCopy.end();)
  {
    if (this->ProcessVisualMsg(*visualIter, Visual::VT_LINK))
      linkVisualMsgsCopy.erase(visualIter++);
    else
      ++visualIter;
  }

  // Process the visual messages.
  for (visualIter = visualMsgsCopy.begin(); visualIter != visualMsgsCopy.end();)
  {
    Visual::VisualType visualType = Visual::VT_VISUAL;
    if ((*visualIter)->has_type())
      visualType = Visual::ConvertVisualType((*visualIter)->type());

    if (this->ProcessVisualMsg(*visualIter, visualType))
      visualMsgsCopy.erase(visualIter++);
    else
      ++visualIter;
  }

  // Process the collision visual messages.
  for (visualIter = collisionVisualMsgsCopy.begin();
      visualIter != collisionVisualMsgsCopy.end();)
  {
    if (this->ProcessVisualMsg(*visualIter, Visual::VT_COLLISION))
      collisionVisualMsgsCopy.erase(visualIter++);
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
  for (rIter =  this->dataPtr->requestMsgs.begin();
      rIter != this->dataPtr->requestMsgs.end(); ++rIter)
  {
    this->ProcessRequestMsg(*rIter);
  }
  this->dataPtr->requestMsgs.clear();

  {
    boost::mutex::scoped_lock lock(*this->dataPtr->receiveMutex);

    std::copy(sceneMsgsCopy.begin(), sceneMsgsCopy.end(),
        std::front_inserter(this->dataPtr->sceneMsgs));

    std::copy(modelMsgsCopy.begin(), modelMsgsCopy.end(),
        std::front_inserter(this->dataPtr->modelMsgs));

    std::copy(sensorMsgsCopy.begin(), sensorMsgsCopy.end(),
        std::front_inserter(this->dataPtr->sensorMsgs));

    std::copy(lightMsgsCopy.begin(), lightMsgsCopy.end(),
        std::front_inserter(this->dataPtr->lightMsgs));

    std::copy(modelVisualMsgsCopy.begin(), modelVisualMsgsCopy.end(),
        std::front_inserter(this->dataPtr->modelVisualMsgs));

    std::copy(linkVisualMsgsCopy.begin(), linkVisualMsgsCopy.end(),
        std::front_inserter(this->dataPtr->linkVisualMsgs));

    std::copy(visualMsgsCopy.begin(), visualMsgsCopy.end(),
        std::front_inserter(this->dataPtr->visualMsgs));

    std::copy(collisionVisualMsgsCopy.begin(), collisionVisualMsgsCopy.end(),
        std::front_inserter(this->dataPtr->collisionVisualMsgs));

    std::copy(jointMsgsCopy.begin(), jointMsgsCopy.end(),
        std::front_inserter(this->dataPtr->jointMsgs));

    std::copy(linkMsgsCopy.begin(), linkMsgsCopy.end(),
        std::front_inserter(this->dataPtr->linkMsgs));
  }

  {
    boost::recursive_mutex::scoped_lock lock(this->dataPtr->poseMsgMutex);

    // Process all the model messages last. Remove pose message from the list
    // only when a corresponding visual exits. We may receive pose updates
    // over the wire before  we recieve the visual
    pIter = this->dataPtr->poseMsgs.begin();
    while (pIter != this->dataPtr->poseMsgs.end())
    {
      Visual_M::iterator iter = this->dataPtr->visuals.find(pIter->first);
      if (iter != this->dataPtr->visuals.end() && iter->second)
      {
        // If an object is selected, don't let the physics engine move it.
        if (!this->dataPtr->selectedVis
            || this->dataPtr->selectionMode != "move" ||
            (iter->first != this->dataPtr->selectedVis->GetId() &&
            !this->dataPtr->selectedVis->IsAncestorOf(iter->second)))
        {
          ignition::math::Pose3d pose = msgs::ConvertIgn(pIter->second);
          GZ_ASSERT(iter->second, "Visual pointer is NULL");
          iter->second->SetPose(pose);
          PoseMsgs_M::iterator prev = pIter++;
          this->dataPtr->poseMsgs.erase(prev);
        }
        else
          ++pIter;
      }
      else
        ++pIter;
    }

    // process skeleton pose msgs
    spIter = this->dataPtr->skeletonPoseMsgs.begin();
    while (spIter != this->dataPtr->skeletonPoseMsgs.end())
    {
      Visual_M::iterator iter =
          this->dataPtr->visuals.find((*spIter)->model_id());
      for (int i = 0; i < (*spIter)->pose_size(); ++i)
      {
        const msgs::Pose& pose_msg = (*spIter)->pose(i);
        if (pose_msg.has_id())
        {
          Visual_M::iterator iter2 = this->dataPtr->visuals.find(pose_msg.id());
          if (iter2 != this->dataPtr->visuals.end())
          {
            // If an object is selected, don't let the physics engine move it.
            if (!this->dataPtr->selectedVis ||
                this->dataPtr->selectionMode != "move" ||
                (iter->first != this->dataPtr->selectedVis->GetId()&&
                !this->dataPtr->selectedVis->IsAncestorOf(iter->second)))
            {
              ignition::math::Pose3d pose = msgs::ConvertIgn(pose_msg);
              iter2->second->SetPose(pose);
            }
          }
        }
      }

      if (iter != this->dataPtr->visuals.end())
      {
        iter->second->SetSkeletonPose(*(*spIter).get());
        SkeletonPoseMsgs_L::iterator prev = spIter++;
        this->dataPtr->skeletonPoseMsgs.erase(prev);
      }
      else
        ++spIter;
    }

    // official time stamp of approval
    this->dataPtr->sceneSimTimePosesApplied =
        this->dataPtr->sceneSimTimePosesReceived;
  }
}

/////////////////////////////////////////////////
void Scene::OnJointMsg(ConstJointPtr &_msg)
{
  boost::mutex::scoped_lock lock(*this->dataPtr->receiveMutex);
  this->dataPtr->jointMsgs.push_back(_msg);
}

/////////////////////////////////////////////////
bool Scene::ProcessSensorMsg(ConstSensorPtr &_msg)
{
  if (!this->dataPtr->enableVisualizations)
    return true;

  if ((_msg->type() == "ray" || _msg->type() == "gpu_ray") && _msg->visualize()
      && !_msg->topic().empty())
  {
    std::string rayVisualName = _msg->parent() + "::" + _msg->name();
    if (this->dataPtr->visuals.find(_msg->id()) == this->dataPtr->visuals.end())
    {
      VisualPtr parentVis = this->GetVisual(_msg->parent_id());
      if (!parentVis)
        return false;

      LaserVisualPtr laserVis(new LaserVisual(
            rayVisualName+"_GUIONLY_laser_vis", parentVis, _msg->topic()));
      laserVis->Load();
      laserVis->SetId(_msg->id());
      this->dataPtr->visuals[_msg->id()] = laserVis;
    }
  }
  else if ((_msg->type() == "sonar") && _msg->visualize()
      && !_msg->topic().empty())
  {
    std::string sonarVisualName = _msg->parent() + "::" + _msg->name();
    if (this->dataPtr->visuals.find(_msg->id()) == this->dataPtr->visuals.end())
    {
      VisualPtr parentVis = this->GetVisual(_msg->parent());
      if (!parentVis)
        return false;

      SonarVisualPtr sonarVis(new SonarVisual(
            sonarVisualName+"_GUIONLY_sonar_vis", parentVis, _msg->topic()));
      sonarVis->Load();
      sonarVis->SetId(_msg->id());
      this->dataPtr->visuals[_msg->id()] = sonarVis;
    }
  }
  else if ((_msg->type() == "force_torque") && _msg->visualize()
      && !_msg->topic().empty())
  {
    std::string wrenchVisualName = _msg->parent() + "::" + _msg->name();
    if (this->dataPtr->visuals.find(_msg->id()) == this->dataPtr->visuals.end())
    {
      ConstJointPtr jointMsg = this->dataPtr->joints[_msg->parent()];

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
      this->dataPtr->visuals[_msg->id()] = wrenchVis;
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
      Visual_M::iterator iter = this->dataPtr->visuals.find(_msg->id());
      if (iter == this->dataPtr->visuals.end())
      {
        CameraVisualPtr cameraVis(new CameraVisual(
              _msg->name()+"_GUIONLY_camera_vis", parentVis));

        // need to call AttachVisual in order for cameraVis to be added to
        // parentVis' children list so that it can be properly deleted.
        parentVis->AttachVisual(cameraVis);

        cameraVis->SetPose(msgs::ConvertIgn(_msg->pose()));
        cameraVis->SetId(_msg->id());
        cameraVis->Load(_msg->camera());
        this->dataPtr->visuals[cameraVis->GetId()] = cameraVis;
      }
    }
  }
  else if (_msg->type() == "logical_camera" && _msg->visualize())
  {
    VisualPtr parentVis = this->GetVisual(_msg->parent_id());
    if (!parentVis)
      return false;

    Visual_M::iterator iter = this->dataPtr->visuals.find(_msg->id());
    if (iter == this->dataPtr->visuals.end())
    {
      LogicalCameraVisualPtr cameraVis(new LogicalCameraVisual(
            _msg->name()+"_GUIONLY_logical_camera_vis", parentVis));

      // need to call AttachVisual in order for cameraVis to be added to
      // parentVis' children list so that it can be properly deleted.
      parentVis->AttachVisual(cameraVis);

      cameraVis->SetPose(msgs::ConvertIgn(_msg->pose()));
      cameraVis->SetId(_msg->id());
      cameraVis->Load(_msg->logical_camera());
      this->dataPtr->visuals[cameraVis->GetId()] = cameraVis;
    }
  }
  else if (_msg->type() == "contact" && _msg->visualize() &&
           !_msg->topic().empty())
  {
    ContactVisualPtr contactVis(new ContactVisual(
          _msg->name()+"__GUIONLY_CONTACT_VISUAL__",
          this->dataPtr->worldVisual, _msg->topic()));
    contactVis->SetId(_msg->id());

    this->dataPtr->contactVisId = _msg->id();
    this->dataPtr->visuals[contactVis->GetId()] = contactVis;
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

    this->dataPtr->visuals[rfidVis->GetId()] = rfidVis;
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
    this->dataPtr->visuals[rfidVis->GetId()] = rfidVis;
  }
  else if (_msg->type() == "wireless_transmitter" && _msg->visualize() &&
           !_msg->topic().empty())
  {
    VisualPtr parentVis = this->GetVisual(_msg->parent());
    if (!parentVis)
      return false;

    VisualPtr transmitterVis(new TransmitterVisual(
          _msg->name() + "_GUIONLY_transmitter_vis", parentVis, _msg->topic()));
    this->dataPtr->visuals[transmitterVis->GetId()] = transmitterVis;
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

  if (!this->GetVisual(_msg->name() + "_INERTIA_VISUAL__"))
  {
    this->CreateInertiaVisual(_msg, linkVis);
  }

  if (!this->GetVisual(_msg->name() + "_LINK_FRAME_VISUAL__"))
  {
    this->CreateLinkFrameVisual(_msg, linkVis);
  }

  for (int i = 0; i < _msg->projector_size(); ++i)
  {
    std::string pname = _msg->name() + "::" + _msg->projector(i).name();

    if (this->dataPtr->projectors.find(pname) ==
        this->dataPtr->projectors.end())
    {
      Projector *projector = new Projector(linkVis);
      projector->Load(_msg->projector(i));
      projector->Toggle();
      this->dataPtr->projectors[pname] = projector;
    }
  }

  return true;
}

/////////////////////////////////////////////////
bool Scene::ProcessJointMsg(ConstJointPtr &_msg)
{
  VisualPtr childVis;

  if (_msg->has_child() && _msg->child() == "world")
    childVis = this->dataPtr->worldVisual;
  else if (_msg->has_child_id())
    childVis = this->GetVisual(_msg->child_id());

  if (!childVis)
    return false;

  JointVisualPtr jointVis(new JointVisual(
      _msg->name() + "_JOINT_VISUAL__", childVis));
  jointVis->Load(_msg);
  jointVis->SetVisible(this->dataPtr->showJoints);
  if (_msg->has_id())
    jointVis->SetId(_msg->id());

  this->dataPtr->visuals[jointVis->GetId()] = jointVis;

  return true;
}

/////////////////////////////////////////////////
void Scene::OnScene(ConstScenePtr &_msg)
{
  boost::mutex::scoped_lock lock(*this->dataPtr->receiveMutex);
  this->dataPtr->sceneMsgs.push_back(_msg);
}

/////////////////////////////////////////////////
void Scene::OnResponse(ConstResponsePtr &_msg)
{
  if (!this->dataPtr->requestMsg ||
      _msg->id() != this->dataPtr->requestMsg->id())
    return;

  msgs::Scene sceneMsg;
  sceneMsg.ParseFromString(_msg->serialized_data());
  boost::shared_ptr<msgs::Scene> sm(new msgs::Scene(sceneMsg));

  boost::mutex::scoped_lock lock(*this->dataPtr->receiveMutex);
  this->dataPtr->sceneMsgs.push_back(sm);
  this->dataPtr->requestMsg = NULL;
}

/////////////////////////////////////////////////
void Scene::OnRequest(ConstRequestPtr &_msg)
{
  boost::mutex::scoped_lock lock(*this->dataPtr->receiveMutex);
  this->dataPtr->requestMsgs.push_back(_msg);
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
    iter = this->dataPtr->lights.find(_msg->data());
    if (iter != this->dataPtr->lights.end())
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
    Light_M::iterator lightIter = this->dataPtr->lights.find(_msg->data());

    // Check to see if the deleted entity is a light.
    if (lightIter != this->dataPtr->lights.end())
    {
      this->dataPtr->lights.erase(lightIter);
    }
    // Otherwise delete a visual
    else
    {
      VisualPtr visPtr;
      try
      {
        Visual_M::iterator iter;
        iter = this->dataPtr->visuals.find(
            boost::lexical_cast<uint32_t>(_msg->data()));
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
        gzerr << "Unable to find COM visual[" << _msg->data() << "]\n";
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
  else if (_msg->request() == "show_inertia")
  {
    if (_msg->data() == "all")
      this->ShowInertias(true);
    else
    {
      VisualPtr vis = this->GetVisual(_msg->data());
      if (vis)
        vis->ShowInertia(true);
      else
        gzerr << "Unable to find inertia visual[" << _msg->data() << "]\n";
    }
  }
  else if (_msg->request() == "hide_inertia")
  {
    if (_msg->data() == "all")
      this->ShowInertias(false);
    else
    {
      VisualPtr vis = this->GetVisual(_msg->data());
      if (vis)
        vis->ShowInertia(false);
    }
  }
  else if (_msg->request() == "show_link_frame")
  {
    if (_msg->data() == "all")
      this->ShowLinkFrames(true);
    else
    {
      VisualPtr vis = this->GetVisual(_msg->data());
      if (vis)
        vis->ShowLinkFrame(true);
      else
        gzerr << "Unable to find link frame visual[" << _msg->data() << "]\n";
    }
  }
  else if (_msg->request() == "hide_link_frame")
  {
    if (_msg->data() == "all")
      this->ShowLinkFrames(false);
    else
    {
      VisualPtr vis = this->GetVisual(_msg->data());
      if (vis)
        vis->ShowLinkFrame(false);
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
bool Scene::ProcessVisualMsg(ConstVisualPtr &_msg, Visual::VisualType _type)
{
  bool result = false;
  Visual_M::iterator iter = this->dataPtr->visuals.end();

  if (_msg->has_id())
    iter = this->dataPtr->visuals.find(_msg->id());
  else
  {
    VisualPtr vis = this->GetVisual(_msg->name());
    iter = vis ? this->dataPtr->visuals.find(vis->GetId()) :
        this->dataPtr->visuals.end();
  }

  if (_msg->has_delete_me() && _msg->delete_me())
  {
    if (iter != this->dataPtr->visuals.end())
    {
      this->dataPtr->visuals.erase(iter);
      result = true;
    }
  }
  else if (iter != this->dataPtr->visuals.end())
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
          this->dataPtr->terrain == NULL)
      {
        try
        {
          if (!this->dataPtr->terrain)
          {
            this->dataPtr->terrain = new Heightmap(shared_from_this());
            this->dataPtr->terrain->LoadFromMsg(_msg);
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
        iter = this->dataPtr->visuals.find(_msg->id());
      else
      {
        VisualPtr vis = this->GetVisual(_msg->name());
        iter = vis ? this->dataPtr->visuals.find(vis->GetId()) :
            this->dataPtr->visuals.end();
      }

      if (iter != this->dataPtr->visuals.end())
        gzerr << "Visual already exists. This shouldn't happen.\n";

      // Make sure the parent visual exists before trying to add a child
      // visual
      iter = this->dataPtr->visuals.find(_msg->parent_id());
      if (iter != this->dataPtr->visuals.end())
      {
        visual.reset(new Visual(_msg->name(), iter->second));
        if (_msg->has_id())
          visual->SetId(_msg->id());
      }
    }
    else
    {
      // Add a visual that is attached to the scene root
      visual.reset(new Visual(_msg->name(), this->dataPtr->worldVisual));
      if (_msg->has_id())
        visual->SetId(_msg->id());
    }

    if (visual)
    {
      result = true;
      visual->LoadFromMsg(_msg);
      visual->SetType(_type);

      this->dataPtr->visuals[visual->GetId()] = visual;
      if (visual->GetName().find("__COLLISION_VISUAL__") != std::string::npos ||
          visual->GetName().find("__SKELETON_VISUAL__") != std::string::npos)
      {
        visual->SetVisible(false);
        visual->SetVisibilityFlags(GZ_VISIBILITY_GUI);
      }

      visual->ShowCOM(this->dataPtr->showCOMs);
      visual->ShowInertia(this->dataPtr->showInertias);
      visual->ShowLinkFrame(this->dataPtr->showLinkFrames);
      visual->ShowCollision(this->dataPtr->showCollisions);
      visual->ShowJoints(this->dataPtr->showJoints);
      visual->SetTransparency(this->dataPtr->transparent ? 0.5 : 0.0);
      visual->SetWireframe(this->dataPtr->wireframe);
    }
  }

  return result;
}

/////////////////////////////////////////////////
common::Time Scene::GetSimTime() const
{
  boost::mutex::scoped_lock lock(*this->dataPtr->receiveMutex);
  return this->dataPtr->sceneSimTimePosesApplied;
}

/////////////////////////////////////////////////
void Scene::OnPoseMsg(ConstPosesStampedPtr &_msg)
{
  boost::recursive_mutex::scoped_lock lock(this->dataPtr->poseMsgMutex);
  this->dataPtr->sceneSimTimePosesReceived =
    common::Time(_msg->time().sec(), _msg->time().nsec());

  for (int i = 0; i < _msg->pose_size(); ++i)
  {
    PoseMsgs_M::iterator iter =
        this->dataPtr->poseMsgs.find(_msg->pose(i).id());
    if (iter != this->dataPtr->poseMsgs.end())
      iter->second.CopyFrom(_msg->pose(i));
    else
      this->dataPtr->poseMsgs.insert(
          std::make_pair(_msg->pose(i).id(), _msg->pose(i)));
  }
}

/////////////////////////////////////////////////
void Scene::OnSkeletonPoseMsg(ConstPoseAnimationPtr &_msg)
{
  boost::mutex::scoped_lock lock(*this->dataPtr->receiveMutex);
  SkeletonPoseMsgs_L::iterator iter;

  // Find an old model message, and remove them
  for (iter = this->dataPtr->skeletonPoseMsgs.begin();
        iter != this->dataPtr->skeletonPoseMsgs.end(); ++iter)
  {
    if ((*iter)->model_name() == _msg->model_name())
    {
      this->dataPtr->skeletonPoseMsgs.erase(iter);
      break;
    }
  }

  this->dataPtr->skeletonPoseMsgs.push_back(_msg);
}


/////////////////////////////////////////////////
void Scene::OnLightMsg(ConstLightPtr &_msg)
{
  boost::mutex::scoped_lock lock(*this->dataPtr->receiveMutex);
  this->dataPtr->lightMsgs.push_back(_msg);
}

/////////////////////////////////////////////////
bool Scene::ProcessLightMsg(ConstLightPtr &_msg)
{
  Light_M::iterator iter;
  iter = this->dataPtr->lights.find(_msg->name());

  if (iter == this->dataPtr->lights.end())
  {
    LightPtr light(new Light(shared_from_this()));
    light->LoadFromMsg(_msg);
    this->dataPtr->lights[_msg->name()] = light;
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
void Scene::OnModelMsg(ConstModelPtr &_msg)
{
  boost::mutex::scoped_lock lock(*this->dataPtr->receiveMutex);
  this->dataPtr->modelMsgs.push_back(_msg);
}

/////////////////////////////////////////////////
void Scene::OnSkyMsg(ConstSkyPtr &_msg)
{
  if (!this->dataPtr->skyx)
    return;

  Ogre::Root::getSingletonPtr()->addFrameListener(this->dataPtr->skyx);
  this->dataPtr->skyx->update(0);

  this->dataPtr->skyx->setVisible(true);

  SkyX::VClouds::VClouds *vclouds =
    this->dataPtr->skyx->getVCloudsManager()->getVClouds();

  if (_msg->has_time())
  {
    Ogre::Vector3 t = this->dataPtr->skyxController->getTime();
    t.x = math::clamp(_msg->time(), 0.0, 24.0);
    this->dataPtr->skyxController->setTime(t);
  }

  if (_msg->has_sunrise())
  {
    Ogre::Vector3 t = this->dataPtr->skyxController->getTime();
    t.y = math::clamp(_msg->sunrise(), 0.0, 24.0);
    this->dataPtr->skyxController->setTime(t);
  }

  if (_msg->has_sunset())
  {
    Ogre::Vector3 t = this->dataPtr->skyxController->getTime();
    t.z = math::clamp(_msg->sunset(), 0.0, 24.0);
    this->dataPtr->skyxController->setTime(t);
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

  this->dataPtr->skyx->update(0);
}

/////////////////////////////////////////////////
void Scene::SetSky()
{
  // Create SkyX
  this->dataPtr->skyxController = new SkyX::BasicController();
  this->dataPtr->skyx = new SkyX::SkyX(this->dataPtr->manager,
      this->dataPtr->skyxController);
  this->dataPtr->skyx->create();

  this->dataPtr->skyx->setTimeMultiplier(0);

  // Set the time: x = current time[0-24], y = sunrise time[0-24],
  // z = sunset time[0-24]
  this->dataPtr->skyxController->setTime(Ogre::Vector3(10.0, 6.0, 20.0f));

  // Moon phase in [-1,1] range, where -1 means fully covered Moon,
  // 0 clear Moon and 1 fully covered Moon
  this->dataPtr->skyxController->setMoonPhase(0);

  this->dataPtr->skyx->getAtmosphereManager()->setOptions(
      SkyX::AtmosphereManager::Options(
        9.77501f,   // Inner radius
        10.2963f,   // Outer radius
        0.01f,      // Height position
        0.0017f,    // RayleighMultiplier
        0.000675f,  // MieMultiplier
        30,         // Sun Intensity
        Ogre::Vector3(0.57f, 0.54f, 0.44f),  // Wavelength
        -0.991f, 2.5f, 4));

  this->dataPtr->skyx->getVCloudsManager()->setWindSpeed(0.6);

  // Use true to update volumetric clouds based on the time multiplier
  this->dataPtr->skyx->getVCloudsManager()->setAutoupdate(false);

  SkyX::VClouds::VClouds *vclouds =
    this->dataPtr->skyx->getVCloudsManager()->getVClouds();

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
    if (!this->dataPtr->skyx->getVCloudsManager()->isCreated())
    {
      // SkyX::MeshManager::getSkydomeRadius(...) works for both finite and
      // infinite(=0) camera far clip distances
      this->dataPtr->skyx->getVCloudsManager()->create(2000.0);
      // this->dataPtr->skyx->getMeshManager()->getSkydomeRadius(
      //    mRenderingCamera));
    }
  }
  else
  {
    // Remove VClouds
    if (this->dataPtr->skyx->getVCloudsManager()->isCreated())
    {
      this->dataPtr->skyx->getVCloudsManager()->remove();
    }
  }

  // vclouds->getLightningManager()->setEnabled(preset.vcLightnings);
  // vclouds->getLightningManager()->setAverageLightningApparitionTime(
  //     preset.vcLightningsAT);
  // vclouds->getLightningManager()->setLightningColor(
  //     preset.vcLightningsColor);
  // vclouds->getLightningManager()->setLightningTimeMultiplier(
  //    preset.vcLightningsTM);

  this->dataPtr->skyx->setVisible(false);
}

/////////////////////////////////////////////////
void Scene::SetShadowsEnabled(bool _value)
{
  // If a usercamera is set to stereo mode, then turn off shadows.
  // If a usercamera uses orthographic projection, then turn off shadows.
  // Our shadow mapping technique disables stereo.
  bool shadowOverride = true;
  for (std::vector<UserCameraPtr>::iterator iter =
       this->dataPtr->userCameras.begin();
       iter != this->dataPtr->userCameras.end() && shadowOverride; ++iter)
  {
    shadowOverride = !(*iter)->StereoEnabled() &&
                     (*iter)->GetProjectionType() != "orthographic";
  }

  _value = _value && shadowOverride;

  this->dataPtr->sdf->GetElement("shadows")->Set(_value);

  if (RenderEngine::Instance()->GetRenderPathType() == RenderEngine::DEFERRED)
  {
#if OGRE_VERSION_MAJOR >= 1 && OGRE_VERSION_MINOR >= 8
    this->dataPtr->manager->setShadowTechnique(
        Ogre::SHADOWTYPE_TEXTURE_ADDITIVE);
    this->dataPtr->manager->setShadowTextureCasterMaterial(
        "DeferredRendering/Shadows/RSMCaster_Spot");
    this->dataPtr->manager->setShadowTextureCount(1);
    this->dataPtr->manager->setShadowFarDistance(150);
    // Use a value of "2" to use a different depth buffer pool and
    // avoid sharing this with the Backbuffer's
    this->dataPtr->manager->setShadowTextureConfig(0, 1024, 1024,
        Ogre::PF_FLOAT32_RGBA, 0, 2);
    this->dataPtr->manager->setShadowDirectionalLightExtrusionDistance(75);
    this->dataPtr->manager->setShadowCasterRenderBackFaces(false);
    this->dataPtr->manager->setShadowTextureSelfShadow(true);
    this->dataPtr->manager->setShadowDirLightTextureOffset(1.75);
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
    this->dataPtr->manager->setShadowCasterRenderBackFaces(false);
    this->dataPtr->manager->setShadowTextureSize(512);

    // The default shadows.
    if (_value && this->dataPtr->manager->getShadowTechnique()
        != Ogre::SHADOWTYPE_TEXTURE_ADDITIVE)
    {
      this->dataPtr->manager->setShadowTechnique(
          Ogre::SHADOWTYPE_TEXTURE_ADDITIVE);
    }
    else
      this->dataPtr->manager->setShadowTechnique(Ogre::SHADOWTYPE_NONE);
  }
}

/////////////////////////////////////////////////
bool Scene::GetShadowsEnabled() const
{
  return this->dataPtr->sdf->Get<bool>("shadows");
}

/////////////////////////////////////////////////
void Scene::AddVisual(VisualPtr _vis)
{
  if (this->dataPtr->visuals.find(_vis->GetId()) !=
      this->dataPtr->visuals.end())
  {
    gzwarn << "Duplicate visuals detected[" << _vis->GetName() << "]\n";
  }

  this->dataPtr->visuals[_vis->GetId()] = _vis;
}

/////////////////////////////////////////////////
void Scene::RemoveVisual(uint32_t _id)
{
  // Delete the visual
  auto iter = this->dataPtr->visuals.find(_id);
  if (iter != this->dataPtr->visuals.end())
  {
    VisualPtr vis = iter->second;
    // Remove all projectors attached to the visual
    auto piter = this->dataPtr->projectors.begin();
    while (piter != this->dataPtr->projectors.end())
    {
      // Check to see if the projector is a child of the visual that is
      // being removed.
      if (piter->second->GetParent()->GetRootVisual()->GetName() ==
          vis->GetRootVisual()->GetName())
      {
        delete piter->second;
        this->dataPtr->projectors.erase(piter++);
      }
      else
        ++piter;
    }

    this->RemoveVisualizations(vis);

    vis->Fini();
    this->dataPtr->visuals.erase(iter);
    if (this->dataPtr->selectedVis && this->dataPtr->selectedVis->GetId() ==
        vis->GetId())
      this->dataPtr->selectedVis.reset();
  }
}

/////////////////////////////////////////////////
void Scene::RemoveVisual(VisualPtr _vis)
{
  this->RemoveVisual(_vis->GetId());
}

/////////////////////////////////////////////////
void Scene::SetVisualId(VisualPtr _vis, uint32_t _id)
{
  if (!_vis)
    return;

  auto iter = this->dataPtr->visuals.find(_vis->GetId());
  if (iter != this->dataPtr->visuals.end())
  {
    this->dataPtr->visuals.erase(_vis->GetId());
    this->dataPtr->visuals[_id] = _vis;
    _vis->SetId(_id);
  }
}

/////////////////////////////////////////////////
void Scene::AddLight(LightPtr _light)
{
  std::string n = this->StripSceneName(_light->GetName());
  Light_M::iterator iter = this->dataPtr->lights.find(n);
  if (iter != this->dataPtr->lights.end())
    gzerr << "Duplicate lights detected[" << _light->GetName() << "]\n";

  this->dataPtr->lights[n] = _light;
}

/////////////////////////////////////////////////
void Scene::RemoveLight(LightPtr _light)
{
  if (_light)
  {
    // Delete the light
    std::string n = this->StripSceneName(_light->GetName());
    this->dataPtr->lights.erase(n);
  }
}

/////////////////////////////////////////////////
void Scene::SetGrid(bool _enabled)
{
  if (_enabled && this->dataPtr->grids.empty())
  {
    Grid *grid = new Grid(this, 20, 1, 10, common::Color(0.3, 0.3, 0.3, 0.5));
    grid->Init();
    this->dataPtr->grids.push_back(grid);

    grid = new Grid(this, 4, 5, 20, common::Color(0.8, 0.8, 0.8, 0.5));
    grid->Init();
    this->dataPtr->grids.push_back(grid);
  }
  else
  {
    for (uint32_t i = 0; i < this->dataPtr->grids.size(); ++i)
    {
      this->dataPtr->grids[i]->Enable(_enabled);
    }
  }
}

/////////////////////////////////////////////////
void Scene::ShowOrigin(bool _show)
{
  this->dataPtr->originVisual->SetVisible(_show);
}

//////////////////////////////////////////////////
std::string Scene::StripSceneName(const std::string &_name) const
{
  if (_name.find(this->GetName() + "::") != std::string::npos)
    return _name.substr(this->GetName().size() + 2);
  else
    return _name;
}

//////////////////////////////////////////////////
Heightmap *Scene::GetHeightmap() const
{
  boost::mutex::scoped_lock lock(*this->dataPtr->receiveMutex);
  return this->dataPtr->terrain;
}

/////////////////////////////////////////////////
void Scene::CreateCOMVisual(ConstLinkPtr &_msg, VisualPtr _linkVisual)
{
  COMVisualPtr comVis(new COMVisual(_msg->name() + "_COM_VISUAL__",
                                    _linkVisual));
  comVis->Load(_msg);
  comVis->SetVisible(this->dataPtr->showCOMs);
  this->dataPtr->visuals[comVis->GetId()] = comVis;
}

/////////////////////////////////////////////////
void Scene::CreateCOMVisual(sdf::ElementPtr _elem, VisualPtr _linkVisual)
{
  COMVisualPtr comVis(new COMVisual(_linkVisual->GetName() + "_COM_VISUAL__",
                                    _linkVisual));
  comVis->Load(_elem);
  comVis->SetVisible(false);
  this->dataPtr->visuals[comVis->GetId()] = comVis;
}

/////////////////////////////////////////////////
void Scene::CreateInertiaVisual(ConstLinkPtr &_msg, VisualPtr _linkVisual)
{
  InertiaVisualPtr inertiaVis(new InertiaVisual(_msg->name() +
      "_INERTIA_VISUAL__", _linkVisual));
  inertiaVis->Load(_msg);
  inertiaVis->SetVisible(this->dataPtr->showInertias);
  this->dataPtr->visuals[inertiaVis->GetId()] = inertiaVis;
}

/////////////////////////////////////////////////
void Scene::CreateInertiaVisual(sdf::ElementPtr _elem, VisualPtr _linkVisual)
{
  InertiaVisualPtr inertiaVis(new InertiaVisual(_linkVisual->GetName() +
      "_INERTIA_VISUAL__", _linkVisual));
  inertiaVis->Load(_elem);
  inertiaVis->SetVisible(false);
  this->dataPtr->visuals[inertiaVis->GetId()] = inertiaVis;
}

/////////////////////////////////////////////////
void Scene::CreateLinkFrameVisual(ConstLinkPtr &_msg, VisualPtr _linkVisual)
{
  LinkFrameVisualPtr linkFrameVis(new LinkFrameVisual(_msg->name() +
      "_LINK_FRAME_VISUAL__", _linkVisual));
  linkFrameVis->Load();
  linkFrameVis->SetVisible(this->dataPtr->showLinkFrames);
  this->dataPtr->visuals[linkFrameVis->GetId()] = linkFrameVis;
}

/////////////////////////////////////////////////
void Scene::RemoveVisualizations(rendering::VisualPtr _vis)
{
  std::vector<VisualPtr> toRemove;
  for (unsigned int i = 0; i < _vis->GetChildCount(); ++i)
  {
    rendering::VisualPtr childVis = _vis->GetChild(i);
    Visual::VisualType visType = childVis->GetType();
    if (visType == Visual::VT_PHYSICS || visType == Visual::VT_SENSOR)
    {
      toRemove.push_back(childVis);
    }
  }
  for (auto vis : toRemove)
    this->RemoveVisual(vis);
}

/////////////////////////////////////////////////
void Scene::SetWireframe(bool _show)
{
  this->dataPtr->wireframe = _show;
  for (auto visual : this->dataPtr->visuals)
  {
    visual.second->SetWireframe(_show);
  }

  if (this->dataPtr->terrain)
    this->dataPtr->terrain->SetWireframe(_show);
}

/////////////////////////////////////////////////
void Scene::SetTransparent(bool _show)
{
  this->dataPtr->transparent = _show;
  for (auto visual : this->dataPtr->visuals)
  {
    if (visual.second->GetType() != Visual::VT_GUI &&
        visual.second->GetType() != Visual::VT_PHYSICS &&
        visual.second->GetType() != Visual::VT_SENSOR)
    {
      visual.second->SetTransparency(_show ? 0.5 : 0.0);
    }
  }
}

/////////////////////////////////////////////////
void Scene::ShowCOMs(bool _show)
{
  this->dataPtr->showCOMs = _show;
  for (auto visual : this->dataPtr->visuals)
  {
    visual.second->ShowCOM(_show);
  }
}

/////////////////////////////////////////////////
void Scene::ShowInertias(bool _show)
{
  this->dataPtr->showInertias = _show;
  for (auto visual : this->dataPtr->visuals)
  {
    visual.second->ShowInertia(_show);
  }
}

/////////////////////////////////////////////////
void Scene::ShowLinkFrames(bool _show)
{
  this->dataPtr->showLinkFrames = _show;
  for (auto visual : this->dataPtr->visuals)
  {
    visual.second->ShowLinkFrame(_show);
  }
}

/////////////////////////////////////////////////
void Scene::ShowCollisions(bool _show)
{
  this->dataPtr->showCollisions = _show;
  for (auto visual : this->dataPtr->visuals)
  {
    visual.second->ShowCollision(_show);
  }
}

/////////////////////////////////////////////////
void Scene::ShowJoints(bool _show)
{
  this->dataPtr->showJoints = _show;
  for (auto visual : this->dataPtr->visuals)
  {
    visual.second->ShowJoints(_show);
  }
}

/////////////////////////////////////////////////
void Scene::ShowContacts(bool _show)
{
  ContactVisualPtr vis;

  if (this->dataPtr->contactVisId == GZ_UINT32_MAX && _show)
  {
    vis.reset(new ContactVisual("__GUIONLY_CONTACT_VISUAL__",
              this->dataPtr->worldVisual, "~/physics/contacts"));
    vis->SetEnabled(_show);
    this->dataPtr->contactVisId = vis->GetId();
    this->dataPtr->visuals[this->dataPtr->contactVisId] = vis;
  }
  else
    vis = boost::dynamic_pointer_cast<ContactVisual>(
        this->dataPtr->visuals[this->dataPtr->contactVisId]);

  if (vis)
    vis->SetEnabled(_show);
  else
    gzerr << "Unable to get contact visualization. This should never happen.\n";
}

/////////////////////////////////////////////////
void Scene::ShowClouds(bool _show)
{
  if (!this->dataPtr->skyx)
    return;

  SkyX::VCloudsManager *mgr = this->dataPtr->skyx->getVCloudsManager();
  if (mgr)
  {
    SkyX::VClouds::VClouds *vclouds =
        this->dataPtr->skyx->getVCloudsManager()->getVClouds();
    if (vclouds)
      vclouds->setVisible(_show);
  }
}

/////////////////////////////////////////////////
bool Scene::GetShowClouds() const
{
  if (!this->dataPtr->skyx)
    return false;

  SkyX::VCloudsManager *mgr = this->dataPtr->skyx->getVCloudsManager();
  if (mgr)
  {
    SkyX::VClouds::VClouds *vclouds =
        this->dataPtr->skyx->getVCloudsManager()->getVClouds();
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

  if (!this->dataPtr->skyx)
    return;

  bool enabled = _mode != GZ_SKYX_NONE;
  this->dataPtr->skyx->setEnabled(enabled);

  if (!enabled)
    return;

  this->dataPtr->skyx->setCloudsEnabled(_mode & GZ_SKYX_CLOUDS);
  this->dataPtr->skyx->setMoonEnabled(_mode & GZ_SKYX_MOON);
}

/////////////////////////////////////////////////
SkyX::SkyX *Scene::GetSkyX() const
{
  return this->dataPtr->skyx;
}

/////////////////////////////////////////////////
void Scene::RemoveProjectors()
{
  for (std::map<std::string, Projector *>::iterator iter =
      this->dataPtr->projectors.begin();
      iter != this->dataPtr->projectors.end(); ++iter)
  {
    delete iter->second;
  }
  this->dataPtr->projectors.clear();
}

/////////////////////////////////////////////////
void Scene::ToggleLayer(const int32_t _layer)
{
  for (auto visual : this->dataPtr->visuals)
  {
    visual.second->ToggleLayer(_layer);
  }
}
