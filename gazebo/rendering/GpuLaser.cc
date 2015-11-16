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

#include <sstream>

#ifndef _WIN32
  #include <dirent.h>
#else
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
  #include <Winsock2.h>
  #include "gazebo/common/win_dirent.h"
#endif

#include "gazebo/rendering/ogre_gazebo.h"

#include "gazebo/common/Assert.hh"
#include "gazebo/common/Events.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Exception.hh"
#include "gazebo/common/Mesh.hh"
#include "gazebo/common/MeshManager.hh"
#include "gazebo/common/Timer.hh"
#include "gazebo/math/Pose.hh"

#include "gazebo/rendering/Visual.hh"
#include "gazebo/rendering/Conversions.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/GpuLaser.hh"

using namespace gazebo;
using namespace rendering;

int GpuLaser::texCount = 0;

//////////////////////////////////////////////////
GpuLaser::GpuLaser(const std::string &_namePrefix, ScenePtr _scene,
                   bool _autoRender)
: Camera(_namePrefix, _scene, _autoRender)
{
  this->laserBuffer = NULL;
  this->laserScan = NULL;
  this->matFirstPass = NULL;
  this->matSecondPass = NULL;
  for (int i = 0; i < 3; ++i)
    this->firstPassTextures[i] = NULL;
  this->secondPassTexture = NULL;
  this->orthoCam = NULL;
  this->w2nd = 0;
  this->h2nd = 0;
  this->visual.reset();
}

//////////////////////////////////////////////////
GpuLaser::~GpuLaser()
{
  delete [] this->laserBuffer;
  delete [] this->laserScan;

  for (unsigned int i = 0; i < this->textureCount; ++i)
  {
    if (this->firstPassTextures[i])
    {
      Ogre::TextureManager::getSingleton().remove(
          this->firstPassTextures[i]->getName());
    }
  }
  if (this->secondPassTexture)
  {
    Ogre::TextureManager::getSingleton().remove(
        this->secondPassTexture->getName());
  }

  if (this->scene && this->orthoCam)
    this->scene->GetManager()->destroyCamera(this->orthoCam);

  this->visual.reset();
  this->texIdx.clear();
  texCount = 0;
}

//////////////////////////////////////////////////
void GpuLaser::Load(sdf::ElementPtr _sdf)
{
  Camera::Load(_sdf);
}

//////////////////////////////////////////////////
void GpuLaser::Load()
{
  Camera::Load();
}

//////////////////////////////////////////////////
void GpuLaser::Init()
{
  Camera::Init();
  this->w2nd = this->GetImageWidth();
  this->h2nd = this->GetImageHeight();
  this->visual.reset(new Visual(this->GetName()+"second_pass_canvas",
     this->GetScene()->GetWorldVisual()));
}

//////////////////////////////////////////////////
void GpuLaser::Fini()
{
  Camera::Fini();
}

//////////////////////////////////////////////////
void GpuLaser::CreateLaserTexture(const std::string &_textureName)
{
  this->camera->yaw(Ogre::Radian(this->horzHalfAngle));
  this->camera->pitch(Ogre::Radian(this->vertHalfAngle));

  this->CreateOrthoCam();

  this->textureCount = this->cameraCount;

  if (this->textureCount == 2)
  {
    cameraYaws[0] = -this->hfov/2;
    cameraYaws[1] = +this->hfov;
    cameraYaws[2] = 0;
    cameraYaws[3] = -this->hfov/2;
  }
  else
  {
    cameraYaws[0] = -this->hfov;
    cameraYaws[1] = +this->hfov;
    cameraYaws[2] = +this->hfov;
    cameraYaws[3] = -this->hfov;
  }

  for (unsigned int i = 0; i < this->textureCount; ++i)
  {
    std::stringstream texName;
    texName << _textureName << "first_pass_" << i;
    this->firstPassTextures[i] =
      Ogre::TextureManager::getSingleton().createManual(
      texName.str(), "General", Ogre::TEX_TYPE_2D,
      this->GetImageWidth(), this->GetImageHeight(), 0,
      Ogre::PF_FLOAT32_RGB, Ogre::TU_RENDERTARGET).getPointer();

    this->Set1stPassTarget(
        this->firstPassTextures[i]->getBuffer()->getRenderTarget(), i);

    this->firstPassTargets[i]->setAutoUpdated(false);
  }

  this->matFirstPass = (Ogre::Material*)(
  Ogre::MaterialManager::getSingleton().getByName("Gazebo/LaserScan1st").get());

  this->matFirstPass->load();

  this->secondPassTexture = Ogre::TextureManager::getSingleton().createManual(
      _textureName + "second_pass",
      "General",
      Ogre::TEX_TYPE_2D,
      this->GetImageWidth(), this->GetImageHeight(), 0,
      Ogre::PF_FLOAT32_RGB,
      Ogre::TU_RENDERTARGET).getPointer();

  this->Set2ndPassTarget(
      this->secondPassTexture->getBuffer()->getRenderTarget());

  this->secondPassTarget->setAutoUpdated(false);

  this->matSecondPass = (Ogre::Material*)(
  Ogre::MaterialManager::getSingleton().getByName("Gazebo/LaserScan2nd").get());

  this->matSecondPass->load();

  Ogre::TextureUnitState *texUnit;
  for (unsigned int i = 0; i < this->textureCount; ++i)
  {
    unsigned int texIndex = texCount++;
    Ogre::Technique *technique = this->matSecondPass->getTechnique(0);
    GZ_ASSERT(technique, "GpuLaser material script error: technique not found");

    Ogre::Pass *pass = technique->getPass(0);
    GZ_ASSERT(pass, "GpuLaser material script error: pass not found");

    if (!pass->getTextureUnitState(this->firstPassTextures[i]->getName()))
    {
      texUnit = pass->createTextureUnitState(
            this->firstPassTextures[i]->getName(), texIndex);

      this->texIdx.push_back(texIndex);

      texUnit->setTextureFiltering(Ogre::TFO_NONE);
      texUnit->setTextureAddressingMode(Ogre::TextureUnitState::TAM_MIRROR);
    }
  }

  this->CreateCanvas();
}

//////////////////////////////////////////////////
void GpuLaser::PostRender()
{
//  common::Timer postRenderT, blitT;
//  postRenderT.Start();
//  double blitDur = 0.0;
//  double postRenderDur = 0.0;

  for (unsigned int i = 0; i < this->textureCount; ++i)
  {
    this->firstPassTargets[i]->swapBuffers();
  }

  this->secondPassTarget->swapBuffers();

  if (this->newData && this->captureData)
  {
    Ogre::HardwarePixelBufferSharedPtr pixelBuffer;

    unsigned int width = this->secondPassViewport->getActualWidth();
    unsigned int height = this->secondPassViewport->getActualHeight();

    // Get access to the buffer and make an image and write it to file
    pixelBuffer = this->secondPassTexture->getBuffer();

    size_t size = Ogre::PixelUtil::getMemorySize(
                    width, height, 1, Ogre::PF_FLOAT32_RGB);

    // Blit the depth buffer if needed
    if (!this->laserBuffer)
      this->laserBuffer = new float[size];

    memset(this->laserBuffer, 255, size);

    Ogre::PixelBox dstBox(width, height,
        1, Ogre::PF_FLOAT32_RGB, this->laserBuffer);

//    blitT.Start();
    pixelBuffer->blitToMemory(dstBox);
//    blitDur = blitT.GetElapsed().Double();

    if (!this->laserScan)
      this->laserScan =  new float[this->w2nd * this->h2nd * 3];

    memcpy(this->laserScan, this->laserBuffer,
           this->w2nd * this->h2nd * 3 * sizeof(this->laserScan[0]));

    this->newLaserFrame(this->laserScan, this->w2nd, this->h2nd, 3, "BLABLA");
  }

  this->newData = false;
//  postRenderDur = postRenderT.GetElapsed().Double();

/*  std::cerr << " Render: " << this->lastRenderDuration * 1000
            << " BLIT: " << blitDur * 1000
            << " postRender: " << postRenderDur * 1000
            << " TOTAL: " << (this->lastRenderDuration + postRenderDur) * 1000
            << " Total - BLIT: " << (this->lastRenderDuration + postRenderDur
            - blitDur) * 1000 << "\n";   */
}

/////////////////////////////////////////////////
void GpuLaser::UpdateRenderTarget(Ogre::RenderTarget *_target,
                   Ogre::Material *_material, Ogre::Camera *_cam,
                   bool _updateTex)
{
  Ogre::RenderSystem *renderSys;
  Ogre::Viewport *vp = NULL;
  Ogre::SceneManager *sceneMgr = this->scene->GetManager();
  Ogre::Pass *pass;

  renderSys = this->scene->GetManager()->getDestinationRenderSystem();
  // Get pointer to the material pass
  pass = _material->getBestTechnique()->getPass(0);

  // Render the depth texture
  // OgreSceneManager::_render function automatically sets farClip to 0.
  // Which normally equates to infinite distance. We don't want this. So
  // we have to set the distance every time.
  _cam->setFarClipDistance(this->GetFarClip());

  Ogre::AutoParamDataSource autoParamDataSource;

  vp = _target->getViewport(0);

  // Need this line to render the ground plane. No idea why it's necessary.
  renderSys->_setViewport(vp);
  sceneMgr->_setPass(pass, true, false);
  autoParamDataSource.setCurrentPass(pass);
  autoParamDataSource.setCurrentViewport(vp);
  autoParamDataSource.setCurrentRenderTarget(_target);
  autoParamDataSource.setCurrentSceneManager(sceneMgr);
  autoParamDataSource.setCurrentCamera(_cam, true);

  renderSys->setLightingEnabled(false);
  renderSys->_setFog(Ogre::FOG_NONE);

#if OGRE_VERSION_MAJOR == 1 && OGRE_VERSION_MINOR == 6
  pass->_updateAutoParamsNoLights(&autoParamDataSource);
#else
  pass->_updateAutoParams(&autoParamDataSource, 1);
#endif

  if (_updateTex)
  {
    pass->getFragmentProgramParameters()->setNamedConstant("tex1",
      this->texIdx[0]);
    if (this->texIdx.size() > 1)
    {
      pass->getFragmentProgramParameters()->setNamedConstant("tex2",
        this->texIdx[1]);
      if (this->texIdx.size() > 2)
        pass->getFragmentProgramParameters()->setNamedConstant("tex3",
          this->texIdx[2]);
    }
  }

  // NOTE: We MUST bind parameters AFTER updating the autos
  if (pass->hasVertexProgram())
  {
    renderSys->bindGpuProgram(
        pass->getVertexProgram()->_getBindingDelegate());

#if OGRE_VERSION_MAJOR == 1 && OGRE_VERSION_MINOR == 6
    renderSys->bindGpuProgramParameters(Ogre::GPT_VERTEX_PROGRAM,
    pass->getVertexProgramParameters());
#else
    renderSys->bindGpuProgramParameters(Ogre::GPT_VERTEX_PROGRAM,
      pass->getVertexProgramParameters(), 1);
#endif
  }

  if (pass->hasFragmentProgram())
  {
    renderSys->bindGpuProgram(
    pass->getFragmentProgram()->_getBindingDelegate());

#if OGRE_VERSION_MAJOR == 1 && OGRE_VERSION_MINOR == 6
    renderSys->bindGpuProgramParameters(Ogre::GPT_FRAGMENT_PROGRAM,
    pass->getFragmentProgramParameters());
#else
      renderSys->bindGpuProgramParameters(Ogre::GPT_FRAGMENT_PROGRAM,
      pass->getFragmentProgramParameters(), 1);
#endif
  }
}

/////////////////////////////////////////////////
void GpuLaser::notifyRenderSingleObject(Ogre::Renderable *_rend,
      const Ogre::Pass* /*pass*/, const Ogre::AutoParamDataSource* /*source*/,
      const Ogre::LightList* /*lights*/, bool /*supp*/)
{
  Ogre::Vector4 retro = Ogre::Vector4(0, 0, 0, 0);
  try
  {
    retro = _rend->getCustomParameter(1);
  }
  catch(Ogre::ItemIdentityException& e)
  {
    _rend->setCustomParameter(1, Ogre::Vector4(0, 0, 0, 0));
  }

  Ogre::Pass *pass = this->currentMat->getBestTechnique()->getPass(0);
  Ogre::RenderSystem *renderSys =
                  this->scene->GetManager()->getDestinationRenderSystem();

  Ogre::AutoParamDataSource autoParamDataSource;

  Ogre::Viewport *vp = this->currentTarget->getViewport(0);

  renderSys->_setViewport(vp);
  autoParamDataSource.setCurrentRenderable(_rend);
  autoParamDataSource.setCurrentPass(pass);
  autoParamDataSource.setCurrentViewport(vp);
  autoParamDataSource.setCurrentRenderTarget(this->currentTarget);
  autoParamDataSource.setCurrentSceneManager(this->scene->GetManager());
  autoParamDataSource.setCurrentCamera(this->camera, true);

  pass->_updateAutoParams(&autoParamDataSource,
      Ogre::GPV_GLOBAL || Ogre::GPV_PER_OBJECT);
  pass->getFragmentProgramParameters()->setNamedConstant("retro", retro[0]);
  renderSys->bindGpuProgram(
      pass->getVertexProgram()->_getBindingDelegate());

  renderSys->bindGpuProgramParameters(Ogre::GPT_VERTEX_PROGRAM,
      pass->getVertexProgramParameters(),
      Ogre::GPV_GLOBAL || Ogre::GPV_PER_OBJECT);

  renderSys->bindGpuProgram(
      pass->getFragmentProgram()->_getBindingDelegate());

  renderSys->bindGpuProgramParameters(Ogre::GPT_FRAGMENT_PROGRAM,
      pass->getFragmentProgramParameters(),
      Ogre::GPV_GLOBAL || Ogre::GPV_PER_OBJECT);
}

//////////////////////////////////////////////////
void GpuLaser::RenderImpl()
{
  common::Timer firstPassTimer, secondPassTimer;

  firstPassTimer.Start();

  Ogre::SceneManager *sceneMgr = this->scene->GetManager();

  sceneMgr->_suppressRenderStateChanges(true);
  sceneMgr->addRenderObjectListener(this);

  for (unsigned int i = 0; i < this->textureCount; ++i)
  {
    if (this->textureCount > 1)
    {
      // Cannot call Camera::RotateYaw because it rotates in world frame,
      // but we need rotation in camera local frame
      this->sceneNode->roll(Ogre::Radian(this->cameraYaws[i]));
    }

    this->currentMat = this->matFirstPass;
    this->currentTarget = this->firstPassTargets[i];

    this->UpdateRenderTarget(this->firstPassTargets[i],
                  this->matFirstPass, this->camera);
    this->firstPassTargets[i]->update(false);
  }

  if (this->textureCount > 1)
      this->sceneNode->roll(Ogre::Radian(this->cameraYaws[3]));

  sceneMgr->removeRenderObjectListener(this);

  double firstPassDur = firstPassTimer.GetElapsed().Double();
  secondPassTimer.Start();

  this->visual->SetVisible(true);

  this->UpdateRenderTarget(this->secondPassTarget,
                this->matSecondPass, this->orthoCam, true);
  this->secondPassTarget->update(false);

  this->visual->SetVisible(false);

  sceneMgr->_suppressRenderStateChanges(false);

  double secondPassDur = secondPassTimer.GetElapsed().Double();
  this->lastRenderDuration = firstPassDur + secondPassDur;
}

//////////////////////////////////////////////////
const float* GpuLaser::GetLaserData()
{
  return this->laserBuffer;
}

/////////////////////////////////////////////////
void GpuLaser::CreateOrthoCam()
{
  this->pitchNodeOrtho =
    this->GetScene()->GetWorldVisual()->GetSceneNode()->createChildSceneNode();

  this->orthoCam = this->scene->GetManager()->createCamera(
        this->pitchNodeOrtho->getName() + "_ortho_cam");

  // Use X/Y as horizon, Z up
  this->orthoCam->pitch(Ogre::Degree(90));

  // Don't yaw along variable axis, causes leaning
  this->orthoCam->setFixedYawAxis(true, Ogre::Vector3::UNIT_Z);

  this->orthoCam->setDirection(1, 0, 0);

  this->pitchNodeOrtho->attachObject(this->orthoCam);
  this->orthoCam->setAutoAspectRatio(true);

  if (this->orthoCam)
  {
    this->orthoCam->setNearClipDistance(0.01);
    this->orthoCam->setFarClipDistance(0.02);
    this->orthoCam->setRenderingDistance(0.02);

    this->orthoCam->setProjectionType(Ogre::PT_ORTHOGRAPHIC);
  }
}

/////////////////////////////////////////////////
Ogre::Matrix4 GpuLaser::BuildScaledOrthoMatrix(float _left, float _right,
           float _bottom, float _top, float _near, float _far)
{
  float invw = 1 / (_right - _left);
  float invh = 1 / (_top - _bottom);
  float invd = 1 / (_far - _near);

  Ogre::Matrix4 proj = Ogre::Matrix4::ZERO;
  proj[0][0] = 2 * invw;
  proj[0][3] = -(_right + _left) * invw;
  proj[1][1] = 2 * invh;
  proj[1][3] = -(_top + _bottom) * invh;
  proj[2][2] = -2 * invd;
  proj[2][3] = -(_far + _near) * invd;
  proj[3][3] = 1;

  return proj;
}

//////////////////////////////////////////////////
void GpuLaser::Set1stPassTarget(Ogre::RenderTarget *_target,
                                unsigned int _index)
{
  this->firstPassTargets[_index] = _target;

  if (this->firstPassTargets[_index])
  {
    // Setup the viewport to use the texture
    this->firstPassViewports[_index] =
      this->firstPassTargets[_index]->addViewport(this->camera);
    this->firstPassViewports[_index]->setClearEveryFrame(true);
    this->firstPassViewports[_index]->setOverlaysEnabled(false);
    this->firstPassViewports[_index]->setShadowsEnabled(false);
    this->firstPassViewports[_index]->setSkiesEnabled(false);
    this->firstPassViewports[_index]->setBackgroundColour(
        Ogre::ColourValue(this->farClip, 0.0, 1.0));
    this->firstPassViewports[_index]->setVisibilityMask(
        GZ_VISIBILITY_ALL & ~(GZ_VISIBILITY_GUI | GZ_VISIBILITY_SELECTABLE));
  }
  if (_index == 0)
  {
    this->camera->setAspectRatio(this->rayCountRatio);
    this->camera->setFOVy(Ogre::Radian(this->vfov));
  }
}

//////////////////////////////////////////////////
void GpuLaser::Set2ndPassTarget(Ogre::RenderTarget *_target)
{
  this->secondPassTarget = _target;

  if (this->secondPassTarget)
  {
    // Setup the viewport to use the texture
    this->secondPassViewport =
        this->secondPassTarget->addViewport(this->orthoCam);
    this->secondPassViewport->setClearEveryFrame(true);
    this->secondPassViewport->setOverlaysEnabled(false);
    this->secondPassViewport->setShadowsEnabled(false);
    this->secondPassViewport->setSkiesEnabled(false);
    this->secondPassViewport->setBackgroundColour(
        Ogre::ColourValue(0.0, 1.0, 0.0));
    this->secondPassViewport->setVisibilityMask(
        GZ_VISIBILITY_ALL & ~(GZ_VISIBILITY_GUI | GZ_VISIBILITY_SELECTABLE));
  }
  Ogre::Matrix4 p = this->BuildScaledOrthoMatrix(
      0, static_cast<float>(this->GetImageWidth() / 10.0),
      0, static_cast<float>(this->GetImageHeight() / 10.0),
      0.01, 0.02);

  this->orthoCam->setCustomProjectionMatrix(true, p);
}

/////////////////////////////////////////////////
void GpuLaser::SetRangeCount(unsigned int _w, unsigned int _h)
{
  this->w2nd = _w;
  this->h2nd = _h;
}

/////////////////////////////////////////////////
void GpuLaser::CreateMesh()
{
  std::string meshName = this->GetName() + "_undistortion_mesh";

  common::Mesh *mesh = new common::Mesh();
  mesh->SetName(meshName);

  common::SubMesh *submesh = new common::SubMesh();

  double dx, dy;
  submesh->SetPrimitiveType(common::SubMesh::POINTS);

  double viewHeight = this->GetImageHeight()/10.0;

  if (h2nd == 1)
    dy = 0;
  else
    dy = 0.1;

  dx = 0.1;

  double startX = dx;
  double startY = viewHeight;

  double phi = this->vfov / 2;

  double vAngMin = -phi;

  if (this->GetImageHeight() == 1)
    phi = 0;

  unsigned int ptsOnLine = 0;
  for (unsigned int j = 0; j < this->h2nd; ++j)
  {
    double gamma = 0;
    if (this->h2nd != 1)
      gamma = ((2 * phi / (this->h2nd - 1)) * j) + vAngMin;
    for (unsigned int i = 0; i < this->w2nd; ++i)
    {
      double thfov = this->textureCount * this->hfov;
      double theta = this->hfov / 2;
      double delta = ((thfov / (this->w2nd - 1)) * i);

      unsigned int texture = delta / (theta*2);

      if (texture > this->textureCount-1)
      {
        texture -= 1;
        delta -= (thfov / (this->w2nd - 1));
      }

      delta = delta - (texture * (theta*2));

      delta = delta - theta;

      startX -= dx;
      if (ptsOnLine == this->GetImageWidth())
      {
        ptsOnLine = 0;
        startX = 0;
        startY -= dy;
      }
      ptsOnLine++;
      submesh->AddVertex(texture/1000.0, startX, startY);

      double u, v;
      if (this->isHorizontal)
      {
        u = -(cos(phi) * tan(delta))/(2 * tan(theta) * cos(gamma)) + 0.5;
        v = math::equal(phi, 0.0) ? -tan(gamma)/(2 * tan(phi)) + 0.5 : 0.5;
      }
      else
      {
        v = -(cos(theta) * tan(gamma))/(2 * tan(phi) * cos(delta)) + 0.5;
        u = math::equal(theta, 0.0) ? -tan(delta)/(2 * tan(theta)) + 0.5 : 0.5;
      }
      submesh->AddTexCoord(u, v);
    }
  }

  for (unsigned int j = 0; j < (this->h2nd ); ++j)
    for (unsigned int i = 0; i < (this->w2nd ); ++i)
      submesh->AddIndex(this->w2nd * j + i);

  mesh->AddSubMesh(submesh);

  this->undistMesh = mesh;

  common::MeshManager::Instance()->AddMesh(this->undistMesh);
}

/////////////////////////////////////////////////
void GpuLaser::CreateCanvas()
{
  this->CreateMesh();

  Ogre::Node *parent = this->visual->GetSceneNode()->getParent();
  parent->removeChild(this->visual->GetSceneNode());

  this->pitchNodeOrtho->addChild(this->visual->GetSceneNode());

  this->visual->InsertMesh(this->undistMesh);

  std::ostringstream stream;
  std::string meshName = this->undistMesh->GetName();
  stream << this->visual->GetSceneNode()->getName() << "_ENTITY_" << meshName;

  this->object = (Ogre::MovableObject*)
      (this->visual->GetSceneNode()->getCreator()->createEntity(stream.str(),
        meshName));

  this->visual->AttachObject(this->object);
  this->object->setVisibilityFlags(GZ_VISIBILITY_ALL
      & ~GZ_VISIBILITY_SELECTABLE);

  ignition::math::Pose3d pose;
  pose.Pos().Set(0.01, 0, 0);
  pose.Rot().Euler(ignition::math::Vector3d(0, 0, 0));

  this->visual->SetPose(pose);

  this->visual->SetMaterial("Gazebo/Green");
  this->visual->SetAmbient(common::Color(0, 1, 0, 1));
  this->visual->SetVisible(true);
  this->scene->AddVisual(this->visual);
}

//////////////////////////////////////////////////
void GpuLaser::SetHorzHalfAngle(double _angle)
{
  this->horzHalfAngle = _angle;
}

//////////////////////////////////////////////////
void GpuLaser::SetVertHalfAngle(double _angle)
{
  this->vertHalfAngle = _angle;
}

//////////////////////////////////////////////////
double GpuLaser::GetHorzHalfAngle() const
{
  return this->horzHalfAngle;
}

//////////////////////////////////////////////////
double GpuLaser::GetVertHalfAngle() const
{
  return this->vertHalfAngle;
}

//////////////////////////////////////////////////
void GpuLaser::SetIsHorizontal(bool _horizontal)
{
  this->isHorizontal = _horizontal;
}

//////////////////////////////////////////////////
bool GpuLaser::IsHorizontal() const
{
  return this->isHorizontal;
}

//////////////////////////////////////////////////
double GpuLaser::GetHorzFOV() const
{
  return this->hfov;
}

//////////////////////////////////////////////////
double GpuLaser::GetVertFOV() const
{
  return this->vfov;
}

//////////////////////////////////////////////////
void GpuLaser::SetHorzFOV(double _hfov)
{
  this->hfov = _hfov;
}

//////////////////////////////////////////////////
void GpuLaser::SetVertFOV(double _vfov)
{
  this->vfov = _vfov;
}

//////////////////////////////////////////////////
double GpuLaser::GetCosHorzFOV() const
{
  return this->chfov;
}

//////////////////////////////////////////////////
void GpuLaser::SetCosHorzFOV(double _chfov)
{
  this->chfov = _chfov;
}

//////////////////////////////////////////////////
double GpuLaser::GetCosVertFOV() const
{
  return this->cvfov;
}

//////////////////////////////////////////////////
void GpuLaser::SetCosVertFOV(double _cvfov)
{
  this->cvfov = _cvfov;
}

//////////////////////////////////////////////////
double GpuLaser::GetNearClip() const
{
  return this->nearClip;
}

//////////////////////////////////////////////////
double GpuLaser::GetFarClip() const
{
  return this->farClip;
}

//////////////////////////////////////////////////
void GpuLaser::SetNearClip(double _near)
{
  this->nearClip = _near;
}

//////////////////////////////////////////////////
void GpuLaser::SetFarClip(double _far)
{
  this->farClip = _far;
}

//////////////////////////////////////////////////
double GpuLaser::GetCameraCount() const
{
  return this->cameraCount;
}

//////////////////////////////////////////////////
void GpuLaser::SetCameraCount(double _cameraCount)
{
  this->cameraCount = _cameraCount;
}
//////////////////////////////////////////////////
double GpuLaser::GetRayCountRatio() const
{
  return this->rayCountRatio;
}

//////////////////////////////////////////////////
void GpuLaser::SetRayCountRatio(double _rayCountRatio)
{
  this->rayCountRatio = _rayCountRatio;
}
