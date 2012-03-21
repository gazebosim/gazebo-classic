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

/* Desc: A camera sensor using OpenGL
 * Author: Nate Koenig
 * Date: 15 July 2003
 */

#include <dirent.h>
#include <sstream>

#include "sdf/sdf.h"
#include "rendering/ogre.h"
#include "rendering/RTShaderSystem.hh"

#include "common/Events.hh"
#include "common/Console.hh"
#include "common/Exception.hh"
#include "common/Mesh.hh"
#include "common/MeshManager.hh"
#include "sensors/VisualLaserSensor.hh"
#include "math/Pose.hh"

#include "rendering/Visual.hh"
#include "rendering/Conversions.hh"
#include "rendering/Scene.hh"
#include "rendering/VisualLaser.hh"

using namespace gazebo;
using namespace rendering;


//////////////////////////////////////////////////
VisualLaser::VisualLaser(const std::string &_namePrefix, Scene *_scene,
                         bool _autoRender)
: Camera(_namePrefix, _scene, _autoRender)
{
  this->_1stTarget = NULL;
  this->_2ndTarget = NULL;
  this->_1stTarget_dbg = NULL;
  this->_2ndTarget_dbg = NULL;
  this->laserBuffer = NULL;
  this->laserScan = NULL;
  this->imageBuffer = NULL;
  this->_1stBuffer = NULL;
  this->mat_1st_pass = NULL;
  this->mat_2nd_pass = NULL;
  this->mat_1st_pass_dbg = NULL;
  this->mat_2nd_pass_dbg = NULL;
  this->w2nd = 0;
  this->h2nd = 0;
  this->visual.reset();
}

//////////////////////////////////////////////////
VisualLaser::~VisualLaser()
{
}

//////////////////////////////////////////////////
void VisualLaser::Load(sdf::ElementPtr &_sdf)
{
  Camera::Load(_sdf);
}

//////////////////////////////////////////////////
void VisualLaser::Load()
{
  Camera::Load();
}

//////////////////////////////////////////////////
void VisualLaser::Init()
{
  Camera::Init();
  this->w2nd = this->GetImageWidth();
  this->h2nd = this->GetImageHeight();
  this->visual.reset(new Visual(this->GetName()+"_2nd_pass_canvas", this->GetScene()->GetWorldVisual()));
}

//////////////////////////////////////////////////
void VisualLaser::Fini()
{
  Camera::Fini();
}

//////////////////////////////////////////////////
void VisualLaser::CreateLaserTexture(const std::string &_textureName)
{
  this->CreateOrthoCam();
  
  this->_1stTexture = Ogre::TextureManager::getSingleton().createManual(
      _textureName + "_1st_pass",
      "General",
      Ogre::TEX_TYPE_2D,
      this->GetImageWidth(), this->GetImageHeight(), 0,
      Ogre::PF_FLOAT32_RGB,
      Ogre::TU_RENDERTARGET).getPointer();

  this->_1stTexture_dbg = Ogre::TextureManager::getSingleton().createManual(
      _textureName + "_1st_pass_dbg",
      "General",
      Ogre::TEX_TYPE_2D,
      this->GetImageWidth(), this->GetImageHeight(), 0,
      Ogre::PF_BYTE_RGB,
      Ogre::TU_RENDERTARGET).getPointer();

  this->Set1stTarget(this->_1stTexture->getBuffer()->getRenderTarget());
  RTShaderSystem::AttachViewport(this->_1stViewport, this->GetScene());
  RTShaderSystem::AttachViewport(this->_1stViewport_dbg, this->GetScene());
  this->_1stTarget->setAutoUpdated(false);
  this->_1stTarget_dbg->setAutoUpdated(false);

  this->mat_1st_pass = (Ogre::Material*)(
      Ogre::MaterialManager::getSingleton().getByName("Gazebo/LaserScan1st").getPointer());

  this->mat_1st_pass->load();
 
  this->mat_1st_pass_dbg = (Ogre::Material*)(
      Ogre::MaterialManager::getSingleton().getByName("Gazebo/LaserScan1stDBG").getPointer());

  this->mat_1st_pass_dbg->load();
 
  this->_2ndTexture = Ogre::TextureManager::getSingleton().createManual(
      _textureName + "_2nd_pass",
      "General",
      Ogre::TEX_TYPE_2D,
      this->GetImageWidth(), this->GetImageHeight(), 0,
      Ogre::PF_FLOAT32_RGB,
      Ogre::TU_RENDERTARGET).getPointer();

  this->_2ndTexture_dbg = Ogre::TextureManager::getSingleton().createManual(
      _textureName + "_2nd_pass_dbg",
      "General",
      Ogre::TEX_TYPE_2D,
      this->GetImageWidth(), this->GetImageHeight(), 0,
      Ogre::PF_BYTE_RGB,
      Ogre::TU_RENDERTARGET).getPointer();

  this->Set2ndTarget(this->_2ndTexture->getBuffer()->getRenderTarget());
  RTShaderSystem::AttachViewport(this->_2ndViewport, this->GetScene());
  RTShaderSystem::AttachViewport(this->_2ndViewport_dbg, this->GetScene());
  this->_2ndTarget->setAutoUpdated(false);
  this->_2ndTarget_dbg->setAutoUpdated(false);
  
  this->mat_2nd_pass = (Ogre::Material*)(
      Ogre::MaterialManager::getSingleton().getByName("Gazebo/LaserScan2nd").getPointer());

  this->mat_2nd_pass->load();
  
  Ogre::TextureUnitState *tex_unit = 
    this->mat_2nd_pass->getTechnique(0)->getPass(0)->createTextureUnitState(this->_1stTexture->getName());

  tex_unit->setTextureFiltering(Ogre::TFO_NONE);
  tex_unit->setTextureAddressingMode(Ogre::TextureUnitState::TAM_MIRROR);

  this->mat_2nd_pass_dbg = (Ogre::Material*)(
      Ogre::MaterialManager::getSingleton().getByName("Gazebo/LaserScan2ndDBG").getPointer());
  this->mat_2nd_pass_dbg->load();
  tex_unit = this->mat_2nd_pass_dbg->getTechnique(0)->getPass(0)->createTextureUnitState(this->_1stTexture_dbg->getName());
  tex_unit->setTextureFiltering(Ogre::TFO_NONE);
  tex_unit->setTextureAddressingMode(Ogre::TextureUnitState::TAM_MIRROR);

  this->CreateCanvas();
}

//////////////////////////////////////////////////
void VisualLaser::PostRender()
{
  this->_1stTarget->swapBuffers();
  this->_1stTarget_dbg->swapBuffers();
  this->_2ndTarget->swapBuffers();
  this->_2ndTarget_dbg->swapBuffers();
  if (this->newData && this->captureData)
  {
    this->Publish1stTexture();
    this->Publish2ndTexture();

    Ogre::HardwarePixelBufferSharedPtr pixelBuffer;

    unsigned int width = this->_2ndViewport->getActualWidth();
    unsigned int height = this->_2ndViewport->getActualHeight();

    // Get access to the buffer and make an image and write it to file
    pixelBuffer = this->_2ndTexture->getBuffer();

    size_t size = Ogre::PixelUtil::getMemorySize(width, height, 1, Ogre::PF_FLOAT32_RGB);

    // Blit the depth buffer if needed
    if (!this->laserBuffer)
      this->laserBuffer = new float[size];
    
    memset(this->laserBuffer, 255, size);

    Ogre::PixelBox dst_box(width, height,
        1, Ogre::PF_FLOAT32_RGB, this->laserBuffer);

    pixelBuffer->blitToMemory(dst_box);

    if (!this->laserScan)
      this->laserScan =  new float[this->w2nd * this->h2nd * 3];

    memcpy(this->laserScan, this->laserBuffer, this->w2nd*this->h2nd*3*sizeof(float));
  
    //this->newLaserFrame(this->laserBuffer, width, height, 3,"BLABLA");
    this->newLaserFrame(this->laserScan, this->w2nd, this->h2nd, 3,"BLABLA");
  }

  this->newData = false;
}

void VisualLaser::Publish1stTexture()
{
  Ogre::HardwarePixelBufferSharedPtr pixelBuffer;

  unsigned int w1 = this->_1stViewport_dbg->getActualWidth();
  unsigned int h1 = this->_1stViewport_dbg->getActualHeight();

  // Get access to the buffer and make an image and write it to file
  pixelBuffer = this->_1stTexture_dbg->getBuffer();

  size_t size = Ogre::PixelUtil::getMemorySize(w1, h1, 1, Ogre::PF_BYTE_RGB);

  if (!this->_1stBuffer)
    this->_1stBuffer = new unsigned char[size];
  
  memset(this->_1stBuffer, 255, size);
      
  Ogre::PixelBox dst_box(w1, h1,
      1, Ogre::PF_BYTE_RGB, this->_1stBuffer);

  pixelBuffer->blitToMemory(dst_box);

  this->newImage2Frame(this->_1stBuffer, w1, h1, 3, "BLABLA");
}

void VisualLaser::Publish2ndTexture()
{
  Ogre::HardwarePixelBufferSharedPtr pixelBuffer;

  unsigned int width = this->_2ndViewport_dbg->getActualWidth();
  unsigned int height = this->_2ndViewport_dbg->getActualHeight();

  // Get access to the buffer and make an image and write it to file
  pixelBuffer = this->_2ndTexture_dbg->getBuffer();

  size_t size = Ogre::PixelUtil::getMemorySize(width, height, 1, Ogre::PF_BYTE_RGB);

  if (!this->imageBuffer)
    this->imageBuffer = new unsigned char[size];
  
  memset(this->imageBuffer, 255, size);
      
  Ogre::PixelBox dst_box(width, height,
      1, Ogre::PF_BYTE_RGB, this->imageBuffer);

  pixelBuffer->blitToMemory(dst_box);

  unsigned char *result = new unsigned char[this->w2nd*this->h2nd*3];

  memcpy(result, this->imageBuffer, this->w2nd*this->h2nd*3);

  this->newImageFrame(result, this->w2nd, this->h2nd, 3, "BLABLA");
  //this->newImageFrame(this->imageBuffer, width, height, 3, "BLABLA");
}

void VisualLaser::UpdateRenderTarget(Ogre::RenderTarget *target, Ogre::Material *material, Ogre::Camera *cam)
{
  Ogre::RenderSystem *renderSys;
  Ogre::Viewport *vp = NULL;
  Ogre::SceneManager *sceneMgr = this->scene->GetManager();
  Ogre::Pass *pass;

  renderSys = this->scene->GetManager()->getDestinationRenderSystem();
  // Get pointer to the material pass
  pass = material->getBestTechnique()->getPass(0);

  // Render the depth texture
  // OgreSceneManager::_render function automatically sets farClip to 0.
  // Which normally equates to infinite distance. We don't want this. So
  // we have to set the distance every time.
  cam->setFarClipDistance(this->GetFarClip());

  Ogre::AutoParamDataSource autoParamDataSource;

  vp = target->getViewport(0);

  //Ogre::CompositorManager::getSingleton().setCompositorEnabled(vp, matName, true);

  // Need this line to render the ground plane. No idea why it's necessary.
  renderSys->_setViewport(vp);
  sceneMgr->_setPass(pass, true, false);
  autoParamDataSource.setCurrentPass(pass);
  autoParamDataSource.setCurrentViewport(vp);
  autoParamDataSource.setCurrentRenderTarget(target);
  autoParamDataSource.setCurrentSceneManager(sceneMgr);
  autoParamDataSource.setCurrentCamera(cam, true);

  renderSys->setLightingEnabled(false);
  renderSys->_setFog(Ogre::FOG_NONE);

  // These two lines don't seem to do anything useful
  /*renderSys->_setProjectionMatrix(
      this->GetOgreCamera()->getProjectionMatrixRS());
  renderSys->_setViewMatrix(this->GetOgreCamera()->getViewMatrix(true));*/

#if OGRE_VERSION_MAJOR == 1 && OGRE_VERSION_MINOR == 6
  pass->_updateAutoParamsNoLights(&autoParamDataSource);
#else
  pass->_updateAutoParams(&autoParamDataSource, 1);
#endif

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
      
void VisualLaser::notifyRenderSingleObject (Ogre::Renderable *rend, const Ogre::Pass* /*pass*/, 
              const Ogre::AutoParamDataSource* /*source*/, const Ogre::LightList* /*lights*/, bool /*supp*/)
{
  Ogre::Vector4 retro = Ogre::Vector4(0,0,0,0);
  try
  {
    retro = rend->getCustomParameter(1);
  }
  catch (Ogre::ItemIdentityException e)
  {
    rend->setCustomParameter(1, Ogre::Vector4(0,0,0,0));
  }

  Ogre::Pass *my_pass = this->current_mat->getBestTechnique()->getPass(0);
  Ogre::RenderSystem *renderSys = this->scene->GetManager()->getDestinationRenderSystem();

  Ogre::AutoParamDataSource autoParamDataSource;

  Ogre::Viewport *vp = this->current_target->getViewport(0);

  renderSys->_setViewport(vp);
  //this->scene->GetManager()->_setPass(pass, true, false);
  autoParamDataSource.setCurrentRenderable(rend);
  autoParamDataSource.setCurrentPass(my_pass);
  autoParamDataSource.setCurrentViewport(vp);
  autoParamDataSource.setCurrentRenderTarget(this->current_target);
  autoParamDataSource.setCurrentSceneManager(this->scene->GetManager());
  autoParamDataSource.setCurrentCamera(this->camera, true);

  my_pass->_updateAutoParams(&autoParamDataSource,  Ogre::GPV_GLOBAL || Ogre::GPV_PER_OBJECT);
  my_pass->getFragmentProgramParameters()->setNamedConstant("retro", retro[0]);
 
  renderSys->bindGpuProgram(
    my_pass->getVertexProgram()->_getBindingDelegate());

  renderSys->bindGpuProgramParameters(Ogre::GPT_VERTEX_PROGRAM,
    my_pass->getVertexProgramParameters(), Ogre::GPV_GLOBAL || Ogre::GPV_PER_OBJECT);
  
  renderSys->bindGpuProgram(
  my_pass->getFragmentProgram()->_getBindingDelegate());

  renderSys->bindGpuProgramParameters(Ogre::GPT_FRAGMENT_PROGRAM,
  my_pass->getFragmentProgramParameters(), Ogre::GPV_GLOBAL || Ogre::GPV_PER_OBJECT);
}

//////////////////////////////////////////////////
void VisualLaser::RenderImpl()
{
  Ogre::SceneManager *sceneMgr = this->scene->GetManager();

  sceneMgr->setShadowTechnique(Ogre::SHADOWTYPE_NONE);
  sceneMgr->_suppressRenderStateChanges(true);

  sceneMgr->addRenderObjectListener(this);

  this->current_mat = this->mat_1st_pass_dbg;
  this->current_target = this->_1stTarget_dbg;

  //this->UpdateRenderTarget(this->_1stTarget_dbg, this->mat_1st_pass_dbg, this->camera);
  this->_1stTarget_dbg->update(false);
  
  this->current_mat = this->mat_1st_pass;
  this->current_target = this->_1stTarget;

  //this->UpdateRenderTarget(this->_1stTarget, this->mat_1st_pass, this->camera);
  this->_1stTarget->update(false);

  sceneMgr->removeRenderObjectListener(this);
  
  this->visual->SetVisible(true);
  
  this->UpdateRenderTarget(this->_2ndTarget, this->mat_2nd_pass, this->orthoCam);
  this->_2ndTarget->update(false);
  
  this->UpdateRenderTarget(this->_2ndTarget_dbg, this->mat_2nd_pass_dbg, this->orthoCam);
  this->_2ndTarget_dbg->update(false);
  
  this->visual->SetVisible(false);

  sceneMgr->_suppressRenderStateChanges(false);
  sceneMgr->setShadowTechnique(Ogre::SHADOWTYPE_TEXTURE_MODULATIVE_INTEGRATED);
}

//////////////////////////////////////////////////
const float* VisualLaser::GetLaserData()
{
  return this->laserBuffer;
}

void VisualLaser::CreateOrthoCam()
{
  this->orthoCam = this->scene->GetManager()->createCamera(this->GetName() + "_ortho_cam");

  // Use X/Y as horizon, Z up
  this->orthoCam->pitch(Ogre::Degree(90));

  // Don't yaw along variable axis, causes leaning
  this->orthoCam->setFixedYawAxis(true, Ogre::Vector3::UNIT_Z);

  this->orthoCam->setDirection(1, 0, 0);
  
  this->pitchNode_ortho =                  
    this->sceneNode->createChildSceneNode(this->GetName()+"OrthoPitchNode");
  this->pitchNode_ortho->pitch(Ogre::Degree(0));

  this->pitchNode_ortho->attachObject(this->orthoCam);
  this->orthoCam->setAutoAspectRatio(true);
  
  if (this->orthoCam)
  {
    this->orthoCam->setNearClipDistance(0.01);
    this->orthoCam->setFarClipDistance(0.02);
    this->orthoCam->setRenderingDistance(0.02);
    
    this->orthoCam->setProjectionType(Ogre::PT_ORTHOGRAPHIC);
  }

}

Ogre::Matrix4 VisualLaser::BuildScaledOrthoMatrix(float left, float right, float bottom, float top, float near, float far)
{
  float invw = 1 / (right - left);
  float invh = 1 / (top - bottom);
  float invd = 1 / (far - near);

  Ogre::Matrix4 proj = Ogre::Matrix4::ZERO;
  proj[0][0] = 2 * invw;
  proj[0][3] = -(right + left) * invw;
  proj[1][1] = 2 * invh;
  proj[1][3] = -(top + bottom) * invh;
  proj[2][2] = -2 * invd;
  proj[2][3] = -(far + near) * invd;
  proj[3][3] = 1;

  return proj;
}

//////////////////////////////////////////////////
void VisualLaser::Set1stTarget(Ogre::RenderTarget *target)
{
  this->_1stTarget = target;
  this->_1stTarget_dbg = this->_1stTexture_dbg->getBuffer()->getRenderTarget();

  if (this->_1stTarget)
  {
    // Setup the viewport to use the texture
    this->_1stViewport = this->_1stTarget->addViewport(this->camera);
    this->_1stViewport->setClearEveryFrame(true);
    this->_1stViewport->setBackgroundColour(Ogre::ColourValue(0.0,1.0,0.0));
    this->_1stViewport->setVisibilityMask(
        GZ_VISIBILITY_ALL & ~GZ_VISIBILITY_GUI);
    this->_1stViewport_dbg = this->_1stTarget_dbg->addViewport(this->camera);
    this->_1stViewport_dbg->setClearEveryFrame(true);
    this->_1stViewport_dbg->setBackgroundColour(Ogre::ColourValue(0.0,1.0,0.0));
    this->_1stViewport_dbg->setVisibilityMask(
        GZ_VISIBILITY_ALL & ~GZ_VISIBILITY_GUI);
  }
  this->camera->setAspectRatio(this->parent_sensor->Get1stRatio());
  this->camera->setFOVy(Ogre::Radian(this->parent_sensor->GetVFOV()));
}

//////////////////////////////////////////////////
void VisualLaser::Set2ndTarget(Ogre::RenderTarget *target)
{
  this->_2ndTarget = target;
  this->_2ndTarget_dbg = this->_2ndTexture_dbg->getBuffer()->getRenderTarget();

  if (this->_2ndTarget)
  {
    // Setup the viewport to use the texture
    this->_2ndViewport = this->_2ndTarget->addViewport(this->orthoCam);
    this->_2ndViewport->setClearEveryFrame(true);
    this->_2ndViewport->setBackgroundColour(Ogre::ColourValue(0.0,0.0,1.0));
    this->_2ndViewport->setVisibilityMask(
        GZ_VISIBILITY_ALL & ~GZ_VISIBILITY_GUI);
    this->_2ndViewport_dbg = this->_2ndTarget_dbg->addViewport(this->orthoCam);
    this->_2ndViewport_dbg->setClearEveryFrame(true);
    this->_2ndViewport_dbg->setBackgroundColour(Ogre::ColourValue(0.0,0.0,1.0));
    this->_2ndViewport_dbg->setVisibilityMask(
        GZ_VISIBILITY_ALL & ~GZ_VISIBILITY_GUI);
  }
  Ogre::Matrix4 p = this->BuildScaledOrthoMatrix(0, (float) (this->GetImageWidth()/10.0),
                       0, (float)(this->GetImageHeight()/10.0), 0.01, 0.02);

  this->orthoCam->setCustomProjectionMatrix(true, p);
}

void VisualLaser::SetRangeCount(unsigned int _w, unsigned int _h)
{
  this->w2nd = _w;
  this->h2nd = _h;
}

void VisualLaser::SetParentSensor(sensors::VisualLaserSensor *parent)
{
  this->parent_sensor = parent;
}

void VisualLaser::CreateMesh()
{
  std::string meshName = this->GetName() + "_undistortion_mesh";
  
  common::Mesh *mesh = new common::Mesh();
  mesh->SetName(meshName);
  
  common::SubMesh *submesh = new common::SubMesh();

  double dx, dy;
  submesh->SetPrimitiveType(common::SubMesh::POINTS);
  
  double view_height = this->GetImageHeight()/10.0;

  if (h2nd == 1)
    dy = 0;
  else
    dy = 0.1;

  dx = 0.1;
  
  double start_x = dx;
  double start_y = view_height;

  double h_ang_min = this->parent_sensor->GetAngleMin().GetAsRadian();
  double v_ang_min = this->parent_sensor->GetVerticalAngleMin().GetAsRadian();

  double theta = this->parent_sensor->GetHFOV() / 2;
  double phi = this->parent_sensor->GetVFOV() / 2;

  if (this->GetImageHeight() == 1)
    phi = 0;
 
  unsigned int pts_on_line = 0;
  
  for (unsigned int j = 0; j < this->h2nd; j++)
  {
    double gamma = 0;
    if (this->h2nd != 1)
      gamma = ((2 * phi / (this->h2nd - 1)) * j) + v_ang_min;
    for (unsigned int i = 0; i < this->w2nd; i++)
    {
      start_x -= dx;
      if (pts_on_line == this->GetImageWidth())
      {
        pts_on_line = 0;
        start_x = 0;
        start_y -= dy;
      }
      pts_on_line++;
      submesh->AddVertex(0, start_x, start_y );

      double delta = ((2 * theta / (this->w2nd - 1)) * i) + h_ang_min;
      
      double u = -(cos(phi) * tan(delta))/(2 * tan(theta) * cos(gamma)) + 0.5;
      double v = -tan(gamma)/(2 * tan(phi)) + 0.5;

      submesh->AddTexCoord(u, v);
    }
  }

  for (unsigned int j = 0; j < (this->h2nd); j++)
    for (unsigned int i = 0; i < (this->w2nd); i++)
    {
      unsigned int a = this->w2nd * j + i;
      submesh->AddIndex(a);
    }

  mesh->AddSubMesh(submesh);

  this->undist_mesh = mesh;

  common::MeshManager::Instance()->AddMesh(this->undist_mesh);
}

void VisualLaser::CreateCanvas()
{
  this->CreateMesh();

  Ogre::Node *parent = this->visual->GetSceneNode()->getParent();
  parent->removeChild(this->visual->GetSceneNode());

  this->pitchNode_ortho->addChild(this->visual->GetSceneNode());
  //this->visual->AttachMesh(this->undist_mesh->GetName());

  this->visual->InsertMesh(this->undist_mesh);

  std::ostringstream stream;
  std::string meshName = this->undist_mesh->GetName();
  //meshName = "unit_box";
  stream << this->visual->GetSceneNode()->getName() << "_ENTITY_" << meshName;

  this->object = (Ogre::MovableObject*) 
      (this->visual->GetSceneNode()->getCreator()->createEntity(stream.str(), meshName));

  this->visual->AttachObject(this->object);
  this->object->setVisibilityFlags(GZ_VISIBILITY_ALL);

  math::Pose pose;
  pose.pos = math::Vector3(0.01,0,0);
  pose.rot.SetFromEuler(math::Vector3(0,0,0));

  this->visual->SetPose(pose);

  this->visual->SetMaterial("Gazebo/Green");
  this->visual->SetAmbient(common::Color(0,1,0,1));
  this->visual->SetVisible(true); 
  this->scene->AddVisual(this->visual);
}
