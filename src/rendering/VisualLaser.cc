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
      this->w2nd, this->h2nd, 0,
      Ogre::PF_FLOAT32_RGB,
      Ogre::TU_RENDERTARGET).getPointer();

  this->_2ndTexture_dbg = Ogre::TextureManager::getSingleton().createManual(
      _textureName + "_2nd_pass",
      "General",
      Ogre::TEX_TYPE_2D,
      this->w2nd, this->h2nd, 0,
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

  this->mat_2nd_pass_dbg = (Ogre::Material*)(
      Ogre::MaterialManager::getSingleton().getByName("Gazebo/LaserScan2ndDBG").getPointer());
  this->mat_2nd_pass_dbg->load();
  tex_unit = this->mat_2nd_pass_dbg->getTechnique(0)->getPass(0)->createTextureUnitState(this->_1stTexture_dbg->getName());
  tex_unit->setTextureFiltering(Ogre::TFO_NONE);

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

    this->newLaserFrame(this->laserBuffer, width, height, 3,"BLABLA");
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

  this->newImageFrame(this->imageBuffer, width, height, 3, "BLABLA");
}

void VisualLaser::UpdateRenderTarget(Ogre::RenderTarget *target, Ogre::Material *material, std::string matName, Ogre::Camera *cam)
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

  // return 0 in case no renderable object is inside frustrum
  vp->setBackgroundColour(Ogre::ColourValue(Ogre::ColourValue(0, 0, 1)));

  Ogre::CompositorManager::getSingleton().setCompositorEnabled(vp, matName, true);

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

//////////////////////////////////////////////////
void VisualLaser::RenderImpl()
{
  Ogre::SceneManager *sceneMgr = this->scene->GetManager();

  sceneMgr->setShadowTechnique(Ogre::SHADOWTYPE_NONE);
  sceneMgr->_suppressRenderStateChanges(true);

  this->UpdateRenderTarget(this->_1stTarget, this->mat_1st_pass, "Gazebo/LaserScan1st", this->camera);
  this->_1stTarget->update(false);
  
  this->UpdateRenderTarget(this->_1stTarget_dbg, this->mat_1st_pass_dbg, "Gazebo/LaserScan1stDBG", this->camera);
  this->_1stTarget_dbg->update(false);

  this->visual->SetVisible(true);
  
  this->UpdateRenderTarget(this->_2ndTarget, this->mat_2nd_pass, "Gazebo/LaserScan2nd", this->orthoCam);
  this->_2ndTarget->update(false);
  
  this->UpdateRenderTarget(this->_2ndTarget_dbg, this->mat_2nd_pass_dbg, "Gazebo/LaserScan2ndDBG", this->orthoCam);
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
    this->_1stViewport->setBackgroundColour(Ogre::ColourValue(0.0,0.0,0.0));
    this->_1stViewport->setVisibilityMask(
        GZ_VISIBILITY_ALL & ~GZ_VISIBILITY_GUI);
    this->_1stViewport_dbg = this->_1stTarget_dbg->addViewport(this->camera);
    this->_1stViewport_dbg->setClearEveryFrame(true);
    this->_1stViewport_dbg->setBackgroundColour(Ogre::ColourValue(0.0,0.0,0.0));
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
  double start_x, start_y;
  
  if (h2nd == 1)
  {
    submesh->SetPrimitiveType(common::SubMesh::LINESTRIPS);
    dy = 0;
    start_y = 0;
  }
  else
  {
    submesh->SetPrimitiveType(common::SubMesh::TRIANGLES); 
    dy = (2.0 * this->h2nd / this->w2nd) / (this->h2nd -1);
    start_y = -(2.0 * this->h2nd / this->w2nd) / 2;
  }

  dx = 2.0 / (this->w2nd - 1);

  start_x = -1;

  math::Angle minHAngle = this->parent_sensor->GetAngleMin();
  math::Angle maxHAngle = this->parent_sensor->GetAngleMax();
  math::Angle minVAngle = this->parent_sensor->GetVerticalAngleMin();
  math::Angle maxVAngle = this->parent_sensor->GetVerticalAngleMax();
    
  double hfov = maxHAngle.GetAsRadian() - minHAngle.GetAsRadian();
  double vfov = maxVAngle.GetAsRadian() - minVAngle.GetAsRadian();

  double minx = 100;
  double maxx = -100;
  
  double miny = 100;
  double maxy = -100;
  
  for (unsigned int j = 0; j < this->h2nd; j++)
  {
    double fi_y = 0;
    if (this->h2nd != 1)
      fi_y = (vfov / (this->h2nd - 1)) * j;
    for (unsigned int i = 0; i < this->w2nd; i++)
    {
      submesh->AddVertex(0,start_x + (dx * i), start_y + (dy * j));

      if (start_x + (dx * i) > maxx)
        maxx = start_x + (dx * i);
      if (start_x + (dx * i) < minx)
        minx = start_x + (dx * i);

      if (start_y + (dy * j) > maxy)
        maxy = start_y + (dy * j);
      if (start_y + (dy * j) < miny)
        miny = start_y + (dy * j);
      
      double fi_x = (hfov / (this->w2nd - 1)) * i;
      double u = (tan(fi_x - hfov / 2) / tan(hfov / 2) + 1) / 2;
    //  std::cerr<<"fi_x: "<<fi_x<<" hfov/2:"<<hfov/2<<" u: "<<u<<"\n";
      double v = 0;
      if (this->h2nd != 1)
        v = (tan(fi_y - vfov / 2) / tan(vfov / 2) + 1) / 2;
      submesh->AddTexCoord(1-u,1-v);
    }
    //std::cerr<<"________________________________\n";
  }

  Ogre::Matrix4 p = this->BuildScaledOrthoMatrix(minx, maxx, miny, maxy, 0.01, 0.02);

  this->orthoCam->setCustomProjectionMatrix(true, p);

  if (this->h2nd == 1)
  {
    /// mesh is just one line strip
    for (unsigned int i = 0; i < this->w2nd; i++)
      submesh->AddIndex(i);
  }
  else
  {
    for (unsigned int j = 0; j < (this->h2nd-1); j++)
      for (unsigned int i = 0; i < (this->w2nd-1); i++)
      {
        unsigned int a = this->w2nd * j + i;
        unsigned int b = a + 1;
        unsigned int c = b + this->w2nd;
        unsigned int d = c - 1;
        
        //1st triangle
        submesh->AddIndex(d);
        submesh->AddIndex(b);
        submesh->AddIndex(a);
        
        //2nd triangle
        submesh->AddIndex(d);
        submesh->AddIndex(c);
        submesh->AddIndex(b);
      }
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
  this->visual->SetVisible(false); 
  this->scene->AddVisual(this->visual);
}
