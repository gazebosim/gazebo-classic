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
  this->laserTarget = NULL;
  this->laserBuffer = NULL;
  this->laserMaterial = NULL;
  this->w2nd = 0;
  this->h2nd = 0;
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
}

//////////////////////////////////////////////////
void VisualLaser::Fini()
{
  Camera::Fini();
}

//////////////////////////////////////////////////
void VisualLaser::CreateLaserTexture(const std::string &_textureName)
{
  // Create the depth buffer
  std::string laserMaterialName = this->GetName() + "_RttMat_Camera_Laser";
  
  std::cerr<<"before texture create "<<this->w2nd<<" "<<this->h2nd<<"\n";
  this->laserTexture = Ogre::TextureManager::getSingleton().createManual(
      _textureName,
      "General",
      Ogre::TEX_TYPE_2D,
      this->GetImageWidth(), this->GetImageHeight(), 0,
      Ogre::PF_BYTE_RGB,
      Ogre::TU_RENDERTARGET).getPointer();
  std::cerr<<"after texture create "<<this->w2nd<<" "<<this->h2nd<<"\n";

  this->SetLaserTarget(this->laserTexture->getBuffer()->getRenderTarget());
  RTShaderSystem::AttachViewport(this->laserViewport, this->GetScene());
  this->laserTarget->setAutoUpdated(false);

  this->laserMaterial = (Ogre::Material*)(
      Ogre::MaterialManager::getSingleton().getByName("Gazebo/LaserScan").getPointer());

  this->laserMaterial->load();

  this->CreateMesh();
}

//////////////////////////////////////////////////
void VisualLaser::PostRender()
{
  if (this->newData && this->captureData)
  {
    Ogre::HardwarePixelBufferSharedPtr pixelBuffer;

    unsigned int width = this->GetImageWidth();
    unsigned int height = this->GetImageHeight();

    // Get access to the buffer and make an image and write it to file
    pixelBuffer = this->laserTexture->getBuffer();

    size_t size = Ogre::PixelUtil::getMemorySize(width, height, 1, Ogre::PF_BYTE_RGB);

    // Blit the depth buffer if needed
    if (!this->laserBuffer)
      this->laserBuffer = new unsigned char[size];
    
    memset(this->laserBuffer, 255, size);

    Ogre::PixelBox dst_box(width, height,
        1, Ogre::PF_BYTE_RGB, this->laserBuffer);

    pixelBuffer->blitToMemory(dst_box);

    //this->newLaserFrame(this->laserBuffer, width, height, 1, "FLOAT32");
    this->newImageFrame(this->laserBuffer, width, height, 3,"BLABLA");
  }

  // also new image frame for camera texture
  /*Camera::PostRender();*/
    
  this->newData = false;
}

void VisualLaser::UpdateRenderTarget(Ogre::RenderTarget *target, Ogre::Material *material, std::string matName)
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
  this->orthoCam->setFarClipDistance(this->GetFarClip());

  Ogre::AutoParamDataSource autoParamDataSource;

  vp = target->getViewport(0);

  // return 0 in case no renderable object is inside frustrum
  vp->setBackgroundColour(Ogre::ColourValue(Ogre::ColourValue(0, 0, 0)));

  Ogre::CompositorManager::getSingleton().setCompositorEnabled(vp, matName, true);

  // Need this line to render the ground plane. No idea why it's necessary.
  renderSys->_setViewport(vp);
  sceneMgr->_setPass(pass, true, false);
  autoParamDataSource.setCurrentPass(pass);
  autoParamDataSource.setCurrentViewport(vp);
  autoParamDataSource.setCurrentRenderTarget(target);
  autoParamDataSource.setCurrentSceneManager(sceneMgr);
  autoParamDataSource.setCurrentCamera(this->orthoCam, true);

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

  this->UpdateRenderTarget(this->laserTarget, this->laserMaterial, "Gazebo/LaserScan");
  
  // Does actual rendering
  this->laserTarget->update(false);

  sceneMgr->_suppressRenderStateChanges(false);
  sceneMgr->setShadowTechnique(Ogre::SHADOWTYPE_TEXTURE_MODULATIVE_INTEGRATED);

  // for camera image
  /*Camera::RenderImpl();*/
}

//////////////////////////////////////////////////
const unsigned char* VisualLaser::GetLaserData()
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
  
  sdf::ElementPtr clipElem = this->sdf->GetOrCreateElement("clip");
  
  if (this->orthoCam)
  {
    this->orthoCam->setNearClipDistance(clipElem->GetValueDouble("near"));
    this->orthoCam->setFarClipDistance(clipElem->GetValueDouble("far"));
    this->orthoCam->setRenderingDistance(clipElem->GetValueDouble("far"));
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
void VisualLaser::SetLaserTarget(Ogre::RenderTarget *target)
{
  this->laserTarget = target;

  this->CreateOrthoCam();
  
  if (this->laserTarget)
  {
    // Setup the viewport to use the texture
    this->laserViewport = this->laserTarget->addViewport(this->orthoCam);
    this->laserViewport->setClearEveryFrame(true);
    this->laserViewport->setBackgroundColour(Ogre::ColourValue(0.0,0.0,0.0));
    this->laserViewport->setVisibilityMask(
        GZ_VISIBILITY_ALL & ~GZ_VISIBILITY_GUI);

    double ratio = static_cast<double>(this->laserViewport->getActualWidth()) /
                   static_cast<double>(this->laserViewport->getActualHeight());
    
    this->orthoCam->setProjectionType(Ogre::PT_ORTHOGRAPHIC);
    this->orthoCam->setAspectRatio(ratio);
    this->orthoCam->setOrthoWindowHeight(3/*this->laserViewport->getActualHeight()*/);
  }
}

void VisualLaser::SetRangeCount(unsigned int _w, unsigned int _h)
{
  this->w2nd = _w;
  this->h2nd = _h;
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

  for (unsigned int j = 0; j < this->h2nd; j++)
    for (unsigned int i = 0; i < this->w2nd; i++)
    {
      submesh->AddVertex(start_x + (dx * i), start_y + (dy * j), 0);
      //std::cerr<<start_x + (dx * i)<<" "<<start_y + (dy * j)<<"\n";
    }
}

