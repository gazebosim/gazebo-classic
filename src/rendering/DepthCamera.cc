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

#include <sstream>
#include <dirent.h>


#include "sdf/sdf_parser.h"
#include "rendering/ogre.h"
#include "rendering/RTShaderSystem.hh"

#include "common/Events.hh"
#include "common/Console.hh"
#include "common/Exception.hh"
#include "math/Pose.hh"

#include "rendering/Visual.hh"
#include "rendering/Conversions.hh"
#include "rendering/Scene.hh"
#include "rendering/DepthCamera.hh"

using namespace gazebo;
using namespace rendering;


//////////////////////////////////////////////////////////////////////////////
// Constructor
DepthCamera::DepthCamera(const std::string &_namePrefix, Scene *_scene) :
             Camera(_namePrefix, _scene)
{
  this->renderTarget = NULL;
  this->depthBuffer = NULL;
}

//////////////////////////////////////////////////////////////////////////////
// Destructor
DepthCamera::~DepthCamera()
{
}

//////////////////////////////////////////////////////////////////////////////
// Load the camera
void DepthCamera::Load( sdf::ElementPtr &_sdf )
{
  Camera::Load(_sdf);
}

//////////////////////////////////////////////////////////////////////////////
// Load the camera
void DepthCamera::Load()
{
  Camera::Load();
}

//////////////////////////////////////////////////////////////////////////////
// Initialize the camera
void DepthCamera::Init()
{
  Camera::Init();
}

//////////////////////////////////////////////////////////////////////////////
// Finalize the camera
void DepthCamera::Fini()
{
  Camera::Fini();
}

//////////////////////////////////////////////////////////////////////////////
void DepthCamera::CreateDepthTexture( const std::string &_textureName )
{
  // Create the depth buffer
  std::string depthMaterialName = this->GetName() + "_RttMat_Camera_Depth";

  this->renderTexture = Ogre::TextureManager::getSingleton().createManual(
      _textureName,
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
      Ogre::TEX_TYPE_2D,
      this->GetImageWidth(), this->GetImageHeight(), 0,
      Ogre::PF_FLOAT32_R,
      Ogre::TU_RENDERTARGET).getPointer();

  this->renderTarget = this->renderTexture->getBuffer()->getRenderTarget();
  this->renderTarget->setAutoUpdated(false);

  this->SetRenderTarget( this->renderTarget );

  this->viewport->setOverlaysEnabled(false);

  // Create materials for all the render textures.
  Ogre::MaterialPtr matPtr = Ogre::MaterialManager::getSingleton().create(
      depthMaterialName,
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

  matPtr->getTechnique(0)->getPass(0)->setDepthCheckEnabled(false);
  matPtr->getTechnique(0)->getPass(0)->setDepthWriteEnabled(false);
  matPtr->getTechnique(0)->getPass(0)->setLightingEnabled(false);

  matPtr->getTechnique(0)->getPass(0)->createTextureUnitState(
      _textureName );

  this->depthMaterial = (Ogre::Material*)(Ogre::MaterialManager::getSingleton().getByName("Gazebo/DepthMap").getPointer());
  this->depthMaterial->load();
}

////////////////////////////////////////////////////////////////////////////////
// Render the camera
void DepthCamera::Render()
{
  // produce depth data for the camera
  if (!this->newData)
  {
    this->newData = true;
    this->RenderDepthData();
  }
  //this->renderTarget->update(false);
}

////////////////////////////////////////////////////////////////////////////////
void DepthCamera::PostRender()
{
  //this->renderTarget->swapBuffers();

  if (this->newData && this->captureData)
  {
    Ogre::HardwarePixelBufferSharedPtr pixelBuffer;

    size_t size;

    // Get access to the buffer and make an image and write it to file
    pixelBuffer = this->renderTexture->getBuffer();

    Ogre::PixelFormat format = pixelBuffer->getFormat();

    sdf::ElementPtr imageElem = this->sdf->GetOrCreateElement("image");
    size = Ogre::PixelUtil::getMemorySize(
        imageElem->GetValueInt("width"),
        imageElem->GetValueInt("height"), 
        1, 
        format);


    // Blit the depth buffer if needed
    if (!this->depthBuffer)
      this->depthBuffer = new float[size];

    pixelBuffer = this->renderTexture->getBuffer(0, 0);
    pixelBuffer->lock(Ogre::HardwarePixelBuffer::HBL_NORMAL);
    Ogre::Box src_box(0,0,this->GetImageWidth(), this->GetImageHeight());
    Ogre::PixelBox dst_box(this->GetImageWidth(), this->GetImageHeight(),
        1, Ogre::PF_FLOAT32_R, this->depthBuffer);

    pixelBuffer->blitToMemory(src_box, dst_box);
    pixelBuffer->unlock();  // FIXME: do we need to lock/unlock still?

    // Update the last captured render time
    //this->lastRenderTime = this->lastRenderTimeNotCaptured;
   
    float min = 1000;
    float max = 0; 
    for (unsigned int i=0; i < size; i++)
    {
      if (this->depthBuffer[i] > max)
        max = this->depthBuffer[i];
      if (this->depthBuffer[i] < min)
        min = this->depthBuffer[i];
    }

    std::cout << "Min[" << min << "] Max[" << max << "] Mid[" << this->depthBuffer[size/2] << "]\n";
  }
  this->newData = false;
}

//////////////////////////////////////////////////////////////////////////////
// Simulate Depth Data
void DepthCamera::RenderDepthData()
{
  Ogre::RenderSystem *renderSys = this->scene->GetManager()->getDestinationRenderSystem();
  Ogre::Viewport *vp = NULL;
  Ogre::SceneManager *sceneMgr = this->scene->GetManager();
  Ogre::Pass *pass;

  sceneMgr->_suppressRenderStateChanges(true);

  // Get pointer to the material pass
  pass = this->depthMaterial->getBestTechnique()->getPass(0);

  // Render the depth texture
  // OgreSceneManager::_render function automatically sets farClip to 0.
  // Which normally equates to infinite distance. We don't want this. So
  // we have to set the distance every time.
  this->GetOgreCamera()->setFarClipDistance( this->GetFarClip() );

  Ogre::AutoParamDataSource autoParamDataSource;

  vp = this->renderTarget->getViewport(0);

  // return 0 in case no renderable object is inside frustrum
  vp->setBackgroundColour( Ogre::ColourValue(Ogre::ColourValue(0,0,0)) );

  Ogre::CompositorManager::getSingleton().setCompositorEnabled(vp, "Gazebo/DepthMap", true);

  // Need this line to render the ground plane. No idea why it's necessary.
  renderSys->_setViewport(vp);
  sceneMgr->_setPass(pass, true, false);
  autoParamDataSource.setCurrentPass(pass);
  autoParamDataSource.setCurrentViewport(vp);
  autoParamDataSource.setCurrentRenderTarget(this->renderTarget);
  autoParamDataSource.setCurrentSceneManager(sceneMgr);
  autoParamDataSource.setCurrentCamera(this->GetOgreCamera(), true);

#if OGRE_VERSION_MAJOR == 1 && OGRE_VERSION_MINOR == 6
  pass->_updateAutoParamsNoLights(&autoParamDataSource);
#else
  pass->_updateAutoParams(&autoParamDataSource,1);
#endif

  renderSys->setLightingEnabled(false);
  renderSys->_setFog(Ogre::FOG_NONE);

  // These two lines don't seem to do anything useful
  renderSys->_setProjectionMatrix(this->GetOgreCamera()->getProjectionMatrixRS());
  renderSys->_setViewMatrix(this->GetOgreCamera()->getViewMatrix(true));

  // NOTE: We MUST bind parameters AFTER updating the autos
  if (pass->hasVertexProgram())
  {
    renderSys->bindGpuProgram(
    pass->getVertexProgram()->_getBindingDelegate() );

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
    pass->getFragmentProgram()->_getBindingDelegate() );

#if OGRE_VERSION_MAJOR == 1 && OGRE_VERSION_MINOR == 6
    renderSys->bindGpuProgramParameters(Ogre::GPT_FRAGMENT_PROGRAM,
    pass->getFragmentProgramParameters());
#else
      renderSys->bindGpuProgramParameters(Ogre::GPT_FRAGMENT_PROGRAM,
      pass->getFragmentProgramParameters(), 1);
#endif
  }

  // Does actual rendering
  this->renderTarget->update(false);

  sceneMgr->_suppressRenderStateChanges(false);
}

//////////////////////////////////////////////////////////////////////////////
/// Get a pointer to the image data
const float* DepthCamera::GetDepthData()
{
  return this->depthBuffer;
}
