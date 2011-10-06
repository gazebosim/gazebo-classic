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
#include "common/Global.hh"
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

  this->depthTarget = NULL;
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
  this->sdf = _sdf;
  this->Load();
}

//////////////////////////////////////////////////////////////////////////////
// Load the camera
void DepthCamera::Load()
{
  Camera::Load();
}

void DepthCamera::CreateDepthTexture()
{

  // Create the depth buffer
  this->depthTextureName = this->GetName() + "_RttTex_Camera_Depth";
  this->depthMaterialName = this->GetName() + "_RttMat_Camera_Depth";

  this->depthTexture = Ogre::TextureManager::getSingleton().createManual(
      depthTextureName,
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
      Ogre::TEX_TYPE_2D,
      this->GetImageWidth(), this->GetImageHeight(), 0,
      Ogre::PF_FLOAT32_R,
      Ogre::TU_RENDERTARGET);

  this->depthTarget = this->depthTexture->getBuffer()->getRenderTarget();

  this->depthTarget->setAutoUpdated(false);
  //
  // Setup the viewport to use the texture
  Ogre::Viewport *cviewport;
  Ogre::MaterialPtr matPtr;
  cviewport = this->depthTarget->addViewport(this->camera);
  //cviewport->setClearEveryFrame(true);
  cviewport->setOverlaysEnabled(false);
  cviewport->setBackgroundColour( Conversions::Convert( this->scene->GetBackgroundColor() ) );


  // TODO: reimplement Visibility Mast
  // cviewport->setVisibilityMask(this->visibilityMask);


// Create materials for all the render textures.
  matPtr = Ogre::MaterialManager::getSingleton().create(
      depthMaterialName,
  Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  matPtr->getTechnique(0)->getPass(0)->setDepthCheckEnabled(false);
  matPtr->getTechnique(0)->getPass(0)->setDepthWriteEnabled(false);
  matPtr->getTechnique(0)->getPass(0)->setLightingEnabled(false);

  matPtr->getTechnique(0)->getPass(0)->createTextureUnitState(
      depthTextureName );

  //this->depthMaterial = Ogre::MaterialManager::getSingleton().getByName(depthMaterialName);
  this->depthMaterial = Ogre::MaterialManager::getSingleton().getByName("Gazebo/DepthMap");
  this->depthMaterial->load();

}

 
////////////////////////////////////////////////////////////////////////////////
void DepthCamera::PostRender()
{

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
    if (!this->saveDepthBuffer)
      this->saveDepthBuffer = new float[size];

    pixelBuffer = this->depthTexture->getBuffer(0, 0);
    pixelBuffer->lock(Ogre::HardwarePixelBuffer::HBL_NORMAL);
    Ogre::Box src_box(0,0,this->GetImageWidth(), this->GetImageHeight());
    Ogre::PixelBox dst_box(this->GetImageWidth(), this->GetImageHeight(),
        1, Ogre::PF_FLOAT32_R, this->saveDepthBuffer);

    pixelBuffer->blitToMemory(src_box, dst_box);
    pixelBuffer->unlock();  // FIXME: do we need to lock/unlock still?

    // Update the last captured render time
    //this->lastRenderTime = this->lastRenderTimeNotCaptured;

  }
  this->newData = false;
}

////////////////////////////////////////////////////////////////////////////////
// Render the camera
void DepthCamera::Render()
{
  // produce depth data for the camera
  this->RenderDepthData();
  //this->depthTarget->update(false);
}
//////////////////////////////////////////////////////////////////////////////
// Simulate Depth Data
void DepthCamera::RenderDepthData()
{
  Ogre::RenderSystem *renderSys = this->scene->GetManager()->getDestinationRenderSystem();
  Ogre::Viewport *vp = NULL;
  Ogre::SceneManager *sceneMgr = this->scene->GetManager();
  Ogre::Pass *pass;
  Ogre::SceneNode *gridNode = NULL;

  try
  {
    gridNode = sceneMgr->getSceneNode("__OGRE_GRID_NODE__");
  }
  catch (...)
  {
    gridNode = NULL;
  }

  sceneMgr->_suppressRenderStateChanges(true);

  // Get pointer to the material pass
  pass = this->depthMaterial->getBestTechnique()->getPass(0);

  if (gridNode)
    gridNode->setVisible(false);

  // Render the depth texture
  // OgreSceneManager::_render function automatically sets farClip to 0.
  // Which normally equates to infinite distance. We don't want this. So
  // we have to set the distance every time.
  this->GetOgreCamera()->setFarClipDistance( this->GetFarClip() );

  Ogre::AutoParamDataSource autoParamDataSource;

  vp = this->depthTarget->getViewport(0);

  // return 0 in case no renderable object is inside frustrum
  vp->setBackgroundColour( Ogre::ColourValue(Ogre::ColourValue(0,0,0)) );

  Ogre::CompositorManager::getSingleton().setCompositorEnabled(vp, "Gazebo/DepthMap", true);

  // Need this line to render the ground plane. No idea why it's necessary.
  renderSys->_setViewport(vp);
  sceneMgr->_setPass(pass, true, false);
  autoParamDataSource.setCurrentPass(pass);
  autoParamDataSource.setCurrentViewport(vp);
  autoParamDataSource.setCurrentRenderTarget(this->depthTarget);
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
  this->depthTarget->update();

  sceneMgr->_suppressRenderStateChanges(false);

  if (gridNode)
    gridNode->setVisible(true);
}
//////////////////////////////////////////////////////////////////////////////
/// Get a pointer to the image data
const float* DepthCamera::GetDepthData()
{
  return this->saveDepthBuffer;
}


//////////////////////////////////////////////////////////////////////////////
// Initialize the camera
void DepthCamera::Init()
{

  this->CreateDepthTexture();
}

//////////////////////////////////////////////////////////////////////////////
// Finalize the camera
void DepthCamera::Fini()
{
}

