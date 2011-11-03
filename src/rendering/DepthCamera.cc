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
DepthCamera::DepthCamera(const std::string &_namePrefix, Scene *_scene, bool _autoRender) :
             Camera(_namePrefix, _scene, _autoRender)
{
  this->depthTarget = NULL;
  this->depthBuffer = NULL;
  this->depthMaterial = NULL;
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

  this->depthTexture = Ogre::TextureManager::getSingleton().createManual(
      _textureName,
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
      Ogre::TEX_TYPE_2D,
      this->GetImageWidth(), this->GetImageHeight(), 0,
      Ogre::PF_FLOAT32_R,
      Ogre::TU_RENDERTARGET).getPointer();

  this->depthTarget = this->depthTexture->getBuffer()->getRenderTarget();
  this->depthTarget->setAutoUpdated(false);

  this->SetDepthTarget( this->depthTarget );

  this->viewport->setOverlaysEnabled(false);
  this->viewport->setBackgroundColour(Ogre::ColourValue::Black);

  // Create materials for all the render textures.
  Ogre::MaterialPtr matPtr = Ogre::MaterialManager::getSingleton().create(
      depthMaterialName,
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

  matPtr->getTechnique(0)->getPass(0)->setDepthCheckEnabled(false);
  matPtr->getTechnique(0)->getPass(0)->setDepthWriteEnabled(false);
  matPtr->getTechnique(0)->getPass(0)->setLightingEnabled(false);

  matPtr->getTechnique(0)->getPass(0)->createTextureUnitState(
      _textureName );

  this->depthMaterial = (Ogre::Material*)(
      Ogre::MaterialManager::getSingleton().getByName(
        "Gazebo/DepthMap").getPointer());

  this->depthMaterial->load();

/*
  // Create a custom render queue invocation sequence for the depth 
  // render texture
  Ogre::RenderQueueInvocationSequence* invocationSequence =
    Ogre::Root::getSingleton().createRenderQueueInvocationSequence(_textureName + "_DepthMap");

  // Add a render queue invocation to the sequence, and disable shadows for it
  Ogre::RenderQueueInvocation* invocation = 
    invocationSequence->add(Ogre::RENDER_QUEUE_MAIN, _textureName + "_main");
  invocation->setSuppressShadows(true);

  // Set the render queue invocation sequence for the depth render texture 
  // viewport
  this->viewport->setRenderQueueInvocationSequenceName(_textureName + "_DepthMap");
*/
}

////////////////////////////////////////////////////////////////////////////////
void DepthCamera::PostRender()
{
  this->depthTarget->swapBuffers();

  if (this->newData && this->captureData)
  {
    Ogre::HardwarePixelBufferSharedPtr pixelBuffer;

    unsigned int width = this->GetImageWidth();
    unsigned int height = this->GetImageHeight();

    // Get access to the buffer and make an image and write it to file
    pixelBuffer = this->depthTexture->getBuffer();

    Ogre::PixelFormat format = pixelBuffer->getFormat();

    // Blit the depth buffer if needed
    if (!this->depthBuffer)
      this->depthBuffer = new float[width*height];

    Ogre::Box src_box(0,0,width, height);
    Ogre::PixelBox dst_box(width, height,
        1, format, this->depthBuffer);

    pixelBuffer->lock(Ogre::HardwarePixelBuffer::HBL_NORMAL);
    pixelBuffer->blitToMemory(src_box, dst_box);
    pixelBuffer->unlock();  // FIXME: do we need to lock/unlock still?

    this->newDepthFrame( this->depthBuffer, width, height, 1, "FLOAT32");
  }

  // also new image frame for camera texture
  Camera::PostRender();

  this->newData = false;
}

//////////////////////////////////////////////////////////////////////////////
// Simulate Depth Data
void DepthCamera::RenderImpl()
{
  Ogre::RenderSystem *renderSys;
  Ogre::Viewport *vp = NULL;
  Ogre::SceneManager *sceneMgr = this->scene->GetManager();
  Ogre::Pass *pass;

  renderSys = this->scene->GetManager()->getDestinationRenderSystem();
  sceneMgr->setShadowTechnique(Ogre::SHADOWTYPE_NONE);
  sceneMgr->_suppressRenderStateChanges(true);

  // Get pointer to the material pass
  pass = this->depthMaterial->getBestTechnique()->getPass(0);

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
  this->depthTarget->update(false);

  sceneMgr->_suppressRenderStateChanges(false);
  sceneMgr->setShadowTechnique(Ogre::SHADOWTYPE_TEXTURE_MODULATIVE_INTEGRATED);

  // for camera image
  Camera::RenderImpl();
}

//////////////////////////////////////////////////////////////////////////////
/// Get a pointer to the image data
const float* DepthCamera::GetDepthData()
{
  return this->depthBuffer;
}

////////////////////////////////////////////////////////////////////////////////
// Set the render target for the camera
void DepthCamera::SetDepthTarget( Ogre::RenderTarget *target )
{
  this->depthTarget = target;

  if (this->depthTarget)
  {
    // Setup the viewport to use the texture
    this->viewport = this->depthTarget->addViewport(this->camera);
    this->viewport->setClearEveryFrame(true);
    this->viewport->setBackgroundColour( Conversions::Convert( this->scene->GetBackgroundColor() ) );
    this->viewport->setVisibilityMask(GZ_VISIBILITY_ALL & ~GZ_VISIBILITY_GUI);

    double ratio = (double)this->viewport->getActualWidth() / 
                   (double)this->viewport->getActualHeight();

    double hfov = this->GetHFOV().GetAsRadian();
    double vfov = 2.0 * atan(tan( hfov / 2.0) / ratio);
    //gzerr << "debug " << hfov << " " << vfov << " " << ratio << "\n";
    this->camera->setAspectRatio(ratio);
    this->camera->setFOVy(Ogre::Radian(vfov));
  }
}

