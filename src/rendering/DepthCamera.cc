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


unsigned int DepthCamera::cameraCounter = 0;

//////////////////////////////////////////////////////////////////////////////
// Constructor
DepthCamera::DepthCamera(const std::string &namePrefix_, Scene *scene_) :
             Camera(namePrefix_, scene_)
{
  this->sdf.reset(new sdf::Element);
  sdf::initFile( "/sdf/camera.sdf", this->sdf );

  this->windowId = 0;
  this->scene = scene_;

  this->textureWidth = this->textureHeight = 0;

  this->saveFrameBuffer = NULL;
  this->saveCount = 0;
  this->bayerFrameBuffer = NULL;

  this->myCount = cameraCounter++;

  std::ostringstream stream;
  stream << namePrefix_ << "(" << this->myCount << ")";
  this->name = stream.str();

  this->renderTarget = NULL;
  this->userMovable = true;

  this->captureData = false;

  this->camera = NULL;
  this->viewport = NULL;

  this->renderPeriod = common::Time(0.0);

  this->renderingEnabled = true;

  this->pitchNode = NULL;
  this->sceneNode = NULL;
  this->origParentNode = NULL;

  // Connect to the render signal
  this->connections.push_back( event::Events::ConnectPreRenderSignal( boost::bind(&DepthCamera::Update, this) ) );
  this->connections.push_back( event::Events::ConnectRenderSignal( boost::bind(&DepthCamera::Render, this) ) );
  this->connections.push_back( event::Events::ConnectPostRenderSignal( boost::bind(&DepthCamera::PostRender, this) ) );
  this->connections.push_back( event::Events::ConnectShowWireframeSignal( boost::bind(&DepthCamera::ToggleShowWireframe, this) ));
}

//////////////////////////////////////////////////////////////////////////////
// Destructor
DepthCamera::~DepthCamera()
{
  if (this->saveFrameBuffer)
    delete [] this->saveFrameBuffer;

  if (this->bayerFrameBuffer)
    delete [] this->bayerFrameBuffer;

  if (this->pitchNode)
  {
    this->sceneNode->removeAndDestroyChild( this->name + "PitchNode");
    this->pitchNode = NULL;
  }

  if (this->camera)
  {
    this->scene->GetManager()->destroyCamera(this->name);
    this->camera = NULL;
  }
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

  sdf::ElementPtr imageElem = this->sdf->GetOrCreateElement("image");

  std::string imgFmt = imageElem->GetValueString("format");

  if (imgFmt == "L8")
    this->imageFormat = (int)Ogre::PF_L8;
  else if (imgFmt == "R8G8B8")
    this->imageFormat = (int)Ogre::PF_BYTE_RGB;
  else if (imgFmt == "B8G8R8")
    this->imageFormat = (int)Ogre::PF_BYTE_BGR;
  else if (imgFmt == "FLOAT32")
    this->imageFormat = (int)Ogre::PF_FLOAT32_R;
  else if (imgFmt == "FLOAT16")
    this->imageFormat = (int)Ogre::PF_FLOAT16_R;
  else if ( (imgFmt == "BAYER_RGGB8") ||
            (imgFmt == "BAYER_BGGR8") ||
            (imgFmt == "BAYER_GBRG8") ||
            (imgFmt == "BAYER_GRBG8") )
  {
    // let ogre generate rgb8 images for all bayer format requests
    // then post process to produce actual bayer images
    this->imageFormat = (int)Ogre::PF_BYTE_RGB;
  }
  else
  {
    gzerr << "Error parsing image format (" << imgFmt << "), using default Ogre::PF_R8G8B8\n";
    this->imageFormat = (int)Ogre::PF_R8G8B8;
  }

  // Create the directory to store frames
  if (this->sdf->HasElement("save") && 
      this->sdf->GetElement("save")->GetValueBool("enabled"))
  {
    sdf::ElementPtr elem = this->sdf->GetElement("save");
    std::string command;

    command = "mkdir " + elem->GetValueString("path")+ " 2>>/dev/null";
    if (system(command.c_str()) < 0)
      gzerr << "Error making directory\n";
  }

  if (this->sdf->HasElement("horizontal_fov"))
  {
    sdf::ElementPtr elem = this->sdf->GetElement("horizontal_fov");
    double angle = elem->GetValueDouble("angle");
    if (angle < 0.01 || angle > M_PI)
    {
      gzthrow("DepthCamera horizontal field of veiw invalid.");
    }
  }



}

void DepthCamera::CreateDepthTexture( const std::string &textureName )
{

  // Create the depth buffer
  this->depthTextureName = this->GetName() + "_RttTex_Camera_Depth";
  this->depthMaterialName = this->GetName() + "_RttMat_Camera_Depth";

  depthTexture = this->CreateRTT(depthTextureName, true);
  depthTarget = depthTexture->getBuffer()->getRenderTarget();
  depthTarget->setAutoUpdated(false);
  //
  // Setup the viewport to use the texture
  Ogre::Viewport *cviewport;
  Ogre::MaterialPtr matPtr;
  cviewport = this->depthTarget->addViewport(this->camera);
  //cviewport->setClearEveryFrame(true);
  cviewport->setOverlaysEnabled(false);
  cviewport->setBackgroundColour( *OgreAdaptor::Instance()->backgroundColor );
  cviewport->setVisibilityMask(this->visibilityMask);


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

Ogre::TexturePtr DepthCamera::CreateRTT(const std::string &name, bool depth)
{
  Ogre::PixelFormat pf;

  if (depth)
    pf = Ogre::PF_FLOAT32_R;
  else
    pf = Ogre::PF_BYTE_RGB;

  // Create the depth render texture
  return Ogre::TextureManager::getSingleton().createManual(
      name,
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
      Ogre::TEX_TYPE_2D,
      this->imageSizeP->GetValue().x, this->imageSizeP->GetValue().y, 0,
      pf,
      Ogre::TU_RENDERTARGET);
}

 
////////////////////////////////////////////////////////////////////////////////
void DepthCamera::PostRender()
{
  this->renderTarget->swapBuffers();

  if (this->newData && this->captureData)
  {
    Ogre::HardwarePixelBufferSharedPtr pixelBuffer;
    Ogre::RenderTexture *rTexture;
    Ogre::Viewport* renderViewport;

    size_t size;

    // Get access to the buffer and make an image and write it to file
    pixelBuffer = this->renderTexture->getBuffer();
    rTexture = pixelBuffer->getRenderTarget();

    Ogre::PixelFormat format = pixelBuffer->getFormat();
    renderViewport = rTexture->getViewport(0);

    sdf::ElementPtr imageElem = this->sdf->GetOrCreateElement("image");
    size = Ogre::PixelUtil::getMemorySize(
        imageElem->GetValueInt("width"),
        imageElem->GetValueInt("height"), 
        1, 
        format);

    // Allocate buffer
    if (!this->saveFrameBuffer)
      this->saveFrameBuffer = new unsigned char[size];

    memset(this->saveFrameBuffer,128,size);

    Ogre::PixelBox box(imageElem->GetValueInt("width"), 
                       imageElem->GetValueInt("height"),
        1, (Ogre::PixelFormat)this->imageFormat, this->saveFrameBuffer);

    pixelBuffer->blitToMemory( box );

    if (this->sdf->HasElement("save") && 
        this->sdf->GetElement("save")->GetValueBool("enabled"))
    {
      this->SaveFrame();
    }


    // Blit the depth buffer if needed
    if (this->simulateDepthData)
    {
        if (!this->saveDepthBuffer)
          this->saveDepthBuffer = new float[size];

        pixelBuffer = this->depthTexture->getBuffer(0, 0);
        pixelBuffer->lock(Ogre::HardwarePixelBuffer::HBL_NORMAL);
      Ogre::PixelBox dpt_box((**this->imageSizeP).x, (**this->imageSizeP).y,
            1, Ogre::PF_FLOAT32_R, this->saveDepthBuffer);

      pixelBuffer->blitToMemory(src_box, dpt_box);
        pixelBuffer->unlock();  // FIXME: do we need to lock/unlock still?
    }

    // Update the last captured render time
    //this->lastRenderTime = this->lastRenderTimeNotCaptured;

  }
  this->newData = false;
}

////////////////////////////////////////////////////////////////////////////////
// Render the camera
void DepthCamera::Render()
{
  this->newData = true;
  this->renderTarget->update(false);
  // produce depth data for the camera
  if (this->simulateDepthData)
          this->RenderDepthData();
  //this->depthTarget->update(false);
}
//////////////////////////////////////////////////////////////////////////////
// Simulate Depth Data
void DepthCamera::RenderDepthData()
{
  OgreAdaptor *adapt = OgreAdaptor::Instance();
  Ogre::RenderSystem *renderSys = adapt->root->getRenderSystem();
  Ogre::Viewport *vp = NULL;
  Ogre::SceneManager *sceneMgr = adapt->sceneMgr;
  Ogre::Pass *pass;
  Ogre::SceneNode *gridNode = NULL;
  int i;

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
  this->GetOgreCamera()->setFarClipDistance( this->farClipP->GetValue() );

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
// Initialize the camera
void DepthCamera::Init()
{
  this->SetSceneNode( this->scene->GetManager()->getRootSceneNode()->createChildSceneNode( this->GetName() + "_SceneNode") );

  this->CreateCamera();

  // Create a scene node to control pitch motion
  this->pitchNode = this->sceneNode->createChildSceneNode( this->name + "PitchNode");
  this->pitchNode->pitch(Ogre::Degree(0));

  this->pitchNode->attachObject(this->camera);
  this->camera->setAutoAspectRatio(true);

  this->saveCount = 0;

  this->origParentNode = (Ogre::SceneNode*)this->sceneNode->getParent();

  this->SetFOV( DTOR(60) );
  this->SetClipDist(0.001, 100);
}

//////////////////////////////////////////////////////////////////////////////
// Finalize the camera
void DepthCamera::Fini()
{
}

//////////////////////////////////////////////////////////////////////////////
// Update the drawing
void DepthCamera::Update()
{
  if (this->sceneNode)
  {
    Ogre::Vector3 v = this->sceneNode->_getDerivedPosition();

    this->pose.pos.x = v.x;
    this->pose.pos.y = v.y;
    this->pose.pos.z = v.z;
  }

  if (this->pitchNode)
  {
    Ogre::Quaternion q = this->pitchNode->_getDerivedOrientation();

    this->pose.rot.w = q.w;
    this->pose.rot.x = q.x;
    this->pose.rot.y = q.y;
    this->pose.rot.z = q.z;
  }
}

