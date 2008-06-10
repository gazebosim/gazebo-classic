/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

/* Desc: Stereo Camera Sensor
 * Author: Nate Koenig
 * Date: 25 March 2008
 * SVN: $Id:$
 */
#include <arpa/inet.h>
#include <sstream>
#include <OgreImageCodec.h>
#include <GL/gl.h>
#include <Ogre.h>

#include "Global.hh"
#include "World.hh"
#include "GazeboError.hh"
#include "Body.hh"
#include "OgreVisual.hh"
#include "OgreCreator.hh"
#include "OgreAdaptor.hh"
#include "OgreFrameListener.hh"

#include "SensorFactory.hh"
#include "CameraManager.hh"
#include "StereoCameraSensor.hh"

#define PF_FLOAT Ogre::PF_FLOAT32_R

using namespace gazebo;

GZ_REGISTER_STATIC_SENSOR("stereocamera", StereoCameraSensor);

//////////////////////////////////////////////////////////////////////////////
// Constructor
StereoCameraSensor::StereoCameraSensor(Body *body)
    : CameraSensor(body)
{
  this->depthBuffer[0] = NULL;
  this->depthBuffer[1] = NULL;
  this->rgbBuffer[0] = NULL;
  this->rgbBuffer[1] = NULL;
}


//////////////////////////////////////////////////////////////////////////////
// Destructor
StereoCameraSensor::~StereoCameraSensor()
{
  for (int i=0; i<2; i++)
  {
    if (this->depthBuffer[i])
      delete [] this->depthBuffer[i];

    if (this->rgbBuffer[i])
      delete [] this->rgbBuffer[i];
  }
}

//////////////////////////////////////////////////////////////////////////////
// Load the camera
void StereoCameraSensor::LoadChild( XMLConfigNode *node )
{
  CameraSensor::LoadChild(node);

  this->baseline = node->GetDouble("baseline",0,1);
}

//////////////////////////////////////////////////////////////////////////////
// Initialize the camera
void StereoCameraSensor::InitChild()
{
  Ogre::Viewport *cviewport;
  Ogre::MaterialPtr matPtr;
  Ogre::HardwarePixelBufferSharedPtr mBuffer;
  int i;

  this->textureName[LEFT] = this->GetName() + "_RttTex_Stereo_Left";
  this->textureName[RIGHT] = this->GetName() + "_RttTex_Stereo_Right";
  this->textureName[D_LEFT] = this->GetName() +"_RttTex_Stereo_Left_Depth";
  this->textureName[D_RIGHT] = this->GetName() +"_RttTex_Stereo_Right_Depth";

  this->materialName[LEFT] = this->GetName() + "_RttMat_Stereo_Left";
  this->materialName[RIGHT] = this->GetName() + "_RttMat_Stereo_Right";
  this->materialName[D_LEFT] = this->GetName()+"_RttMat_Stereo_Left_Depth";
  this->materialName[D_RIGHT] = this->GetName()+"_RttMat_Stereo_Right_Depth";

  // Create the render textures for the color textures
  for (i = 0; i<2; i++)
  {
    this->renderTexture[i] = this->CreateRTT(this->textureName[i], false);
    this->renderTarget[i] = this->renderTexture[i]->getBuffer()->getRenderTarget();
    this->renderTarget[i]->setAutoUpdated(false);
  }

  // Create the render texture for the depth textures
  for (i = 2; i<4; i++)
  {
    this->renderTexture[i] = this->CreateRTT(this->textureName[i], true);
    this->renderTarget[i] = this->renderTexture[i]->getBuffer()->getRenderTarget();
    this->renderTarget[i]->setAutoUpdated(false);
  }

  // Create the camera
  this->camera = OgreCreator::CreateCamera(this->GetName(),
                 this->nearClip, this->farClip, this->hfov, 
                 this->renderTarget[D_LEFT]);

  // Hack to make the camera use the right render target too.
  for (i=0; i<4; i++)
  {
    if (i != D_LEFT )
    {
      // Setup the viewport to use the texture
      cviewport = this->renderTarget[i]->addViewport(this->camera);
      cviewport->setClearEveryFrame(true);
      cviewport->setOverlaysEnabled(false);
      cviewport->setBackgroundColour( *OgreAdaptor::Instance()->backgroundColor );
    }

    // Create materials for all the render textures.
    matPtr = Ogre::MaterialManager::getSingleton().create(
             this->materialName[i],
             Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

    matPtr->getTechnique(0)->getPass(0)->createTextureUnitState(
        this->textureName[i] );
  }

  // Get pointer to the depth map material
  this->depthMaterial = Ogre::MaterialManager::getSingleton().load("Gazebo/DepthMap", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);


  mBuffer = this->renderTexture[D_LEFT]->getBuffer(0,0);
  this->textureWidth = mBuffer->getWidth();
  this->textureHeight = mBuffer->getHeight();


  //this->leftCameraListener.Init(this, this->renderTarget[0], true);
  //this->rightCameraListener.Init(this, this->renderTarget[1], false);

  //this->renderTarget[0]->addListener(&this->leftCameraListener);
  //this->renderTarget[1]->addListener(&this->rightCameraListener);
 
  this->rgbBufferSize = this->imageWidth*this->imageHeight * 3;
  this->depthBufferSize = this->imageWidth*this->imageHeight;

  // Allocate buffers
  this->depthBuffer[0] = new float[this->depthBufferSize];
  this->depthBuffer[1] = new float[this->depthBufferSize];
  this->rgbBuffer[0] = new unsigned char[this->rgbBufferSize];
  this->rgbBuffer[1] = new unsigned char[this->rgbBufferSize];

  CameraSensor::InitChild();
}

//////////////////////////////////////////////////////////////////////////////
// Finalize the camera
void StereoCameraSensor::FiniChild()
{
}

//////////////////////////////////////////////////////////////////////////////
// Update the drawing
void StereoCameraSensor::UpdateChild(UpdateParams &params)
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

  CameraSensor::UpdateChild(params);

  sceneMgr->_suppressRenderStateChanges(true);

  //prev_pass = sceneMgr->getPass();

  // Get pointer to the material pass
  pass = this->depthMaterial->getBestTechnique()->getPass(0);

  if (gridNode)
    gridNode->setVisible(false);

  // Render the depth texture
  for (i=2; i<4; i++)
  {

    // OgreSceneManager::_render function automatically sets farClip to 0.
    // Which normally equates to infinite distance. We don't want this. So
    // we have to set the distance every time.
    this->camera->setFarClipDistance( this->farClip );


    Ogre::AutoParamDataSource autoParamDataSource;

    vp = this->renderTarget[i]->getViewport(0);

    // Need this line to render the ground plane. No idea why it's necessary.
    renderSys->_setViewport(vp);
    sceneMgr->_setPass(pass, true, false); 
    autoParamDataSource.setCurrentPass(pass);
    autoParamDataSource.setCurrentViewport(vp);
    autoParamDataSource.setCurrentRenderTarget(this->renderTarget[i]);
    autoParamDataSource.setCurrentSceneManager(sceneMgr);
    autoParamDataSource.setCurrentCamera(this->camera);
    pass->_updateAutoParamsNoLights(autoParamDataSource);

    // These two lines don't seem to do anything useful
    renderSys->_setProjectionMatrix(this->camera->getProjectionMatrixRS()); 
    renderSys->_setViewMatrix(this->camera->getViewMatrix(true));

    // NOTE: We MUST bind parameters AFTER updating the autos
    if (pass->hasVertexProgram())
    {
      renderSys->bindGpuProgram( pass->getVertexProgram()->_getBindingDelegate() );
      renderSys->bindGpuProgramParameters(Ogre::GPT_VERTEX_PROGRAM,
          pass->getVertexProgramParameters());
    }

    if (pass->hasFragmentProgram())
    {   
      renderSys->bindGpuProgram( pass->getFragmentProgram()->_getBindingDelegate() );
      renderSys->bindGpuProgramParameters(Ogre::GPT_FRAGMENT_PROGRAM, 
          pass->getFragmentProgramParameters());
    }   
    this->renderTarget[i]->update();
  }

  sceneMgr->_suppressRenderStateChanges(false); 

  // Render the image texture
  for (i=0; i<2; i++)
  {
    this->renderTarget[i]->update();
  }

  if (gridNode)
    gridNode->setVisible(true);

  this->FillBuffers();

  if (this->saveFrames)
    this->SaveFrame();
}

////////////////////////////////////////////////////////////////////////////////
// Return the material the camera renders to
std::string StereoCameraSensor::GetMaterialName() const
{
  return this->materialName[LEFT];
}

//////////////////////////////////////////////////////////////////////////////
/// Get a pointer to the image data
const unsigned char *StereoCameraSensor::GetImageData(unsigned int i)
{

  if (i > 1)
    gzthrow("Index must be 0 for left, or 1 for right disparity image\n");

  return this->rgbBuffer[i];
}

//////////////////////////////////////////////////////////////////////////////
// Get a pointer to the disparity data
const float *StereoCameraSensor::GetDisparityData(unsigned int i)
{
  if (i > 1)
    gzthrow("Index must be 0 for left, or 1 for right disparity image\n");

  return this->depthBuffer[i];
}

//////////////////////////////////////////////////////////////////////////////
// Fill all RGB and Disparity buffers
void StereoCameraSensor::FillBuffers()
{
  Ogre::HardwarePixelBufferSharedPtr hardwareBuffer;
  int i;
  int top,left,right,bottom;

  for (i=0; i<4; i++)
  {
    // Get access to the buffer and make an image and write it to file
    hardwareBuffer = this->renderTexture[i]->getBuffer(0, 0);

    hardwareBuffer->lock(Ogre::HardwarePixelBuffer::HBL_NORMAL);

    top = (int)((hardwareBuffer->getHeight() - this->imageHeight) / 2.0);
    left = (int)((hardwareBuffer->getWidth() - this->imageWidth) / 2.0);
    right = left + this->imageWidth;
    bottom = top + this->imageHeight;

    if (i < 2)
    {
      hardwareBuffer->blitToMemory ( Ogre::Box(left,top,right,bottom),
          Ogre::PixelBox( this->imageWidth, this->imageHeight,
            1, Ogre::PF_B8G8R8, this->rgbBuffer[i])
          );
    }
    else
    {
      hardwareBuffer->blitToMemory (Ogre::Box(left,top,right,bottom),
          Ogre::PixelBox( this->imageWidth, this->imageHeight,
            1, PF_FLOAT, this->depthBuffer[i-2])
          );
    }

    hardwareBuffer->unlock();
  }
}

////////////////////////////////////////////////////////////////////////////////
// Save a single frame to disk
void StereoCameraSensor::SaveFrame()
{
  char tmp[1024];
  FILE *fp;
  
  sprintf(tmp, "frame%04d.pgm", this->saveCount);

  fp = fopen( tmp, "wb" );

  if (!fp)
  {
    printf( "unable to open file %s\n for writing", tmp );
    return;
  }

  fprintf( fp, "P6\n# Gazebo\n%d %d\n255\n", this->imageWidth, this->imageHeight);

  for (unsigned int i = 0; i<this->imageHeight; i++)
  {
    for (unsigned int j =0; j<this->imageWidth; j++)
    {
      double f = this->depthBuffer[0][i*this->imageWidth+j];
     
      unsigned char value = static_cast<unsigned char>(f * 255);
      fwrite( &value, 1, 1, fp );
      fwrite( &value, 1, 1, fp );
      fwrite( &value, 1, 1, fp );
    }
  }

  fclose( fp );
  this->saveCount++;

  return;
}

//////////////////////////////////////////////////////////////////////////////
// Save the current frame to disk
/*void StereoCameraSensor::SaveFrame()
{
  Ogre::HardwarePixelBufferSharedPtr mBuffer;
  std::ostringstream sstream;
  Ogre::ImageCodec::ImageData *imgData;
  Ogre::Codec * pCodec;
  size_t size, pos;

  for (int i=0; i<4; i++)
  {
    this->GetImageData(i);

    // Get access to the buffer and make an image and write it to file
    mBuffer = this->renderTexture[i]->getBuffer(0, 0);

    // Create image data structure
    imgData  = new Ogre::ImageCodec::ImageData();

    imgData->width = this->imageWidth;
    imgData->height = this->imageHeight;
    imgData->depth = 1;
    if (i<2)
      imgData->format = Ogre::PF_B8G8R8;
    else
      imgData->format = PF_FLOAT;

    size = this->GetImageByteSize();

    // Wrap buffer in a chunk
    Ogre::MemoryDataStreamPtr stream(new Ogre::MemoryDataStream( this->saveFrameBuffer, size, false));

    char tmp[1024];
    if (!this->savePathname.empty())
    {
      //if (i==0)
        sprintf(tmp, "%s/%s-%04d-left.png", this->savePathname.c_str(),
            this->GetName().c_str(), this->saveCount);
      //else
        //sprintf(tmp, "%s/%s-%04d-right.png", this->savePathname.c_str(),
            //this->GetName().c_str(), this->saveCount);
    }
    else
    {
      //if (i==0)
        sprintf(tmp, "%s-%04d-left.png", this->GetName().c_str(), this->saveCount);
      //else
       // sprintf(tmp, "%s-%04d-right.png", this->GetName().c_str(), this->saveCount);
    }

    // Get codec
    Ogre::String filename = tmp;
    pos = filename.find_last_of(".");
    Ogre::String extension;

    while (pos != filename.length() - 1)
      extension += filename[++pos];

    // Get the codec
    pCodec = Ogre::Codec::getCodec(extension);

    // Write out
    Ogre::Codec::CodecDataPtr codecDataPtr(imgData);
    pCodec->codeToFile(stream, filename, codecDataPtr);
  }

  this->saveCount++;
}
  */

////////////////////////////////////////////////////////////////////////////////
/// Get the baselien of the camera
double StereoCameraSensor::GetBaseline() const
{
  return this->baseline;
}


/*void StereoCameraSensor::StereoCameraListener::Init(
    StereoCameraSensor *cam, Ogre::RenderTarget *target, bool isLeft)
{
  this->sensor = cam;
  this->camera = this->sensor->GetOgreCamera();
  this->renderTarget = target;
  this->isLeftCamera = isLeft;
}

void StereoCameraSensor::StereoCameraListener::preViewportUpdate(const Ogre::RenderTargetViewportEvent &evt)
{
	if(evt.source != this->renderTarget->getViewport(0))
  {
    printf("Invalid viewport");
		return;
  }

	double offset = this->sensor->GetBaseline()/2;

	if(this->isLeftCamera)
	{
		offset = -offset;
	}

	this->camera->setFrustumOffset(-offset,0);

	this->pos = this->camera->getPosition();
  Ogre::Vector3 pos = this->pos;

	pos += offset * this->camera->getRight();
	this->camera->setPosition(pos);

	//this->sensor->UpdateAllDependentRenderTargets();
	//this->sensor->chooseDebugPlaneMaterial(mIsLeftEye);
}

void StereoCameraSensor::StereoCameraListener::postViewportUpdate(const Ogre::RenderTargetViewportEvent &evt)
{
  //this->sensor->ReadDepthImage();

	this->camera->setFrustumOffset(0,0);
	this->camera->setPosition(this->pos);
}
*/

Ogre::TexturePtr StereoCameraSensor::CreateRTT(const std::string &name, bool depth)
{
  Ogre::PixelFormat pf;

  if (depth)
    pf = PF_FLOAT;
  else
    pf = Ogre::PF_R8G8B8;

  // Create the left render texture
  return Ogre::TextureManager::getSingleton().createManual(
      name, 
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
      Ogre::TEX_TYPE_2D,
      this->imageWidth, this->imageHeight, 0,
      pf,
      Ogre::TU_RENDERTARGET);
}
