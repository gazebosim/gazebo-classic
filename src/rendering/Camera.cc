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
#include "rendering/Camera.hh"

using namespace gazebo;
using namespace rendering;


unsigned int Camera::cameraCounter = 0;

//////////////////////////////////////////////////////////////////////////////
// Constructor
Camera::Camera(const std::string &namePrefix_, Scene *scene_)
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
  this->connections.push_back( event::Events::ConnectRenderSignal( boost::bind(&Camera::Render, this) ) );
  this->connections.push_back( event::Events::ConnectPostRenderSignal( boost::bind(&Camera::PostRender, this) ) );
  this->connections.push_back( event::Events::ConnectShowWireframeSignal( boost::bind(&Camera::ToggleShowWireframe, this) ));
}

//////////////////////////////////////////////////////////////////////////////
// Destructor
Camera::~Camera()
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
void Camera::Load( sdf::ElementPtr &_sdf )
{
  this->sdf = _sdf;
  this->Load();
}

//////////////////////////////////////////////////////////////////////////////
// Load the camera
void Camera::Load()
{

  sdf::ElementPtr imageElem = this->sdf->GetOrCreateElement("image");

  std::string imgFmt = imageElem->GetValueString("format");

  gzdbg << "Image Format[" << imgFmt << "]\n";
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
      gzthrow("Camera horizontal field of veiw invalid.");
    }
  }
}
 
//////////////////////////////////////////////////////////////////////////////
// Initialize the camera
void Camera::Init()
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
}

//////////////////////////////////////////////////////////////////////////////
// Finalize the camera
void Camera::Fini()
{
}

////////////////////////////////////////////////////////////////////////////////
/// Set the ID of the window this camera is rendering into.
void Camera::SetWindowId( unsigned int windowId_ )
{
  this->windowId = windowId_;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the ID of the window this camera is rendering into.
unsigned int Camera::GetWindowId() const
{
  return this->windowId;
}

//////////////////////////////////////////////////////////////////////////////
/// Set the scene this camera is viewing
void Camera::SetScene( Scene *scene_ )
{
  this->scene = scene_;
}


//////////////////////////////////////////////////////////////////////////////
// Update the drawing
void Camera::Update()
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

////////////////////////////////////////////////////////////////////////////////
/// Set to true to enable rendering
void Camera::SetRenderingEnabled(bool value)
{
  this->renderingEnabled = value;
}

////////////////////////////////////////////////////////////////////////////////
/// Get whether the rendering is enabled
bool Camera::GetRenderingEnabled() const
{
  return this->renderingEnabled;
}

////////////////////////////////////////////////////////////////////////////////
// Render the camera
void Camera::Render()
{
  gzdbg << "Camera Render[" << this->GetWorldPose() << "]\n";
  this->newData = true;
  this->renderTarget->update(false);
}

////////////////////////////////////////////////////////////////////////////////
void Camera::PostRender()
{
  gzdbg << "Camera Post Render\n";
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
  }

  this->newData = false;
}


////////////////////////////////////////////////////////////////////////////////
// Get the global pose of the camera
math::Pose Camera::GetWorldPose()
{
  this->pose.pos = Conversions::Vector3(this->camera->getRealPosition());
  this->pose.rot = Conversions::Quaternion(this->camera->getRealOrientation());
  return this->pose;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the camera position in the world
math::Vector3 Camera::GetWorldPosition() const
{
  Ogre::Vector3 camPos = this->camera->getRealPosition();
  return math::Vector3(camPos.x,camPos.y,camPos.z);
}

////////////////////////////////////////////////////////////////////////////////
/// Set the global pose of the camera
void Camera::SetWorldPose(const math::Pose &pose_)
{
  this->pose = pose_;
  this->pose.Correct();
  this->sceneNode->setPosition( this->pose.pos.x, this->pose.pos.y, this->pose.pos.z);
  //this->pitchNode->setOrientation( this->pose.rot.w, this->pose.rot.x, this->pose.rot.y, this->pose.rot.z);
}

////////////////////////////////////////////////////////////////////////////////
/// Set the world position
void Camera::SetWorldPosition(const math::Vector3 &pos)
{
  this->pose.pos = pos;
  this->pose.Correct();
  
  this->sceneNode->setPosition( this->pose.pos.x, this->pose.pos.y, this->pose.pos.z);
}

////////////////////////////////////////////////////////////////////////////////
/// Set the world orientation
void Camera::SetWorldRotation(const math::Quaternion &quant)
{
  this->pose.rot = quant;
  this->pose.Correct();

 gzdbg << "Set World Rotation[" << quant << "]\n"; 
  this->pitchNode->setOrientation( this->pose.rot.w, this->pose.rot.x, this->pose.rot.y, this->pose.rot.z);
}

////////////////////////////////////////////////////////////////////////////////
// Translate the camera
void Camera::Translate( const math::Vector3 &direction )
{
  Ogre::Vector3 vec(direction.x, direction.y, direction.z);

  this->sceneNode->translate(this->sceneNode->getOrientation() * this->pitchNode->getOrientation() * vec);

}

//////////////////////////////////////////////////////////////////////////////
// Rotate the camera around the yaw axis
void Camera::RotateYaw( float angle )
{
  this->sceneNode->roll(Ogre::Radian(angle), Ogre::Node::TS_WORLD);
}

//////////////////////////////////////////////////////////////////////////////
// Rotate the camera around the pitch axis
void Camera::RotatePitch( float angle )
{
  gzdbg << "Rotate Pitch[" << angle << "]\n";
  this->pitchNode->yaw(Ogre::Radian(angle));
}


////////////////////////////////////////////////////////////////////////////////
/// Set the clip distances
void Camera::SetClipDist(float _near, float _far)
{
  sdf::ElementPtr elem = this->sdf->GetOrCreateElement("clip");

  elem->GetAttribute("near")->Set(_near);
  elem->GetAttribute("far")->Set(_far);

  if (camera)
  {
    this->camera->setNearClipDistance(_near);
    this->camera->setFarClipDistance(_far);
    this->camera->setRenderingDistance(_far);
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Set the camera FOV (horizontal)  
void Camera::SetFOV( float radians )
{
  sdf::ElementPtr elem = this->sdf->GetOrCreateElement("horizontal_fov");
  elem->GetAttribute("angle")->Set(radians);
}


//////////////////////////////////////////////////////////////////////////////
/// Get the horizontal field of view of the camera
math::Angle Camera::GetHFOV() const
{
  sdf::ElementPtr elem = this->sdf->GetOrCreateElement("horizontal_fov");
  return math::Angle( elem->GetValueDouble("angle") );
}

//////////////////////////////////////////////////////////////////////////////
/// Get the vertical field of view of the camera
math::Angle Camera::GetVFOV() const
{
  return math::Angle(this->camera->getFOVy().valueRadians());
}

//////////////////////////////////////////////////////////////////////////////
/// Get the width of the image
unsigned int Camera::GetImageWidth() const
{
  sdf::ElementPtr elem = this->sdf->GetOrCreateElement("image");
  return elem->GetValueInt("width");
}

//////////////////////////////////////////////////////////////////////////////
/// \brief Get the height of the image
unsigned int Camera::GetImageHeight() const
{
  sdf::ElementPtr elem = this->sdf->GetOrCreateElement("image");
  return elem->GetValueInt("height");
}

//////////////////////////////////////////////////////////////////////////////
/// \brief Get the height of the image
int Camera::GetImageDepth() const
{
  sdf::ElementPtr imageElem = this->sdf->GetOrCreateElement("image");
  std::string imgFmt = imageElem->GetValueString("format");

  if (imgFmt == "L8")
    return 1;
  else if (imgFmt == "R8G8B8")
    return 3;
  else if (imgFmt == "B8G8R8")
    return 3;
  else if ( (imgFmt == "BAYER_RGGB8") || (imgFmt == "BAYER_BGGR8") ||
            (imgFmt == "BAYER_GBRG8") || (imgFmt == "BAYER_GRBG8") )
    return 1;
  else
  {
    gzerr << "Error parsing image format (" 
          << imgFmt << "), using default Ogre::PF_R8G8B8\n";
    return 3;
  }
}

//////////////////////////////////////////////////////////////////////////////
/// \brief Get the height of the image
std::string Camera::GetImageFormat() const
{
  sdf::ElementPtr imageElem = this->sdf->GetOrCreateElement("image");
  return imageElem->GetValueString("format");
}

//////////////////////////////////////////////////////////////////////////////
/// Get the width of the texture
unsigned int Camera::GetTextureWidth() const
{
  return this->renderTexture->getBuffer(0,0)->getWidth();
}

//////////////////////////////////////////////////////////////////////////////
/// \brief Get the height of the texture
unsigned int Camera::GetTextureHeight() const
{
  return this->renderTexture->getBuffer(0,0)->getHeight();
}


//////////////////////////////////////////////////////////////////////////////
// Get the image size in bytes
size_t Camera::GetImageByteSize() const
{
  sdf::ElementPtr elem = this->sdf->GetOrCreateElement("image");
  return Ogre::PixelUtil::getMemorySize(elem->GetValueInt("width"),
                                        elem->GetValueInt("height"),
                                        1, 
                                        (Ogre::PixelFormat)this->imageFormat);
}


//////////////////////////////////////////////////////////////////////////////
// Enable or disable saving
void Camera::EnableSaveFrame(bool enable)
{
  sdf::ElementPtr elem = this->sdf->GetOrCreateElement("save");
  elem->GetAttribute("enabled")->Set(enable);
}

//////////////////////////////////////////////////////////////////////////////
// Set the save frame pathname
void Camera::SetSaveFramePathname(const std::string &_pathname)
{
  sdf::ElementPtr elem = this->sdf->GetOrCreateElement("save");
  elem->GetAttribute("path")->Set( _pathname );

  // Create the directory to store frames
  if (elem->GetValueBool("enabled"))
  {
    std::string command;
    command = "mkdir " + _pathname + " 2>>/dev/null";
    if (system(command.c_str()) <0)
      gzerr << "Error making directory\n";
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Get a pointer to the ogre camera
Ogre::Camera *Camera::GetOgreCamera() const
{
  return this->camera;
}

////////////////////////////////////////////////////////////////////////////////
Ogre::Viewport *Camera::GetViewport() const
{
  return this->viewport;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the near clip distance
double Camera::GetNearClip()
{
  if (this->camera)
    return this->camera->getNearClipDistance();
  else
    return 0;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the far clip distance
double Camera::GetFarClip()
{
  if (this->camera)
    return this->camera->getFarClipDistance();
  else
    return 0;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the viewport width in pixels
unsigned int Camera::GetViewportWidth() const
{
  if (this->renderTarget)
    return this->renderTarget->getViewport(0)->getActualWidth();
  else
    return this->camera->getViewport()->getActualWidth();
}

////////////////////////////////////////////////////////////////////////////////
/// Get the viewport height in pixels
unsigned int Camera::GetViewportHeight() const
{
  if (this->renderTarget)
    return this->renderTarget->getViewport(0)->getActualHeight();
  else
    return this->camera->getViewport()->getActualHeight();
}

////////////////////////////////////////////////////////////////////////////////
/// Set the aspect ratio
void Camera::SetAspectRatio( float ratio )
{
  this->camera->setAspectRatio( ratio );
}

////////////////////////////////////////////////////////////////////////////////
/// Get the viewport up vector
math::Vector3 Camera::GetUp()
{
  Ogre::Vector3 up = this->camera->getRealUp();
  return math::Vector3(up.x,up.y,up.z);
}

////////////////////////////////////////////////////////////////////////////////
/// Get the viewport right vector
math::Vector3 Camera::GetRight()
{
  Ogre::Vector3 right = this->camera->getRealRight();
  return math::Vector3(right.x,right.y,right.z);
}



////////////////////////////////////////////////////////////////////////////////
/// Set whether the user can move the camera via the GUI
void Camera::SetUserMovable( bool movable )
{
  this->userMovable = movable;
}

////////////////////////////////////////////////////////////////////////////////
/// Get whether the user can move the camera via the GUI
bool Camera::GetUserMovable() const
{
  return this->userMovable;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the camera's scene node
void Camera::SetSceneNode( Ogre::SceneNode *node )
{
  this->sceneNode = node;
}

//////////////////////////////////////////////////////////////////////////////
/// Get the camera's scene node
Ogre::SceneNode *Camera::GetSceneNode() const
{
  return this->pitchNode;
}

//////////////////////////////////////////////////////////////////////////////
/// Get a pointer to the image data
const unsigned char *Camera::GetImageData(unsigned int i)
{
  if (i!=0)
    gzerr << "Camera index must be zero for cam";

  sdf::ElementPtr elem = this->sdf->GetOrCreateElement("image");

  int width = elem->GetValueInt("width");
  int height = elem->GetValueInt("height");
  std::string imgFmt = elem->GetValueString("format");

  // do last minute conversion if Bayer pattern is requested, go from R8G8B8
  if ( (imgFmt == "BAYER_RGGB8") || (imgFmt == "BAYER_BGGR8") ||
       (imgFmt == "BAYER_GBRG8") || (imgFmt == "BAYER_GRBG8") )
  {
    if (!this->bayerFrameBuffer)
      this->bayerFrameBuffer = new unsigned char[width*height];

    this->ConvertRGBToBAYER(this->bayerFrameBuffer,
        this->saveFrameBuffer, imgFmt, width, height);

    return this->bayerFrameBuffer;
  }
  else
    return this->saveFrameBuffer;
}

//////////////////////////////////////////////////////////////////////////////
/// Get the camera's name
std::string Camera::GetName() const
{
  return this->name;
}

//////////////////////////////////////////////////////////////////////////////
// Save the current frame to disk
void Camera::SaveFrame()
{
  Ogre::HardwarePixelBufferSharedPtr mBuffer;
  std::ostringstream sstream;
  Ogre::ImageCodec::ImageData *imgData;
  Ogre::Codec * pCodec;
  size_t size, pos;

  this->GetImageData();

  sdf::ElementPtr saveElem = this->sdf->GetOrCreateElement("save");
  sdf::ElementPtr imageElem = this->sdf->GetOrCreateElement("image");

  std::string path = saveElem->GetValueString("path");

  // Create a directory if not present
  DIR *dir = opendir( path.c_str() );
  if (!dir)
  {
    std::string command;
    command = "mkdir " + path + " 2>>/dev/null";
    if (system(command.c_str()) < 0)
      gzerr << "Error making directory\n";
  }

  // Get access to the buffer and make an image and write it to file
  mBuffer = this->renderTexture->getBuffer(0, 0);

  // Create image data structure
  imgData  = new Ogre::ImageCodec::ImageData();

  imgData->width =  imageElem->GetValueInt("width");
  imgData->height = imageElem->GetValueInt("height");
  imgData->depth = this->GetImageDepth();
  imgData->format = (Ogre::PixelFormat)this->imageFormat;
  size = this->GetImageByteSize();

  // Wrap buffer in a chunk
  Ogre::MemoryDataStreamPtr stream(new Ogre::MemoryDataStream( this->saveFrameBuffer, size, false));

  char tmp[1024];
  if (!path.empty())
  {
    double wallTime = common::Time::GetWallTime().Double();
    int min = (int)(wallTime / 60.0);
    int sec = (int)(wallTime - min*60);
    int msec = (int)(wallTime*1000 - min*60000 - sec*1000);

    sprintf(tmp, "%s/%s-%04d-%03dm_%02ds_%03dms.jpg", 
        path.c_str(), this->GetName().c_str(), 
        this->saveCount, min, sec, msec);
  }
  else
  {
    sprintf(tmp, "%s-%04d.jpg", this->GetName().c_str(), this->saveCount);
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

  this->saveCount++;
  
}

//////////////////////////////////////////////////////////////////////////////
// Toggle whether to view the world in wireframe
void Camera::ToggleShowWireframe()
{
   if (this->camera)
  {
    if (this->camera->getPolygonMode() == Ogre::PM_WIREFRAME)
      this->camera->setPolygonMode(Ogre::PM_SOLID);
    else
      this->camera->setPolygonMode(Ogre::PM_WIREFRAME);
  } 
}

//////////////////////////////////////////////////////////////////////////////
// Set whether to view the world in wireframe
void Camera::ShowWireframe(bool s)
{
  if (this->camera)
  {
    if (s)
    {
      this->camera->setPolygonMode(Ogre::PM_WIREFRAME);
    }
    else
    {
      this->camera->setPolygonMode(Ogre::PM_SOLID);
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Get a world space ray as cast from the camer through the viewport
void Camera::GetCameraToViewportRay(int screenx, int screeny,
                                        math::Vector3 &origin, math::Vector3 &dir)
{
  Ogre::Ray ray = this->camera->getCameraToViewportRay(
      (float)screenx / this->GetViewportWidth(),
      (float)screeny / this->GetViewportHeight());

  origin.Set(ray.getOrigin().x, ray.getOrigin().y, ray.getOrigin().z);
  dir.Set(ray.getDirection().x, ray.getDirection().y, ray.getDirection().z);
}


////////////////////////////////////////////////////////////////////////////////
/// post process, convert from rgb to bayer
void Camera::ConvertRGBToBAYER(unsigned char* dst, unsigned char* src, std::string format,int width, int height)
{
  if (src)
  {
    // do last minute conversion if Bayer pattern is requested, go from R8G8B8
    if (format == "BAYER_RGGB8")
    {
      for (int i=0;i<width;i++)
      {
        for (int j=0;j<height;j++)
        {
          //
          // RG
          // GB
          //
          // determine position
          if (j%2) // even column
            if (i%2) // even row, red
              dst[i+j*width] = src[i*3+j*width*3+0];
            else // odd row, green
              dst[i+j*width] = src[i*3+j*width*3+1];
          else // odd column
            if (i%2) // even row, green
              dst[i+j*width] = src[i*3+j*width*3+1];
            else // odd row, blue
              dst[i+j*width] = src[i*3+j*width*3+2];
        }
      }
    }
    else if (format == "BAYER_BGGR8")
    {
      for (int i=0;i<width;i++)
      {
        for (int j=0;j<height;j++)
        {
          //
          // BG
          // GR
          //
          // determine position
          if (j%2) // even column
            if (i%2) // even row, blue
              dst[i+j*width] = src[i*3+j*width*3+2];
            else // odd row, green
              dst[i+j*width] = src[i*3+j*width*3+1];
          else // odd column
            if (i%2) // even row, green
              dst[i+j*width] = src[i*3+j*width*3+1];
            else // odd row, red
              dst[i+j*width] = src[i*3+j*width*3+0];
        }
      }
    }
    else if (format == "BAYER_GBRG8")
    {
      for (int i=0;i<width;i++)
      {
        for (int j=0;j<height;j++)
        {
          //
          // GB
          // RG
          //
          // determine position
          if (j%2) // even column
            if (i%2) // even row, green
              dst[i+j*width] = src[i*3+j*width*3+1];
            else // odd row, blue
              dst[i+j*width] = src[i*3+j*width*3+2];
          else // odd column
            if (i%2) // even row, red
              dst[i+j*width] = src[i*3+j*width*3+0];
            else // odd row, green
              dst[i+j*width] = src[i*3+j*width*3+1];
        }
      }
    }
    else if (format == "BAYER_GRBG8")
    {
      for (int i=0;i<width;i++)
      {
        for (int j=0;j<height;j++)
        {
          //
          // GR
          // BG
          //
          // determine position
          if (j%2) // even column
            if (i%2) // even row, green
              dst[i+j*width] = src[i*3+j*width*3+1];
            else // odd row, red
              dst[i+j*width] = src[i*3+j*width*3+0];
          else // odd column
            if (i%2) // even row, blue
              dst[i+j*width] = src[i*3+j*width*3+2];
            else // odd row, green
              dst[i+j*width] = src[i*3+j*width*3+1];
        }
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Set whether to capture data
void Camera::SetCaptureData( bool value )
{
  this->captureData = value;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the render target
void Camera::CreateRenderTexture( const std::string &textureName )
{
  gzdbg << "Creating render texture\n";

  // Create the render texture
  this->renderTexture = (Ogre::TextureManager::getSingleton().createManual(
      textureName,
      "General",
      Ogre::TEX_TYPE_2D,
      this->GetImageWidth(), 
      this->GetImageHeight(),
      0,
      (Ogre::PixelFormat)this->imageFormat,
      Ogre::TU_RENDERTARGET)).getPointer();

  this->SetRenderTarget(this->renderTexture->getBuffer()->getRenderTarget());
  RTShaderSystem::AttachViewport(this->GetViewport(), this->GetScene());
}

////////////////////////////////////////////////////////////////////////////////
// Return this scene
Scene *Camera::GetScene() const
{
  return this->scene;
}

////////////////////////////////////////////////////////////////////////////////
// Create the ogre camera
void Camera::CreateCamera()
{
  this->camera = this->scene->GetManager()->createCamera(this->name);

  // Use X/Y as horizon, Z up
  this->camera->pitch(Ogre::Degree(90));

  // Don't yaw along variable axis, causes leaning
  this->camera->setFixedYawAxis(true, Ogre::Vector3::UNIT_Z);

  this->camera->setDirection(1,0,0);
}

////////////////////////////////////////////////////////////////////////////////
// Get point on a plane
math::Vector3 Camera::GetWorldPointOnPlane(int x, int y, math::Vector3 planeNorm, double d)
{
  math::Vector3 origin, dir;
  double dist;

  // Cast two rays from the camera into the world
  this->GetCameraToViewportRay(x, y, origin, dir);

  dist = origin.GetDistToPlane(dir, planeNorm, d);

  // Compute two points on the plane. The first point is the current
  // mouse position, the second is the previous mouse position
  return origin + dir * dist; 
}

////////////////////////////////////////////////////////////////////////////////
// Set the render target for the camera
void Camera::SetRenderTarget( Ogre::RenderTarget *target )
{
  this->renderTarget = target;

  if (this->renderTarget)
  {
    // Setup the viewport to use the texture
    this->viewport = this->renderTarget->addViewport(this->camera);
    this->viewport->setClearEveryFrame(true);
    this->viewport->setBackgroundColour( Conversions::Color( this->scene->GetBackgroundColor() ) );

    double ratio = (double)this->viewport->getActualWidth() / 
                   (double)this->viewport->getActualHeight();

    double hfov = this->sdf->GetOrCreateElement("horizontal_fov")->GetValueDouble("angle");
    gzdbg << "HFOV[" << hfov << "]\n";
    double vfov = 2.0 * atan(tan( hfov / 2.0) / ratio);
    this->camera->setAspectRatio(ratio);
    this->camera->setFOVy(Ogre::Radian(vfov));
  }
}
