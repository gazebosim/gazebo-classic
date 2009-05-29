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

/* Desc: A camera sensor using OpenGL
 * Author: Nate Koenig
 * Date: 15 July 2003
 * CVS: $Id$
 */

#include <sstream>
#include <OgreImageCodec.h>

#include "Controller.hh"
#include "Global.hh"
#include "World.hh"
#include "GazeboError.hh"
#include "Body.hh"
#include "OgreVisual.hh"
#include "OgreCreator.hh"
#include "OgreFrameListener.hh"
#include "Simulator.hh"

#include "SensorFactory.hh"
#include "CameraManager.hh"
#include "MonoCameraSensor.hh"

#include "Simulator.hh"

using namespace gazebo;

GZ_REGISTER_STATIC_SENSOR("camera", MonoCameraSensor);

//////////////////////////////////////////////////////////////////////////////
// Constructor
MonoCameraSensor::MonoCameraSensor(Body *body)
    : Sensor(body), OgreCamera("Mono")
{
  this->typeName = "monocamera";
  this->captureData = true;
}


//////////////////////////////////////////////////////////////////////////////
// Destructor
MonoCameraSensor::~MonoCameraSensor()
{
}

//////////////////////////////////////////////////////////////////////////////
// Load the camera
void MonoCameraSensor::LoadChild( XMLConfigNode *node )
{
  this->LoadCam( node );

  // Do some sanity checks
  if (this->imageSizeP->GetValue().x == 0 || 
      this->imageSizeP->GetValue().y == 0)
  {
    gzthrow("image has zero size");
  }

  if (Simulator::Instance()->GetRenderEngineEnabled())
  {
    this->ogreTextureName = this->GetName() + "_RttTex";
    this->ogreMaterialName = this->GetName() + "_RttMat";

    // Create the render texture
    this->renderTexture = Ogre::TextureManager::getSingleton().createManual(
                          this->ogreTextureName,
                          "General",
                          Ogre::TEX_TYPE_2D,
                          this->imageSizeP->GetValue().x, 
                          this->imageSizeP->GetValue().y,
                          0,
                          this->imageFormat,
                          Ogre::TU_RENDERTARGET);

    this->renderTarget = this->renderTexture->getBuffer()->getRenderTarget();
  }
}

//////////////////////////////////////////////////////////////////////////////
/// Save the sensor info in XML format
void MonoCameraSensor::SaveChild(std::string &prefix, std::ostream &stream)
{
  std::string p = prefix + "  ";
  this->SaveCam(p, stream);
}

//////////////////////////////////////////////////////////////////////////////
// Initialize the camera
void MonoCameraSensor::InitChild()
{

  if (Simulator::Instance()->GetRenderEngineEnabled())
  {
    this->SetCameraSceneNode( this->GetVisualNode()->GetSceneNode() );
    this->InitCam();
    this->SetCamName(this->GetName());

    /*Ogre::MaterialPtr mat = Ogre::MaterialManager::getSingleton().create(
                              this->ogreMaterialName,
                              Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

     mat->getTechnique(0)->getPass(0)->createTextureUnitState(this->ogreTextureName);
*/

    Ogre::HardwarePixelBufferSharedPtr mBuffer;
    // Get access to the buffer and make an image and write it to file
    mBuffer = this->renderTexture->getBuffer(0, 0);

    this->textureWidth = mBuffer->getWidth();
    this->textureHeight = mBuffer->getHeight();
  }
  
}

//////////////////////////////////////////////////////////////////////////////
// Finalize the camera
void MonoCameraSensor::FiniChild()
{
  this->FiniCam();
}

//////////////////////////////////////////////////////////////////////////////
// Update the drawing
void MonoCameraSensor::UpdateChild()
{
  if (!Simulator::Instance()->GetRenderEngineEnabled())
    return;

  // Only continue if the controller has an active interface. Or frames need
  // to be saved
  if ( (this->controller && !this->controller->IsConnected()) &&
       !this->saveFramesP->GetValue())
    return;

  this->UpdateCam();
}

////////////////////////////////////////////////////////////////////////////////
// Return the material the camera renders to
std::string MonoCameraSensor::GetMaterialName() const
{
  if (!Simulator::Instance()->GetRenderEngineEnabled())
    return NULL;
  else
    return this->ogreMaterialName;
}
