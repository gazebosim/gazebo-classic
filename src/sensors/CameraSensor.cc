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

#include "common/Events.hh"
#include "common/Global.hh"
#include "common/Exception.hh"

#include "rendering/Camera.hh"
#include "rendering/Scene.hh"
#include "rendering/RenderEngine.hh"

#include "sensors/SensorFactory.hh"
#include "sensors/CameraSensor.hh"

using namespace gazebo;
using namespace sensors;

GZ_REGISTER_STATIC_SENSOR("camera", CameraSensor)
 
//////////////////////////////////////////////////////////////////////////////
// Constructor
CameraSensor::CameraSensor()
    : Sensor()
{
  rendering::ScenePtr scene = rendering::RenderEngine::Instance()->GetScene(0);
  this->camera = scene->CreateCamera("mono_camera");

  this->typeName = "monocamera";
  this->camera->SetCaptureData(true);

  this->connections.push_back( event::Events::ConnectRenderSignal( boost::bind(&CameraSensor::Render, this)) );
}


//////////////////////////////////////////////////////////////////////////////
// Destructor
CameraSensor::~CameraSensor()
{
}

//////////////////////////////////////////////////////////////////////////////
// Load the camera
void CameraSensor::Load( sdf::ElementPtr _sdf )
{
  Sensor::Load(_sdf);

  this->camera->Load( _sdf );

  // Do some sanity checks
  if (this->camera->GetImageWidth() == 0 || 
      this->camera->GetImageHeight() == 0)
  {
    gzthrow("image has zero size");
  }
}

//////////////////////////////////////////////////////////////////////////////
// Initialize the camera
void CameraSensor::Init()
{
  Sensor::Init();
  this->camera->Init();

  // Create the render texture
  this->ogreTextureName = this->GetName() + "_RttTex";
  this->camera->CreateRenderTexture(this->ogreTextureName);
}

//////////////////////////////////////////////////////////////////////////////
// Finalize the camera
void CameraSensor::Fini()
{
  Sensor::Fini();
  this->camera->Fini();
}

//////////////////////////////////////////////////////////////////////////////
/// Set whether the sensor is active or not
void CameraSensor::SetActive(bool value)
{
  Sensor::SetActive(value);
  this->camera->SetRenderingEnabled(value);
}

//////////////////////////////////////////////////////////////////////////////
// Render new data
void CameraSensor::Render()
{
  //if (this->active || **this->alwaysActiveP)
  {
    //this->lastUpdate = this->GetWorld()->GetSimTime();
    //this->camera->Render();
    //this->camera->PostRender();
  }
  
}

//////////////////////////////////////////////////////////////////////////////
// Update the drawing
void CameraSensor::Update(bool force)
{
  Sensor::Update(force);
  this->camera->Render();
  this->camera->PostRender();

  // NATY
  //if (this->active || **this->alwaysActiveP)
    //this->camera->Update();

  // Only continue if the controller has an active interface. Or frames need
  // to be saved
  /*if ( (this->controller && !this->controller->IsConnected()) &&
       !this->saveFramesP->GetValue())
    return;

  // Or skip if user sets camera to inactive
  if (this->active)
    this->UpdateCam();
    */
}
