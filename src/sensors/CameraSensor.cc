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
#include "common/GazeboError.hh"

#include "rendering/Camera.hh"

#include "sensors/SensorFactory.hh"
#include "sensors/CameraSensor.hh"

using namespace gazebo;
using namespace sensors;

GZ_REGISTER_STATIC_SENSOR("camera", CameraSensor);
 
//////////////////////////////////////////////////////////////////////////////
// Constructor
CameraSensor::CameraSensor()
    : Sensor()
{
  //this->camera = this->GetWorld()->GetScene()->CreateCamera("Mono");

  this->typeName = "monocamera";
  this->camera->SetCaptureData(true);

  event::Events::ConnectRenderSignal( boost::bind(&CameraSensor::Render, this) );
}


//////////////////////////////////////////////////////////////////////////////
// Destructor
CameraSensor::~CameraSensor()
{
}

//////////////////////////////////////////////////////////////////////////////
// Load the camera
void CameraSensor::Load( common::XMLConfigNode *node )
{
  Sensor::Load(node);

  this->camera->Load( node );

  // Do some sanity checks
  if (this->camera->GetImageWidth() == 0 || 
      this->camera->GetImageHeight() == 0)
  {
    gzthrow("image has zero size");
  }

  this->ogreTextureName = this->GetName() + "_RttTex";

  this->camera->CreateRenderTexture(this->ogreTextureName);
}

//////////////////////////////////////////////////////////////////////////////
/// Save the sensor info in XML format
void CameraSensor::SaveChild(std::string &prefix, std::ostream &stream)
{
  std::string p = prefix + "  ";
  this->camera->Save(p, stream);
}

//////////////////////////////////////////////////////////////////////////////
// Initialize the camera
void CameraSensor::InitChild()
{
  // NATY: Put this back in!!
  // this->camera->SetSceneNode( this->GetVisualNode()->GetSceneNode() );
  this->camera->Init();
}

//////////////////////////////////////////////////////////////////////////////
// Finalize the camera
void CameraSensor::FiniChild()
{
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
  /*
  if (this->active || **this->alwaysActiveP)
  {
    this->lastUpdate = this->GetWorld()->GetSimTime();
    this->camera->Render();
    this->camera->PostRender();
  }
  */
}

//////////////////////////////////////////////////////////////////////////////
// Update the drawing
void CameraSensor::Update()
{
  Sensor::Update();

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
