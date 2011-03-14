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
 * CVS: $Id$
 */

#include <sstream>
#include <OgreImageCodec.h>

#include "common/Events.hh"
#include "Controller.hh"
#include "common/Global.hh"
#include "World.hh"
#include "common/GazeboError.hh"
#include "Body.hh"
#include "rendering/Visual.hh"
#include "Simulator.hh"

#include "SensorFactory.hh"
#include "MonoCameraSensor.hh"
#include "rendering/Scene.hh"
#include "Simulator.hh"

using namespace gazebo;

GZ_REGISTER_STATIC_SENSOR("camera", MonoCameraSensor);

//////////////////////////////////////////////////////////////////////////////
// Constructor
MonoCameraSensor::MonoCameraSensor(Body *body)
    : Sensor(body)
{
  this->camera = this->GetWorld()->GetScene()->CreateCamera("Mono");

  this->typeName = "monocamera";
  this->camera->SetCaptureData(true);

  event::Events::ConnectRenderSignal( boost::bind(&MonoCameraSensor::Render, this) );
}


//////////////////////////////////////////////////////////////////////////////
// Destructor
MonoCameraSensor::~MonoCameraSensor()
{
}

//////////////////////////////////////////////////////////////////////////////
/// Get the camera
Camera *MonoCameraSensor::GetCamera()
{
  return this->camera;
}

//////////////////////////////////////////////////////////////////////////////
// Load the camera
void MonoCameraSensor::LoadChild( XMLConfigNode *node )
{
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
void MonoCameraSensor::SaveChild(std::string &prefix, std::ostream &stream)
{
  std::string p = prefix + "  ";
  this->camera->Save(p, stream);
}

//////////////////////////////////////////////////////////////////////////////
// Initialize the camera
void MonoCameraSensor::InitChild()
{
  // NATY: Put this back in!!
  // this->camera->SetSceneNode( this->GetVisualNode()->GetSceneNode() );
  this->camera->Init();
}

//////////////////////////////////////////////////////////////////////////////
// Finalize the camera
void MonoCameraSensor::FiniChild()
{
  this->camera->Fini();
}

//////////////////////////////////////////////////////////////////////////////
/// Set whether the sensor is active or not
void MonoCameraSensor::SetActive(bool value)
{
  Sensor::SetActive(value);
  this->camera->SetRenderingEnabled(value);
}

//////////////////////////////////////////////////////////////////////////////
// Render new data
void MonoCameraSensor::Render()
{
  if (this->active || **this->alwaysActiveP)
  {
    this->lastUpdate = this->GetWorld()->GetSimTime();
    this->camera->Render();
    this->camera->PostRender();
  }
}

//////////////////////////////////////////////////////////////////////////////
// Update the drawing
void MonoCameraSensor::UpdateChild()
{
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
