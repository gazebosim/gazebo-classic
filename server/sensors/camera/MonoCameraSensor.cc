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

#include "Events.hh"
#include "Controller.hh"
#include "Global.hh"
#include "World.hh"
#include "GazeboError.hh"
#include "Body.hh"
#include "Visual.hh"
#include "Simulator.hh"

#include "SensorFactory.hh"
#include "MonoCameraSensor.hh"
#include "Scene.hh"
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

  Events::ConnectRenderSignal( boost::bind(&MonoCameraSensor::Render, this) );
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

  if (Simulator::Instance()->GetRenderEngineEnabled())
  {
    this->ogreTextureName = this->GetName() + "_RttTex";

    this->camera->CreateRenderTexture(this->ogreTextureName);
  }
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

  if (Simulator::Instance()->GetRenderEngineEnabled())
  {
    // NATY: Put this back in!!
    // this->camera->SetSceneNode( this->GetVisualNode()->GetSceneNode() );
    this->camera->Init();
  }
  
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
  if (!Simulator::Instance()->GetRenderEngineEnabled())
    return;

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
