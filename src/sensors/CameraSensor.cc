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
#include "common/Exception.hh"

#include "transport/transport.h"

#include "physics/World.hh"

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
  this->connections.push_back( event::Events::ConnectRender(
        boost::bind(&CameraSensor::Render, this)) );
  this->connections.push_back( event::Events::ConnectPostRender(
        boost::bind(&CameraSensor::PostRender, this)) );
}

//////////////////////////////////////////////////////////////////////////////
// Destructor
CameraSensor::~CameraSensor()
{
}

//////////////////////////////////////////////////////////////////////////////
/// Set the parent of the sensor
void CameraSensor::SetParent( const std::string &_name )
{
  Sensor::SetParent(_name);
}
   
//////////////////////////////////////////////////////////////////////////////
// Load the camera with SDF parameters
void CameraSensor::Load( sdf::ElementPtr &_sdf )
{
  Sensor::Load(_sdf);
}

//////////////////////////////////////////////////////////////////////////////
/// Load the camera using default parameters
void CameraSensor::Load()
{
  Sensor::Load();
  this->poseSub = this->node->Subscribe("~/pose", 
                                        &CameraSensor::OnPose, this );
}
 
//////////////////////////////////////////////////////////////////////////////
// Initialize the camera
void CameraSensor::Init()
{
  //gzerr << "CameraSensor::Init()\n";

  std::string worldName = this->sdf->GetWorldName();

  if (!worldName.empty())
  {
    rendering::ScenePtr scene = rendering::RenderEngine::Instance()->GetScene(worldName);
    if (!scene)
      scene = rendering::RenderEngine::Instance()->CreateScene(worldName);

    this->camera = scene->CreateCamera(this->sdf->GetValueString("name"), false);
    if (!this->camera)
    {
      gzerr << "Unable to create camera sensor[mono_camera]\n";
      return;
    }
    this->camera->SetCaptureData(true);

    sdf::ElementPtr cameraSdf = this->sdf->GetOrCreateElement("camera");
    this->camera->Load( cameraSdf );

    // Do some sanity checks
    if (this->camera->GetImageWidth() == 0 || 
        this->camera->GetImageHeight() == 0)
    {
      gzthrow("image has zero size");
    }

    this->camera->Init();
    this->camera->CreateRenderTexture(this->GetName() + "_RttTex");
    //this->camera->SetRelativePose( this->pose );
    //this->camera->SetInitialRelativePose( this->pose );
    this->camera->SetWorldPose( this->pose );
    this->camera->AttachToVisual( this->parentName, true );
  }
  else
    gzerr << "No world name\n";

  Sensor::Init();
  gzerr << "CameraSensor pose ["<< this->pose << "]\n";

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
}

//////////////////////////////////////////////////////////////////////////////
// Update the drawing
void CameraSensor::Update(bool force)
{
  Sensor::Update(force);
}

void CameraSensor::OnPose(const boost::shared_ptr<msgs::Pose const> &/*_msg*/)
{
}
 
////////////////////////////////////////////////////////////////////////////////
// Render the camera
void CameraSensor::Render()
{
  //TODO: checking if this->camera is null is not the most efficient way to do this.
  if (this->camera && this->world->GetSimTime() - this->lastUpdateTime >= this->updatePeriod)
  {
    this->camera->Render();
    this->lastUpdateTime = this->world->GetSimTime();
  }
}

////////////////////////////////////////////////////////////////////////////////
// Post Render the camera
void CameraSensor::PostRender()
{
  //TODO: checking if this->camera is null is not the most efficient way to do this.
  if (this->camera) this->camera->PostRender();
}
